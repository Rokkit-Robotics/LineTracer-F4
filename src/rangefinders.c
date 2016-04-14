#include "rangefinders.h"

#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_gpio.h>
#include <misc.h>

#include "shell.h"
#include <stdio.h>
#include <string.h>

// Rangefinders should be connected PE3 to PE6 and use TIM9
// PE3 and PE4 - triggers
// PE5 and PE6 - echoes
//
// Additional front sonar is connected to PA1 and PA2
// PA1 - trigger
// PA2 - echo (TIM5_CH3)

static volatile int32_t pulse1 = -1, pulse2 = -1, pulse3 = -1;
static volatile int32_t pulse1_out = 0, pulse2_out = 0, pulse3_out = 0;
static volatile int pulse1_ready = 0, pulse2_ready = 0, pulse3_ready = 0;

ANTARES_INIT_LOW(rangefinders_gpio_init)
{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        GPIO_InitTypeDef echoes = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6,
        };
        GPIO_Init(GPIOE, &echoes);

        echoes.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOA, &echoes);

        GPIO_InitTypeDef trigs = {
                .GPIO_Mode = GPIO_Mode_OUT,
                .GPIO_Speed = GPIO_Speed_2MHz,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4
        };
        GPIO_Init(GPIOE, &trigs);

        trigs.GPIO_Pin = GPIO_Pin_1;
        GPIO_Init(GPIOA, &trigs);

        GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
}

ANTARES_INIT_LOW(rangefinders_tim_init)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
        
        // init timer 9 to process pulses
        // for TIM9 - 84
        TIM_TimeBaseInitTypeDef timebase = {
                .TIM_Prescaler = 84 * 2,
                .TIM_CounterMode = TIM_CounterMode_Up,
                .TIM_ClockDivision = TIM_CKD_DIV1,
                .TIM_Period = 0xFFFF,
                .TIM_RepetitionCounter = 0
        };
        TIM_TimeBaseInit(TIM9, &timebase);

        timebase.TIM_Prescaler = 84;
        TIM_TimeBaseInit(TIM5, &timebase);

        // configure capture channels
        TIM_ICInitTypeDef capture = {
                .TIM_ICPolarity = TIM_ICPolarity_BothEdge,
                .TIM_Channel = TIM_Channel_1, // | TIM_Channel_2,
                .TIM_ICSelection = TIM_ICSelection_DirectTI,
                .TIM_ICPrescaler = TIM_ICPSC_DIV1,
                .TIM_ICFilter = 1
        };
        TIM_ICInit(TIM9, &capture);

        capture.TIM_Channel = TIM_Channel_2;
        TIM_ICInit(TIM9, &capture);

        capture.TIM_Channel = TIM_Channel_3;
        TIM_ICInit(TIM5, &capture);

        TIM_Cmd(TIM9, DISABLE);
        TIM_Cmd(TIM5, DISABLE);

        // enable interrupts
        TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
        TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
        TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
        TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);
}

ANTARES_INIT_LOW(rangefinders_nvic_init)
{
        NVIC_InitTypeDef nvic = {
                .NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 3,
                .NVIC_IRQChannelSubPriority = 0,
                .NVIC_IRQChannelCmd = ENABLE
        };

        NVIC_Init(&nvic);

        nvic.NVIC_IRQChannel = TIM5_IRQn;
        NVIC_Init(&nvic);
}

void rangefinders_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [cm|mm|raw] [noloop]\n", argv[0]);
        }

        int loop = 1;

        float divider = 1;
        if (argc >= 3 && !strcmp(argv[2], "noloop")) {
                loop = 0;
        }

        if (argc >= 2 && !strcmp(argv[1], "cm")) {
                divider = 58;
        } else if (argc >= 2 && !strcmp(argv[1], "mm")) {
                divider = 5.8;
        }

        do {
                printf("%g %g %g\n", rangefinders_get(1) / divider, rangefinders_get(2) / divider, rangefinders_get(3) / divider);
                delay_ms(35);
        } while (loop && !early_avail());
}

ANTARES_INIT_HIGH(rangefinders_start)
{
        pulse1_out = 0xFFFFl;
        pulse2_out = 0xFFFFl;
        shell_register("range", rangefinders_callback);
        rangefinders_measureStart();
}

static volatile enum {
        STATE_TRIGGER,
        STATE_WAIT_PULSE,
        STATE_MEASURE,
        STATE_IDLE,
        STATE_DISABLE
} m_state = STATE_DISABLE;

void rangefinders_enableInterrupt()
{
        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        NVIC_EnableIRQ(TIM5_IRQn);
}

void rangefinders_disableInterrupt()
{
        NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
        NVIC_DisableIRQ(TIM5_IRQn);
}

int32_t rangefinders_get(int channel)
{
        int32_t ret;

        rangefinders_disableInterrupt();
        
        if (channel == 1)
                ret = pulse1_out;
        else if (channel == 2)
                ret = pulse2_out;
        else
                ret = pulse3_out;

        rangefinders_enableInterrupt();

        return ret;
}

void rangefinders_measureStart(void)
{
        // disable timer
        TIM_Cmd(TIM9, DISABLE);
        TIM_Cmd(TIM5, DISABLE);

        // set triggers on
        GPIO_SetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);
        GPIO_SetBits(GPIOA, GPIO_Pin_1);

        // reset TIM9 and setup to 10 uS pulse
        TIM_SetCounter(TIM9, 0);

        TIM_ARRPreloadConfig(TIM9, DISABLE);
        TIM_SetAutoreload(TIM9, 10); // 10 uS pulse
        
        TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
        TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
        TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
        TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
        TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
        TIM_ITConfig(TIM5, TIM_IT_CC3, ENABLE);

        m_state = STATE_TRIGGER;

        // enable timer
        TIM_Cmd(TIM9, ENABLE);
}

static void process_stop_trigger(void)
{
        // disable timer
        TIM_Cmd(TIM9, DISABLE);

        // reset triggers
        GPIO_ResetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);
        GPIO_ResetBits(GPIOA, GPIO_Pin_1);

        // reconfigure TIM9 to receive pulse
        TIM_ARRPreloadConfig(TIM9, DISABLE);
        TIM_SetCounter(TIM9, 0);

        TIM_ARRPreloadConfig(TIM5, DISABLE);
        TIM_SetCounter(TIM5, 0);

        TIM_SetAutoreload(TIM9, 0xFFE0); // just less than 0xffff to be before TIM5
        TIM_SetAutoreload(TIM5, 0xFFFF);

        // clear pulse values
        pulse1 = -1;
        pulse2 = -1;
        pulse3 = -1;
        
        // reset flags
        pulse1_ready = 0;
        pulse2_ready = 0;
        pulse3_ready = 0;

        m_state = STATE_MEASURE;
        /* m_state = STATE_WAIT_PULSE; */

        // enable timer
        TIM_Cmd(TIM9, ENABLE);
        TIM_Cmd(TIM5, ENABLE);

        // now we are waiting for first input pulse
}

static void process_wait_pulse(int channel)
{
        // restart timer just to be sure
        // TODO: here could be problems with second edge
        TIM_SetCounter(TIM9, 0);
        TIM_ARRPreloadConfig(TIM9, DISABLE);

        // clear first pulse
        if (channel == 1) {
                pulse1 = 0;
        } else {
                pulse2 = 0;
        }

        m_state = STATE_MEASURE;
}

static void setup_idle(void)
{
        // stop timer
        TIM_Cmd(TIM9, DISABLE);
        TIM_Cmd(TIM5, DISABLE);

        // program it to 50 ms
        TIM_ARRPreloadConfig(TIM9, DISABLE);
        TIM_SetAutoreload(TIM9, 0xffff);
        
        // reset counter
        TIM_SetCounter(TIM9, 1);
        /* TIM_PrescalerConfig(TIM9, 84 * 4, TIM_PSCReloadMode_Immediate); */

        TIM_ITConfig(TIM9, TIM_IT_CC1, DISABLE);
        TIM_ITConfig(TIM9, TIM_IT_CC2, DISABLE);
        TIM_ITConfig(TIM5, TIM_IT_CC3, DISABLE);

        // set mode to idle
        m_state = STATE_IDLE;
        
        // start timer and wait for update interrupt
        TIM_Cmd(TIM9, ENABLE);
}

static void process_measure(int channel, int32_t value)
{
        if (channel == 1) {
                // check edge on PE5
                if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5)) { // rising - start
                        pulse1 = value;
                } else { // falling
                        if (pulse1 < 0) {
                                pulse1 = 0;
                        }
                        pulse1_out = value - pulse1;
                        pulse1_ready = 1;
                }
        } else if (channel == 2) {
                // check edge on PE6
                if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6)) { // rising
                        pulse2 = value;
                } else { // falling
                        if (pulse2 < 0) {
                                pulse2 = 0;
                        }
                        pulse2_out = value - pulse2;
                        pulse2_ready = 1;
                }
        } else if (channel == 3) {
                // check edge on PA2
                if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)) { // rising
                        pulse3 = value;
                } else {
                        if (pulse3 < 0) {
                                pulse3 = 0;
                        }
                        pulse3_out = value - pulse3;
                        pulse3_ready = 1;
                }
        }

        // all pulses are ready - switch to "idle" mode (waiting for echo)
        if (pulse1_ready && pulse2_ready && pulse3_ready) {
                setup_idle();
        }
}

// if overflow occures - just to be sure
static void process_measure_overflow(void)
{
        setup_idle();

        if (!pulse1_ready) {
                pulse1_out = 0xFFFFl;
        }

        if (!pulse2_ready) {
                pulse2_out = 0xFFFFl;
        }

        if (!pulse3_ready) {
                pulse3_out = 0xFFFFl;
        }

}

// process end of the idle - start next measure
static void process_idle(void)
{
        /* if (m_state == STATE_IDLE2) */
                rangefinders_measureStart();
        /* else */
                /* m_state = STATE_IDLE2; */
}

// interrupt handlers

void TIM5_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM5, TIM_IT_CC3) != RESET) {
                if (m_state == STATE_MEASURE) {
                        process_measure(3, TIM_GetCapture3(TIM5));
                }
                TIM_ClearITPendingBit(TIM5, TIM_IT_CC3);
        }
}

//
void TIM1_BRK_TIM9_IRQHandler(void)
{
        // process sensor 1
        if (TIM_GetITStatus(TIM9, TIM_IT_CC1) != RESET) {
                if (m_state == STATE_WAIT_PULSE) {
                        process_wait_pulse(1);
                } else if (m_state == STATE_MEASURE) {
                        process_measure(1, TIM_GetCapture1(TIM9));
                }
                TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
        }

        // process sensor 2
        else if (TIM_GetITStatus(TIM9, TIM_IT_CC2) != RESET) {
                if (m_state == STATE_WAIT_PULSE) {
                        process_wait_pulse(2);
                } else if (m_state == STATE_MEASURE) {
                        process_measure(2, TIM_GetCapture2(TIM9));
                }
                TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
        }

        // process overflow
        else if (TIM_GetITStatus(TIM9, TIM_IT_Update) != RESET) {
                TIM_ClearITPendingBit(TIM9, TIM_IT_Update);

                if (m_state == STATE_TRIGGER) {
                        process_stop_trigger();
                } else if (m_state == STATE_MEASURE) { // means overflow
                        process_measure_overflow();
                } else if (m_state == STATE_IDLE) {
                        process_idle();
                }
                
        }

        else if (TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET) {
                TIM_ITConfig(TIM1, TIM_IT_Break, DISABLE);
                TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
        }
}
