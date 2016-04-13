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

// Rangefinders should be connected PE3 to PE6
// PE3 and PE4 - triggers
// PE5 and PE6 - echoes

static volatile int32_t pulse1 = -1, pulse2 = -1;
static volatile int32_t pulse1_out = 0, pulse2_out = 0;
static volatile int pulse1_ready = 0, pulse2_ready = 0;

ANTARES_INIT_LOW(rangefinders_gpio_init)
{
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

        GPIO_InitTypeDef echoes = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6,
        };
        GPIO_Init(GPIOE, &echoes);

        GPIO_InitTypeDef trigs = {
                .GPIO_Mode = GPIO_Mode_OUT,
                .GPIO_Speed = GPIO_Speed_2MHz,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_OType = GPIO_OType_PP,
                .GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4
        };
        GPIO_Init(GPIOE, &trigs);

        GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
}

ANTARES_INIT_LOW(rangefinders_tim_init)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
        
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

        TIM_Cmd(TIM9, DISABLE);

        // enable interrupts
        TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
        TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);
        TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
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
}

void rangefinders_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [noloop]\n", argv[0]);
        }

        int loop = 1;
        if (argc >= 2 && !strcmp(argv[1], "noloop")) {
                loop = 0;
        }

        do {
                printf("%ld %ld\n", rangefinders_get(2), rangefinders_get(1));
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
        STATE_IDLE1,
        STATE_IDLE2,
        STATE_DISABLE
} m_state = STATE_DISABLE;

void rangefinders_enableInterrupt()
{
        NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
}

void rangefinders_disableInterrupt()
{
        NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
}

int32_t rangefinders_get(int channel)
{
        int32_t ret;

        rangefinders_disableInterrupt();
        
        if (channel == 1)
                ret = pulse1_out;
        else
                ret = pulse2_out;

        rangefinders_enableInterrupt();

        return ret;
}

void rangefinders_measureStart(void)
{
        // disable timer
        TIM_Cmd(TIM9, DISABLE);

        // set both triggers on
        GPIO_SetBits(GPIOE, GPIO_Pin_3 | GPIO_Pin_4);

        // reset TIM9 and setup to 10 uS pulse
        TIM_SetCounter(TIM9, 0);
        TIM_SetAutoreload(TIM9, 10); // 10 uS pulse
        TIM_ARRPreloadConfig(TIM9, DISABLE);
        
        TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
        TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
        TIM_ITConfig(TIM9, TIM_IT_CC1, ENABLE);
        TIM_ITConfig(TIM9, TIM_IT_CC2, ENABLE);

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

        // reconfigure TIM9 to receive pulse
        TIM_SetCounter(TIM9, 0);
        /* TIM_PrescalerConfig(TIM9, 84, TIM_PSCReloadMode_Immediate); */
        TIM_SetAutoreload(TIM9, 0xFFFF);
        TIM_ARRPreloadConfig(TIM9, DISABLE);

        // clear both pulse values
        pulse1 = -1;
        pulse2 = -1;
        
        // reset flags
        pulse1_ready = 0;
        pulse2_ready = 0;

        m_state = STATE_MEASURE;
        /* m_state = STATE_WAIT_PULSE; */

        // enable timer
        TIM_Cmd(TIM9, ENABLE);

        // now we are waiting for first input pulse
}

static void process_wait_pulse(int channel)
{
        // restart timer just to be sure
        // TODO: here could be problems with second edge
        TIM_SetCounter(TIM9, 0);
        TIM_ARRPreloadConfig(TIM9, ENABLE);

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

        // reset counter
        TIM_SetCounter(TIM9, 0);

        // program it to 50 ms
        /* TIM_SetAutoreload(TIM9, 50000); */
        /* TIM_PrescalerConfig(TIM9, 84 * 4, TIM_PSCReloadMode_Immediate); */
        /* TIM_ARRPreloadConfig(TIM9, DISABLE); */

        TIM_ITConfig(TIM9, TIM_IT_CC1, DISABLE);
        TIM_ITConfig(TIM9, TIM_IT_CC2, DISABLE);

        // set mode to idle
        m_state = STATE_IDLE1;
        
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
        }

        // all pulses are ready - switch to "idle" mode (waiting for echo)
        /* if (pulse1_ready && pulse2_ready) { */
                /* setup_idle(); */
        /* } */
}

// if overflow occures - just to be sure
static void process_measure_overflow(void)
{
        /* setup_idle(); */

        if (!pulse1_ready) {
                pulse1_out = 0xFFFFl;
        }

        if (!pulse2_ready) {
                pulse2_out = 0xFFFFl;
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

// interrupt handler
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
                /* } else if (m_state == STATE_IDLE1 || m_state == STATE_IDLE2) { */
                        process_idle();
                }
                
        }

        else if (TIM_GetITStatus(TIM1, TIM_IT_Break) != RESET) {
                TIM_ITConfig(TIM1, TIM_IT_Break, DISABLE);
                TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
        }
}
