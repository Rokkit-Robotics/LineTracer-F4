#include "encoder.h"
#include "encoder_high.h"

#include "timer.h"

#include <misc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_exti.h>
#include <stm32f4xx_syscfg.h>

#include <arch/antares.h>

#include "shell.h"
#include <lib/earlycon.h>
#include <arch/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stdint.h>
#include <math.h>

static volatile int32_t m_encLeft_path = 0, m_encRight_path = 0;

static volatile uint32_t m_encLeft_prev = 0, m_encRight_prev = 0;
static volatile uint32_t m_encLeft_cur = 0, m_encRight_cur = 0;
static volatile int32_t m_encLeft_dir = 0, m_encRight_dir = 0;

static volatile int32_t m_encLeft_lastUpdate = 0, m_encRight_lastUpdate = 0;

static volatile int m_encLeft_ready = 0, m_encRight_ready = 0;

// first encoder is on PE9 and PE11
// second is on PC6 and PC7

ANTARES_INIT_LOW(encoders_gpio_init)
{
        // init encoders GPIO
        // we want encoders to work 
        GPIO_InitTypeDef enc1_init = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_9
        };

        GPIO_InitTypeDef enc2_init = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7
        };

        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

        GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);

        GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);

        GPIO_Init(GPIOE, &enc1_init);
        GPIO_Init(GPIOC, &enc2_init);
}

// first encoder is on timer1
// second is on timer8
// APB2 in STM32F4 is 84 MHz rated

ANTARES_INIT_LOW(encoders_timer_init)
{
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

        TIM_TimeBaseInitTypeDef timer = {
                .TIM_Prescaler = 168, // 1 us per tick
                .TIM_CounterMode = TIM_CounterMode_Up,
                .TIM_ClockDivision = TIM_CKD_DIV1,
                .TIM_Period = 0xFFFF,
                .TIM_RepetitionCounter = 0
        };

        TIM_TimeBaseInit(TIM1, &timer);
        TIM_TimeBaseInit(TIM8, &timer);

        TIM_ICInitTypeDef ic = {
                .TIM_ICPolarity = TIM_ICPolarity_BothEdge,
                .TIM_Channel = TIM_Channel_1,
                .TIM_ICSelection = TIM_ICSelection_DirectTI,
                .TIM_ICPrescaler = TIM_ICPSC_DIV1,
                .TIM_ICFilter = 1
        };

        TIM_ICInit(TIM1, &ic);
        TIM_ICInit(TIM8, &ic);

        NVIC_InitTypeDef tim_int = {
                .NVIC_IRQChannel = TIM1_CC_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 1,
                .NVIC_IRQChannelSubPriority = 0,
                .NVIC_IRQChannelCmd = ENABLE
        };

        NVIC_Init(&tim_int); // for TIM1

        tim_int.NVIC_IRQChannel = TIM8_CC_IRQn;
        NVIC_Init(&tim_int); // for TIM8

        TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
        TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);

        TIM_Cmd(TIM1, ENABLE);
        TIM_Cmd(TIM8, ENABLE);
}

ANTARES_INIT_LOW(encoders_exti_init)
{
        return; // no EXTI now

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

        EXTI_InitTypeDef exti = {
                .EXTI_Line = EXTI_Line6,
                .EXTI_Mode = EXTI_Mode_Interrupt,
                .EXTI_Trigger = EXTI_Trigger_Rising_Falling,
                .EXTI_LineCmd = ENABLE
        };

        NVIC_InitTypeDef nvic = {
                .NVIC_IRQChannel = EXTI9_5_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 0x02,
                .NVIC_IRQChannelSubPriority = 0x00,
                .NVIC_IRQChannelCmd = ENABLE
        };

        NVIC_Init(&nvic);

        /* nvic.NVIC_IRQChannel = EXTI15_10_IRQn; */
        /* NVIC_Init(&nvic); */

        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);
        EXTI_Init(&exti);

        exti.EXTI_Line = EXTI_Line5;
        SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource5);
        EXTI_Init(&exti);
}

static void _enc_getPath_r(int mode, int loop) 
{
        do {
                if (mode == 1) { // left
                        printf("%ld\n", enc_getPath(LEFT));
                } else if (mode == 2) { // right
                        printf("%ld\n", enc_getPath(RIGHT));
                } else { // both
                        printf("%ld %ld\n", enc_getPath(LEFT), enc_getPath(RIGHT));
                }
                delay_ms(40);
        } while (loop && !early_avail());
        early_getc();
}

static void _enc_getSpeed_r(int mode, int loop)
{
        do {
                if (mode == 1) { // left
                        printf("%g\n", enc_getSpeed(LEFT));
                } else if (mode == 2) { // right
                        printf("%g\n", enc_getSpeed(RIGHT));
                } else { // both
                        printf("%g %g\n", enc_getSpeed(LEFT), enc_getSpeed(RIGHT));
                }
                delay_ms(40);
        } while (loop && !early_avail());
        early_getc();
}

static void _enc_getTimer_r(int mode, int loop)
{
        do {
                if (mode == 1) {
                        printf("tim%ld sys%ld\n", TIM_GetCounter(TIM1), timer_getMillis());
                } else if (mode == 2) {
                        printf("tim%ld sys%ld\n", TIM_GetCounter(TIM8), timer_getMillis());
                } else {
                        printf("left%ld right%ld sys%ld\n", TIM_GetCounter(TIM1), TIM_GetCounter(TIM8), timer_getMillis());
                }
                /* delay_ms(40); */
        } while (loop && !early_avail());
        early_getc();
}

void encoders_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [path|speed|reset] [left|right|both] [noloop]\n", argv[0]);
                return;
        }

        int loop = 1;
        int mode = 3; // 1 - left, 2 - right, 3 - both

        if (argc >= 4 && !strcmp(argv[3], "noloop")) {
                loop = 0;
        }

        if (argc >= 3) {
                if (!strcmp(argv[2], "left")) {
                        mode = 1;
                } else if (!strcmp(argv[2], "right")) {
                        mode = 2;
                } else {
                        mode = 3;
                }
        }


        if (!strcmp(argv[1], "reset")) {
                if (mode == 3 || mode == 1)
                        enc_reset(LEFT);
                if (mode == 3 || mode == 2)
                        enc_reset(RIGHT);
        } else if (!strcmp(argv[1], "path")) {
                _enc_getPath_r(mode, loop);
        } else if (!strcmp(argv[1], "speed")) {
                _enc_getSpeed_r(mode, loop);
        } else if (!strcmp(argv[1], "timer")) {
                _enc_getTimer_r(mode, loop);
        }
        
}

ANTARES_INIT_HIGH(encoders_shell_init)
{
        shell_register("encoders", encoders_callback);
}

static inline void enc_process_left()
{
        if (m_encLeft_ready < 2)
                m_encLeft_ready++;

        int pin = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_6);
        if (pin)
                GPIO_SetBits(GPIOD, GPIO_Pin_14);
        else
                GPIO_ResetBits(GPIOD, GPIO_Pin_14);

        m_encLeft_prev = m_encLeft_cur;
        m_encLeft_cur = TIM_GetCapture1(TIM1);

        if (timer_getMillis() - m_encLeft_lastUpdate >= ENC_ZERO_DELAY)
                m_encLeft_ready = 0;

        m_encLeft_lastUpdate = timer_getMillis();

#ifndef CONFIG_ENC_LEFT_INV
        if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) ^ GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))
#else
        if (!(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) ^ GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)))
#endif
        {
                m_encLeft_path++;
                m_encLeft_dir = 1; // forward
#ifndef CONFIG_ENC_SWAP
                encoder_process_p(1, 0);
#else
                encoder_process_p(0, 1);
#endif
        } else {
                m_encLeft_path--;
                m_encLeft_dir = -1; // backward
#ifndef CONFIG_ENC_SWAP
                encoder_process_p(-1, 0);
#else
                encoder_process_p(0, -1);
#endif
        }
}

static inline void enc_process_right()
{
        if (m_encRight_ready < 2)
                m_encRight_ready++;

        GPIO_ToggleBits(GPIOD, GPIO_Pin_13);

        m_encRight_prev = m_encRight_cur;
        m_encRight_cur = TIM_GetCapture1(TIM8);

        if (timer_getMillis() - m_encRight_lastUpdate >= ENC_ZERO_DELAY)
                m_encRight_ready = 0;

        m_encRight_lastUpdate = timer_getMillis();

#ifndef CONFIG_ENC_RIGHT_INV
        if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) ^ GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))
#else
        if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6) ^ GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)))
#endif
        {
                m_encRight_path++;
                m_encRight_dir = 1; // forward
#ifndef CONFIG_ENC_SWAP
                encoder_process_p(0, 1);
#else
                encoder_process_p(1, 0);
#endif
        } else {
                m_encRight_path--;
                m_encRight_dir = -1; // backward
#ifndef CONFIG_ENC_SWAP
                encoder_process_p(0, -1);
#else
                encoder_process_p(-1, 0);
#endif
        }
}

void TIM1_CC_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) {
                GPIO_ToggleBits(GPIOD, GPIO_Pin_12);
                enc_process_left();
                TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
        }
}

void TIM8_CC_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) {
                GPIO_ToggleBits(GPIOD, GPIO_Pin_15);
                enc_process_right();
                TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);
        }
}

void enc_disableInterrupt(void)
{
        NVIC_DisableIRQ(TIM1_CC_IRQn);
        NVIC_DisableIRQ(TIM8_CC_IRQn);
}

void enc_enableInterrupt(void)
{
        NVIC_EnableIRQ(TIM1_CC_IRQn);
        NVIC_EnableIRQ(TIM8_CC_IRQn);
}

float enc_getSpeed(int channel)
{
        int32_t cur, prev, dir, lastUpdate;
        int ready;

#ifdef CONFIG_ENC_SWAP
        if (channel == RIGHT)
#else
        if (channel == LEFT) 
#endif
        {
                NVIC_DisableIRQ(TIM1_CC_IRQn);
                lastUpdate = m_encLeft_lastUpdate;
                cur = m_encLeft_cur;
                prev = m_encLeft_prev;
                dir = m_encLeft_dir;
                ready = m_encLeft_ready;
                NVIC_EnableIRQ(TIM1_CC_IRQn);
        } else {
                NVIC_DisableIRQ(TIM8_CC_IRQn);
                lastUpdate = m_encRight_lastUpdate;
                cur = m_encRight_cur;
                prev = m_encRight_prev;
                dir = m_encRight_dir;
                ready = m_encRight_ready;
                NVIC_EnableIRQ(TIM8_CC_IRQn);
        }

        if (ready < 2 || timer_getMillis() - lastUpdate > ENC_ZERO_DELAY) {
                return 0.0; // remove first speed blow
                /* return 80000; */
        }

        uint32_t delta = cur > prev ? cur - prev : 0xFFFFul + cur - prev;

        /* printf("c%ld p%ld d%ld ", cur, prev, timer_getMillis() - lastUpdate); */

        if (delta == 0)
                delta = 1;

        float spd = 2 * M_PI / CONFIG_ENC_RESOLUTION / delta * 1e6;
        
        /* return delta; */
        /* return spd; */

        if (dir < 0)
                return -spd;
        else
                return spd;
}

int32_t enc_getPath(int channel)
{
#ifdef CONFIG_ENC_SWAP
        if (channel == RIGHT)
#else
        if (channel == LEFT) 
#endif
        {
                return m_encLeft_path;
        } else {
                return m_encRight_path;
        }
}

void enc_reset(int channel)
{
#ifdef CONFIG_ENC_SWAP
        if (channel == RIGHT)
#else
        if (channel == LEFT) 
#endif
        {
                m_encLeft_path = 0;
        } else {
                m_encRight_path = 0;
        }
}
