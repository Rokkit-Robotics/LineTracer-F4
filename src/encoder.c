#include "encoder.h"

#include "timer.h"

#include <misc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>

#include <arch/antares.h>

#include <stdint.h>
#include <math.h>

static volatile int32_t m_encLeft_path = 0, m_encRight_path = 0;

static volatile int32_t m_encLeft_prev = 0, m_encRight_prev = 0;
static volatile int32_t m_encLeft_cur = 0, m_encRight_cur = 0;
static volatile int32_t m_encLeft_dir = 0, m_encRight_dir = 0;

static volatile int32_t m_encLeft_lastUpdate = 0, m_encRight_lastUpdate = 0;

/* static volatile float m_encLeft_spd = 0.0, m_encRight_spd = 0.0; */

// first encoder is on PE9 and PE11
// second is on PC6 and PC7

ANTARES_INIT_LOW(encoders_gpio_init)
{
        // init encoders GPIO
        // we want encoders to work 
        GPIO_InitTypeDef enc1_init = {
                .GPIO_Mode = GPIO_Mode_AF,
                .GPIO_PuPd = GPIO_PuPd_NOPULL,
                .GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11
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

        TIM_TimeBaseInitTypeDef timebase = {
                .TIM_Prescaler = 84, // to get  tick per us
                .TIM_CounterMode = TIM_CounterMode_Up,
                .TIM_Period = 0xFFFF,
                .TIM_ClockDivision = TIM_CKD_DIV1, // 21 MHz to timer
                .TIM_RepetitionCounter = 0
        };

        TIM_TimeBaseInit(TIM1, &timebase);
        TIM_TimeBaseInit(TIM8, &timebase);

        // configure input capture channels
        TIM_ICInitTypeDef capture = {
                .TIM_Channel = TIM_Channel_1,
                .TIM_ICPolarity = TIM_ICPolarity_BothEdge,
                .TIM_ICSelection = TIM_ICSelection_DirectTI,
                .TIM_ICPrescaler = TIM_ICPSC_DIV1,
                .TIM_ICFilter = 0
        };

        TIM_ICInit(TIM1, &capture);
        TIM_ICInit(TIM8, &capture);

        // configure capture interrupts
        TIM_ITConfig(TIM1, TIM_IT_CC1, ENABLE);
        TIM_ITConfig(TIM8, TIM_IT_CC1, ENABLE);

        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

        NVIC_InitTypeDef tim_irq = {
                .NVIC_IRQChannel = TIM1_CC_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 4,
                .NVIC_IRQChannelSubPriority = 0,
                .NVIC_IRQChannelCmd = ENABLE
        };
        NVIC_Init(&tim_irq);

        tim_irq.NVIC_IRQChannel = TIM8_CC_IRQn;
        NVIC_Init(&tim_irq);

        TIM_Cmd(TIM1, ENABLE);
        TIM_Cmd(TIM8, ENABLE);
}

void TIM1_CC_IRQHandler(void) 
{
        
        if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) {
                TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);


                m_encLeft_prev = m_encLeft_cur;
                m_encLeft_cur = TIM_GetCapture1(TIM1);

                m_encLeft_lastUpdate = timer_getMillis();

#ifndef ENC_LEFT_INVERT
                if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) ^ GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11)) {
#else
                if (!(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) ^ GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11))) {
#endif
                        m_encLeft_path++;
                        m_encLeft_dir = 1; // forward
                } else {
                        m_encLeft_path--;
                        m_encLeft_dir = -1; // backward
                }

                if (TIM_GetFlagStatus(TIM1, TIM_FLAG_CC1OF) != RESET) {
                        TIM_ClearFlag(TIM1, TIM_FLAG_CC1OF);
                        GPIO_SetBits(GPIOD, GPIO_Pin_15);
                        // ? debug
                } else {
                        GPIO_ResetBits(GPIOD, GPIO_Pin_15);
                }
        }
}

void TIM8_CC_IRQHandler(void) 
{
        if (TIM_GetITStatus(TIM8, TIM_IT_CC1) != RESET) {
                TIM_ClearITPendingBit(TIM8, TIM_IT_CC1);

                m_encRight_prev = m_encRight_cur;
                m_encRight_cur = TIM_GetCapture1(TIM8);

                m_encRight_lastUpdate = timer_getMillis();

#ifndef ENC_RIGHT_INVERT
                if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) ^ GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8)) {
#else
                if (!(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7) ^ GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8))) {
#endif
                        m_encRight_path++;
                        m_encRight_dir = 1;
                } else {
                        m_encRight_path--;
                        m_encRight_dir = -1;
                }

                if (TIM_GetFlagStatus(TIM8, TIM_FLAG_CC1OF) != RESET) {
                        TIM_ClearFlag(TIM8, TIM_FLAG_CC1OF);
                        // ? debug
                }
        }
}

float enc_getSpeed(int channel)
{
        int32_t cur, prev, dir;

        if (channel == LEFT) {
                if (timer_getMillis() - m_encLeft_lastUpdate > ENC_ZERO_DELAY)
                        return 0.0;

                /* __set_BASEPRI(4 << (8 - __NVIC_PRIO_BITS)); */
                /* NVIC_DisableIRQ(TIM1_CC_IRQn); */
                cur = m_encLeft_cur;
                prev = m_encLeft_prev;
                dir = m_encLeft_dir;
                __set_BASEPRI(0U);
                /* NVIC_EnableIRQ(TIM1_CC_IRQn); */


                
        } else {
                if (timer_getMillis() - m_encRight_lastUpdate > ENC_ZERO_DELAY)
                        return 0.0;
 
                /* __set_BASEPRI(4 << (8 - __NVIC_PRIO_BITS)); */
                /* NVIC_DisableIRQ(TIM8_CC_IRQn); */
                cur = m_encRight_cur;
                prev = m_encRight_prev;
                dir = m_encRight_dir;
                __set_BASEPRI(0U);
                /* NVIC_EnableIRQ(TIM8_CC_IRQn); */

        }

        int delta = (cur >= prev) ? cur - prev : 0xFFFF - prev + cur;
        float spd = 2 * M_PI / ENC_RESOLUTION / delta * 1e6;
        
        return delta;

        if (dir < 0)
                return -spd;
        else
                return spd;
}

int32_t enc_getPath(int channel)
{
        if (channel == LEFT) {
                return m_encLeft_path;
        } else {
                return m_encRight_path;
        }
}

void enc_reset(int channel)
{
        if (channel == LEFT) {
                m_encLeft_path = 0;
        } else {
                m_encRight_path = 0;
        }
}
