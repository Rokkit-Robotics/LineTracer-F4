#include <arch/antares.h>

#include "chassis.h"
#include "shell.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>

/**
 * TIM3 is used for PWM generation to motors (channels 3 and 4)
 */

/**
 * Time base init structure for TIM3
 */
static TIM_TimeBaseInitTypeDef TIM3_InitStruct = {
        .TIM_Prescaler = 0, // do not divide clock frequency
        .TIM_CounterMode = TIM_CounterMode_Up,
        .TIM_Period = 4096,
        .TIM_ClockDivision = TIM_CKD_DIV1,
        .TIM_RepetitionCounter = 0
};

/**
 * Output compare block init structure
 */
static TIM_OCInitTypeDef TIM3_OCInitStruct = {
        .TIM_OCMode = TIM_OCMode_PWM1, // clear on compare match
        .TIM_OutputState = TIM_OutputState_Enable,
        .TIM_OCPolarity = TIM_OCPolarity_Low,
        .TIM_Pulse = 4096 // inverted PWM
};

/**
 * GPIO init structures
 */
static GPIO_InitTypeDef GPIO_PWMInitStruct = {
        .GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_NOPULL,
        .GPIO_Mode = GPIO_Mode_AF,
        .GPIO_Speed = GPIO_Speed_100MHz
};

// On GPIOB:
// PB13 = M1_DISABLE
// PB15 = M1_DIR
static GPIO_InitTypeDef GPIOB_OutInitStruct = {
        .GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_NOPULL,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_Speed = GPIO_Speed_100MHz
};

// On GPIOD:
// PD10 - ENABLE
// PD9 - M2_DISABLE
// PD11 - M2_DIR
static GPIO_InitTypeDef GPIOD_OutInitStruct = {
        .GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11,
        .GPIO_OType = GPIO_OType_PP,
        .GPIO_PuPd = GPIO_PuPd_NOPULL,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_Speed = GPIO_Speed_100MHz
};

ANTARES_INIT_LOW(chassis_init)
{
        /* Timer init */

        // Init clock for TIM3, it's placed on APB1
        // APB1 frequency by default is as SYSCLK (168 MHz)
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

        // Init TIM3 timer base
        TIM_TimeBaseInit(TIM3, &TIM3_InitStruct);

        // Init PWM modules
        TIM_OC3Init(TIM3, &TIM3_OCInitStruct);
        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

        TIM_OC4Init(TIM3, &TIM3_OCInitStruct);
        TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
        
        TIM_ARRPreloadConfig(TIM3, ENABLE);
        TIM_Cmd(TIM3, ENABLE);

        /* PWM GPIO init */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

        GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);

        GPIO_Init(GPIOC, &GPIO_PWMInitStruct);

        /* Control GPIO init */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

        GPIO_Init(GPIOB, &GPIOB_OutInitStruct);
        GPIO_Init(GPIOD, &GPIOD_OutInitStruct);
}

void chassis_callback(int argc, char *argv[]);

ANTARES_INIT_HIGH(chassis_shell_init)
{
        shell_register("chassis", chassis_callback);
}

void chassis_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [write|stop|enable|disable]\n", argv[0]);
                return;
        }

        if (!strcmp(argv[1], "write")) {
                if (argc != 4) {
                        printf("Usage: %s write _left _right\n", argv[0]);
                        return;
                }

                int left = atoi(argv[2]);
                int right = atoi(argv[3]);

                chassis_write(left, right);
        } else if (!strcmp(argv[1], "stop")) {
                chassis_write(0, 0);
        } else if (!strcmp(argv[1], "enable")) {
                chassis_cmd(1);
        } else if (!strcmp(argv[1], "disable")) {
                chassis_cmd(0);
        }
}


void chassis_set_dir(chassis_unit_t u, chassis_dir_t dir)
{
#ifndef CONFIG_CHASSIS_INV
        if (u == CHASSIS_LEFT) {
#else
        if (u == CHASSIS_RIGHT) {
#endif

#ifndef CONFIG_CHASSIS_LEFT_INV
                if (dir == CHASSIS_FORWARD) {
#else
                if (dir == CHASSIS_BACKWARD) {
#endif
                        GPIO_SetBits(GPIOB, GPIO_Pin_15);
                } else {
                        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
                }
        } else {
#ifndef CONFIG_CHASSIS_RIGHT_INV
                if (dir == CHASSIS_FORWARD) {
#else
                if (dir == CHASSIS_BACKWARD) {
#endif
                        GPIO_SetBits(GPIOD, GPIO_Pin_11);
                } else {
                        GPIO_ResetBits(GPIOD, GPIO_Pin_11);
                }
        }
}

void chassis_set_enable(chassis_unit_t u, int enable)
{
#ifndef CONFIG_CHASSIS_INV
        if (u == CHASSIS_LEFT) {
#else
        if (u == CHASSIS_RIGHT) {
#endif
                if (enable) {
                        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
                } else {
                        GPIO_SetBits(GPIOB, GPIO_Pin_13);
                }
        } else {
                if (enable) {
                        GPIO_ResetBits(GPIOD, GPIO_Pin_9);
                } else {
                        GPIO_SetBits(GPIOD, GPIO_Pin_9);
                }
        }
}

void chassis_set_pwm(chassis_unit_t u, int pwm) 
{
        if (pwm < 0)
                pwm = 0;
        else if (pwm > 4096)
                pwm = 4096;

        TIM3_OCInitStruct.TIM_Pulse = pwm;

#ifndef CONFIG_CHASSIS_INV
        if (u == CHASSIS_LEFT) {
#else
        if (u == CHASSIS_RIGHT) {
#endif
                TIM_OC3Init(TIM3, &TIM3_OCInitStruct);
                TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
        } else {
                TIM_OC4Init(TIM3, &TIM3_OCInitStruct);
                TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
        }
}

void chassis_cmd(int enable)
{
        if (enable) {
                GPIO_SetBits(GPIOD, GPIO_Pin_10);
        } else {
                GPIO_ResetBits(GPIOD, GPIO_Pin_10);
        }
}

void chassis_write(int left, int right)
{
        if (left < 0) {
                left = -left;
#ifndef CONFIG_CHASSIS_LEFT_INV
                left = 4096 - left;
#endif
                chassis_set_dir(CHASSIS_LEFT, CHASSIS_BACKWARD);
        } else {
#ifdef CONFIG_CHASSIS_LEFT_INV
                left = 4096 - left;
#endif
                chassis_set_dir(CHASSIS_LEFT, CHASSIS_FORWARD);
        }

        if (right < 0) {
                right = -right;
#ifndef CONFIG_CHASSIS_RIGHT_INV
                right = 4096 - right;
#endif
                chassis_set_dir(CHASSIS_RIGHT, CHASSIS_BACKWARD);
        } else {
#ifdef CONFIG_CHASSIS_RIGHT_INV
                right = 4096 - right;
#endif
                chassis_set_dir(CHASSIS_RIGHT, CHASSIS_FORWARD);
        }
        
        chassis_set_enable(CHASSIS_LEFT, 0);
        chassis_set_enable(CHASSIS_RIGHT, 0);

        chassis_set_pwm(CHASSIS_LEFT, left);
        chassis_set_pwm(CHASSIS_RIGHT, right);
}
