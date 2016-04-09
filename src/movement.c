#include "movement.h"

#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include <misc.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>

#include "encoder_high.h"
#include "gyroscope_high.h"

#include "eeprom.h"

#include "shell.h"
#include <stdio.h>
#include <string.h>

// we will use TIM2 for movement stabilisation loop
 
static volatile int32_t counter = 0;

static volatile struct move_settings {
        float p;
        float i;
        float d;
        float enc_weight;
        float gyro_weight;
} *m_moveSettings; // number of modes

static volatile enum {
        MODE_LINE = 0,
        MODE_ROTATE = 1
} m_mode;

ANTARES_INIT_LOW(movement_init)
{
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

        // init timer for cyclic calls
        TIM_TimeBaseInitTypeDef timer = {
                .TIM_Prescaler = 4200, // now max frequency is 10 kHz
                .TIM_Period = 10000 / CONFIG_MOVE_FQ,
                .TIM_CounterMode = TIM_CounterMode_Up,
                .TIM_ClockDivision = TIM_CKD_DIV1,
                .TIM_RepetitionCounter = 0
        };

        TIM_TimeBaseInit(TIM2, &timer);

        // init timer interrupt
        TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

        NVIC_InitTypeDef tim_nvic = {
                .NVIC_IRQChannel = TIM2_IRQn,
                .NVIC_IRQChannelPreemptionPriority = 5,
                .NVIC_IRQChannelSubPriority = 0,
                .NVIC_IRQChannelCmd = ENABLE
        };

        NVIC_Init(&tim_nvic);

        // enable timer
        TIM_Cmd(TIM2, ENABLE);
}

void movement_cmd_callback(int argc, char *argv[])
{

}

void movement_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [set|get|cmd|mode] [cmd|p|i|d|enc|gyro] [value]\n", argv[0]);
                return;
        }

        if (!strcmp(argv[1], "cmd")) {
                movement_cmd_callback(argc - 2, &argv[2]);
                return;
        } else if (!strcmp(argv[1], "mode")) {
                if (argc > 3) {
                        printf("No value!");
                        return;
                }
                
                int mode = -1;
                sscanf(argv[2], "%d", &mode);
                mode--;
                if (mode < 0 && mode > 1) {
                        printf("Wrong value, correct 1 or 2\n");
                        return;
                }

                m_mode = mode;
                return;
        } else if (!strcmp(argv[1], "get")) {
                if (argc < 3) { // print all
                        printf("mode %d\np %g\ni %g\nd %g\nenc %g\ngyro %g\n",
                                        m_mode + 1,
                                        m_moveSettings[m_mode].p,
                                        m_moveSettings[m_mode].i,
                                        m_moveSettings[m_mode].d,
                                        m_moveSettings[m_mode].enc_weight,
                                        m_moveSettings[m_mode].gyro_weight);
                        return;
                }
        } else if (!strcmp(argv[1], "set")) {
                if (argc < 4) {
                        printf("No value!\n");
                        return;
                }

                float value = 0.0;
                sscanf(argv[3], "%g", &value);

                if (!strcmp(argv[2], "p")) {
                        m_moveSettings[m_mode].p = value;
                } else if (!strcmp(argv[2], "i")) {
                        m_moveSettings[m_mode].i = value;
                } else if (!strcmp(argv[2], "d")) {
                        m_moveSettings[m_mode].d = value;
                } else if (!strcmp(argv[2], "enc")) {
                        m_moveSettings[m_mode].enc_weight = value;
                } else if (!strcmp(argv[2], "gyro")) {
                        m_moveSettings[m_mode].gyro_weight = value;
                } else {
                        printf("Unknown option: %s\n", argv[2]);
                }
        }
}

ANTARES_INIT_HIGH(movement_callback_init)
{
        shell_register("move", movement_callback);
        m_moveSettings = (struct move_settings *) eeprom_getptr(0x100);
}

void TIM2_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
                counter++;

                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        }
}
