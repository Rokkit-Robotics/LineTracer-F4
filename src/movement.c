#include "movement.h"

#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include <misc.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_tim.h>

#include "encoder_high.h"
#include "gyroscope_high.h"
#include "chassis.h"

#include "eeprom.h"
#include "timer.h"

#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// we will use TIM2 for movement stabilisation loop
 
static volatile int32_t counter = 0;

static volatile struct move_settings {
        float p;
        float i;
        float d;
        float enc_weight;
        float gyro_weight;
        float smooth;
        int32_t min_speed;
} *m_moveSettings; // number of modes

static volatile struct move_line {
        /* float base_angle; */
        int32_t speed;
        float distance;

        int32_t base_speed;
} m_moveLine;

static volatile enum {
        MODE_LINE = 0,
        MODE_LINE_SPEED = 1,
        MODE_ROTATE = 2,
        MODE_STOP = 3,
        MODE_LAST = 4
} m_mode, m_cfgMode;

static volatile int m_isBusy = 0;

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
        if (argc == 0) {
                printf("No command!\n");
                return;
        }

        if (!strcmp(argv[0], "line")) {
                // need 2 args: speed and distance
                if (argc < 3) {
                        printf("Line command args: [speed] [distance]\n");
                        return;
                }

                int32_t speed = 0;
                float distance = 0;

                sscanf(argv[1], "%ld", &speed);
                sscanf(argv[2], "%g", &distance);

                movement_start_line(speed, distance, 1);
        }
}

void movement_start_line(int32_t speed, float distance, int console)
{
        m_moveLine.speed = speed;
        m_moveLine.distance = distance;

        gyroscope_reset();
        encoder_reset_path();

        // set flag to start moving
        m_isBusy = 1;
        m_mode = MODE_LINE;

        int32_t start = timer_getMillis();

        while (m_isBusy && console && !early_avail()) {
                struct encoder_pos enc;
                encoder_get_pos(&enc);

                printf("%ld %g %g\n", timer_getMillis() - start, enc.left, enc.right);
        }

        chassis_write(0, 0);
        m_isBusy = 0;
}

void movement_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [set|get|cmd|mode] [cmd|p|i|d|enc|gyro|smooth|min_speed] [value]\n", argv[0]);
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
                if (mode < 0 && mode >= MODE_LAST) {
                        printf("Wrong value, correct from 1 to %d\n", MODE_LAST);
                        return;
                }

                m_cfgMode = mode;
                return;
        } else if (!strcmp(argv[1], "get")) {
                if (argc < 3) { // print all
                        printf("mode %d\np %g\ni %g\nd %g\nenc %g\ngyro %g\nsmooth %g\nmin_speed %ld\n",
                                        m_cfgMode + 1,
                                        m_moveSettings[m_cfgMode].p,
                                        m_moveSettings[m_cfgMode].i,
                                        m_moveSettings[m_cfgMode].d,
                                        m_moveSettings[m_cfgMode].enc_weight,
                                        m_moveSettings[m_cfgMode].gyro_weight,
                                        m_moveSettings[m_cfgMode].smooth,
                                        m_moveSettings[m_cfgMode].min_speed);
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
                        m_moveSettings[m_cfgMode].p = value;
                } else if (!strcmp(argv[2], "i")) {
                        m_moveSettings[m_cfgMode].i = value;
                } else if (!strcmp(argv[2], "d")) {
                        m_moveSettings[m_cfgMode].d = value;
                } else if (!strcmp(argv[2], "enc")) {
                        m_moveSettings[m_cfgMode].enc_weight = value;
                } else if (!strcmp(argv[2], "gyro")) {
                        m_moveSettings[m_cfgMode].gyro_weight = value;
                } else if (!strcmp(argv[2], "smooth")) {
                        m_moveSettings[m_cfgMode].smooth = value;
                } else if (!strcmp(argv[2], "min_speed")) {
                        m_moveSettings[m_cfgMode].min_speed = value;
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

// Handling line
// Params: base speed, base angle, distance
void handle_line_speed(void)
{
        int mode = MODE_LINE_SPEED;
        
        struct encoder_pos enc;
        encoder_get_pos(&enc);

        float d = (enc.left + enc.right) / 2;
        float path = m_moveSettings[mode].smooth;

        // check if we are stopping - more important
        if (fabs(m_moveLine.distance - d) < m_moveSettings[mode].smooth) { 
                path = m_moveLine.distance - d;
        } else if (fabs(d) < m_moveSettings[mode].smooth) {
                path = d;
        }

        // required speed is a gradation
        float spd = m_moveSettings[mode].min_speed + (m_moveLine.speed - m_moveSettings[mode].min_speed) * 
                (path / m_moveSettings[mode].smooth);
        
        int speed = spd;

        m_moveLine.base_speed = speed;
}

void handle_line(void)
{
        static float integral = 0.0, prev_error = 0.0;

        const float dt = 1.0 / CONFIG_MOVE_FQ;
        const int mode = MODE_LINE;

        // get errors from encoder and gyro
        struct encoder_pos enc;
        struct gyro_pos gyro;

        encoder_get_pos(&enc);
        gyroscope_read_pos(&gyro);
        
        // if we have reached destination - time to stop
        if (enc.left + enc.right > 2 * m_moveLine.distance) {
                chassis_write(0, 0);
                integral = 0;
                prev_error = 0;

                m_isBusy = 0;
                return;
        }

        // error from encoder - path difference
        float enc_error = enc.left - enc.right;

        float error = m_moveSettings[mode].enc_weight * enc_error + m_moveSettings[mode].gyro_weight * gyro.z;

        integral += error * dt;

        float deriv = (error - prev_error) / dt;
        prev_error = error;

        float regulation = m_moveSettings[mode].p * error + m_moveSettings[mode].i * integral +
                + m_moveSettings[mode].d * deriv;

        int reg = (int) regulation;

        chassis_write(m_moveLine.base_speed - reg, m_moveLine.base_speed + reg);
}

void handle_rotate(void)
{
        
}

void handle_stop(void)
{

}

void TIM2_IRQHandler(void)
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
                
                if (m_isBusy) {
                        if (m_mode == MODE_LINE) {
                                handle_line_speed();
                                handle_line();
                        } else if (m_mode == MODE_ROTATE) {
                                /* handle_rotate_speed(); */
                                handle_rotate();
                        } else if (m_mode == MODE_STOP) {
                                handle_stop();
                        }
                }

                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        }
}
