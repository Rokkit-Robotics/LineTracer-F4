#include "encoder_high.h"

#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include "encoder.h"
#include "eeprom.h"

#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static volatile struct encoder_pos m_encPos;
static volatile float m_angle_correct = 1.0;

void enc_high_callback(int argc, char *argv[])
{
        if (argc == 1) {
                printf("Usage: %s [pos|reset|setcor|getcor] [noloop|cor]\n", argv[0]);
                return;
        }

        int loop = 1;

        if (argc >= 3) { // noloop
                if (!strcmp(argv[2], "noloop")) {
                        loop = 0;
                }
        }

        if (!strcmp(argv[1], "reset")) {
                encoder_reset_pos();
        } else if (!strcmp(argv[1], "pos")) {
                do {
                        struct encoder_pos p;
                        encoder_get_pos(&p);
                        printf("%g %g %g | %g %g\n", p.x, p.y, p.theta, p.left, p.right);
                        delay_ms(35);
                } while (loop && !early_avail());
        } else if (!strcmp(argv[1], "setcor")) {
                if (argc < 3) {
                        printf("No value!");
                        return;
                }

                float val;
                sscanf(argv[2], "%g", &val);
                encoder_set_angle_correction(val);
        } else if (!strcmp(argv[1], "getcor")) {
                printf("cor: %g\n", m_angle_correct);
                return;
        }
}

ANTARES_INIT_HIGH(enc_high_init)
{
        encoder_reset_pos();

        float *param = (float *) eeprom_getptr(0x30);
        m_angle_correct = *param;

        shell_register("inert", enc_high_callback);
}

void encoder_reset_pos(void)
{
        enc_disableInterrupt();       
        m_encPos.x = 0.0;
        m_encPos.y = 0.0;
        m_encPos.left = 0.0;
        m_encPos.right = 0.0;
        m_encPos.theta = 0.0;
        enc_enableInterrupt();
}

void encoder_reset_path(void)
{
        enc_disableInterrupt();
        m_encPos.left = 0.0;
        m_encPos.right = 0.0;
        enc_enableInterrupt();
}

void encoder_get_pos_radians(struct encoder_pos *data)
{
        enc_disableInterrupt();
        memcpy(data, (struct encoder_pos *) &m_encPos, sizeof (struct encoder_pos));
        enc_enableInterrupt();
}

void encoder_get_pos(struct encoder_pos *data)
{
        enc_disableInterrupt();
        data->x = m_encPos.x;
        data->y = m_encPos.y;

        data->left = m_encPos.left;
        data->right = m_encPos.right;

        data->theta = m_encPos.theta;
        enc_enableInterrupt();
        
        data->theta = data->theta / M_PI * 180;
}

void encoder_set_angle_correction(float cor)
{
        printf("Setting encoder error correction to %g...\n", cor);
        m_angle_correct = cor;

        *((float *) eeprom_getptr(0x30)) = cor;
        eeprom_save();
}

// This function is running in interrupt context
void encoder_process_p(int32_t di_left, int32_t di_right)
{
        // process delta from ticks to millimeters
        float d_left = di_left * M_PI * CONFIG_ENC_WHEEL_DIAM / CONFIG_ENC_RESOLUTION;
        float d_right = di_right * M_PI * CONFIG_ENC_WHEEL_DIAM / CONFIG_ENC_RESOLUTION;

        m_encPos.left += d_left;
        m_encPos.right += d_right;

        // calculate dtheta
        float dtheta = (d_right - d_left) / ((float) CONFIG_BASE_DIAM);

        dtheta *= m_angle_correct;
        m_encPos.theta += dtheta;

        // calculate dx and dy
        float dx = (d_left + d_right) / 2 * cos(m_encPos.theta);
        float dy = (d_left + d_right) / 2 * sin(m_encPos.theta);

        m_encPos.x += dx;
        m_encPos.y += dy;
}
