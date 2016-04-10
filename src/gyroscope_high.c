#include "gyroscope_high.h"

#include <stm32f4xx.h>
#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include "eeprom.h"

#include "shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define RING_SIZE 5

static volatile struct gyro_ringbuf {
        struct gyro_data data[RING_SIZE];
        int end;
        int size;
} ringbuf;

static volatile struct gyro_data m_filteredValue, *m_calibratedValue, *m_calibratedDisp;

static volatile struct gyro_pos m_integratedValue;

ANTARES_INIT_LOW(gyro_high_init)
{
        // init "ringbuffer"
        ringbuf.end = 0;
        ringbuf.size = 0;

        // get calibrated value from eeprom
        m_calibratedValue = (struct gyro_data *) eeprom_getptr(0x20); // let this offset be
        m_calibratedDisp = (struct gyro_data *) eeprom_getptr(0x10);
}

static void ring_push(const struct gyro_data *data)
{
        memcpy((struct gyro_data *) &ringbuf.data[ringbuf.end], data, sizeof (struct gyro_data));
        ringbuf.end++;

        if (ringbuf.end == RING_SIZE)
                ringbuf.end = 0;

        if (ringbuf.size != RING_SIZE)
                ringbuf.size++;
}

static void bsort(int16_t *data, int size)
{
        for (int i = 0; i < size; i++) {
                for (int j = i + 1; j < size; j++) {
                        if (data[i] > data[j]) {
                                int16_t t = data[i];
                                data[i] = data[j];
                                data[j] = t;
                        }
                }
        }
}

static void ring_getMed(struct gyro_data *data)
{
        int s = ringbuf.size;

        int16_t raw_data[RING_SIZE];

        if (s == 1) {
                memcpy(data, (struct gyro_data *) &ringbuf.data[0], sizeof (struct gyro_data));
                return;
        } else if (s == 2) {
                data->x = (ringbuf.data[0].x + ringbuf.data[1].x) / 2;
                data->y = (ringbuf.data[0].y + ringbuf.data[1].y) / 2;
                data->z = (ringbuf.data[0].z + ringbuf.data[1].z) / 2;
        } else { // s >= 3
                // we need to sort values

                // process x
                for (int i = 0; i < s; i++) {
                        raw_data[i] = ringbuf.data[i].x;
                }
                // sort data array
                bsort(raw_data, s);
                // give x
                data->x = raw_data[s / 2];

                // process y
                for (int i = 0; i < s; i++) {
                        raw_data[i] = ringbuf.data[i].y;
                }
                bsort(raw_data, s);
                data->y = raw_data[s / 2];

                // process y
                for (int i = 0; i < s; i++) {
                        raw_data[i] = ringbuf.data[i].z;
                }
                bsort(raw_data, s);
                data->z = raw_data[s / 2];
        }

}

void gyroscope_process_p(struct gyro_data *data)
{
        // push current value to the buffer
        ring_push(data);

        // get median value (filtered)
        ring_getMed((struct gyro_data *) &m_filteredValue);

        // move it to center using calibrated value
        m_filteredValue.x -= m_calibratedValue->x;
        m_filteredValue.y -= m_calibratedValue->y;
        m_filteredValue.z -= m_calibratedValue->z;

        if (labs(m_filteredValue.x) < m_calibratedDisp->x) {
                m_filteredValue.x = 0;
        }
        if (labs(m_filteredValue.y) < m_calibratedDisp->y) {
                m_filteredValue.y = 0;
        }
        if (labs(m_filteredValue.z) < m_calibratedDisp->z) {
                m_filteredValue.z = 0;
        }

        // scale it and integrate
#if (CONFIG_GYRO_SCALE == 250)
        const float scale = 0.00875;
#elif (CONFIG_GYRO_SCALE == 500)
        const float scale = 0.0175;
#else
        const float scale = 0.07;
#endif

        m_integratedValue.x += m_filteredValue.x * scale / CONFIG_GYRO_RATE;
        m_integratedValue.y += m_filteredValue.y * scale / CONFIG_GYRO_RATE;  
        m_integratedValue.z += m_filteredValue.z * scale / CONFIG_GYRO_RATE;  
}

void gyroscope_read_speed(struct gyro_speed *output)
{
        // scale raw data
#if (CONFIG_GYRO_SCALE == 250)
        const float scale = 0.00875;
#elif (CONFIG_GYRO_SCALE == 500)
        const float scale = 0.0175;
#else
        const float scale = 0.07;
#endif

        // disable DMA interrupt just not to corrupt data
        NVIC_DisableIRQ(DMA1_Stream0_IRQn);
        output->x = scale * m_filteredValue.x;
        output->y = scale * m_filteredValue.y;
        output->z = scale * m_filteredValue.z;
        NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

void gyroscope_read_pos(struct gyro_pos *output)
{
        // disable DMA interrupt just not to corrupt data
        NVIC_DisableIRQ(DMA1_Stream0_IRQn);
        memcpy(output, (struct gyro_pos *) &m_integratedValue, sizeof (struct gyro_pos));
        NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

void gyroscope_reset(void)
{
        // disable DMA interrupt just not to corrupt data
        NVIC_DisableIRQ(DMA1_Stream0_IRQn);
        m_integratedValue.x = 0.0;
        m_integratedValue.y = 0.0;
        m_integratedValue.z = 0.0;
        NVIC_EnableIRQ(DMA1_Stream0_IRQn);
}

void gyroscope_calibrate(void)
{
        printf("Keep robot still...\n");

        struct {
                int32_t val;
                int32_t min;
                int32_t max;
        } cal_x, cal_y, cal_z;

        cal_x.val = 0;
        cal_y.val = 0;
        cal_z.val = 0;

        for (int i = 0; i < 512; i++) {
                struct gyro_data *raw = gyro_getData();
                int32_t x = raw->x;
                int32_t y = raw->y;
                int32_t z = raw->z;

                if (i == 0) {
                        cal_x.min = x;
                        cal_x.max = x;
                        cal_y.min = y;
                        cal_y.max = y;
                        cal_z.min = z;
                        cal_z.max = z;
                } else {
                        if (x > cal_x.max)
                                cal_x.max = x;
                        if (x < cal_x.min)
                                cal_x.min = x;
                        if (y > cal_y.max)
                                cal_y.max = y;
                        if (y < cal_y.min)
                                cal_y.min = y;
                        if (z > cal_z.max)
                                cal_z.max= z;
                        if (z < cal_z.min)
                                cal_z.min = z;
                }
                cal_x.val += x;
                cal_y.val += y;
                cal_z.val += z;

                delay_ms(2);
                if (i % 8 == 0)
                        early_putc('*');
        }

        cal_x.val /= 512;
        cal_y.val /= 512;
        cal_z.val /= 512;

        m_calibratedValue->x = cal_x.val;
        m_calibratedValue->y = cal_y.val;
        m_calibratedValue->z = cal_z.val;

        m_calibratedDisp->x = cal_x.max - cal_x.min;
        m_calibratedDisp->y = cal_y.max - cal_y.min;
        m_calibratedDisp->z = cal_z.max - cal_z.min;

        printf("\nDone. Results: %ld %ld %ld\n", cal_x.val, cal_y.val, cal_z.val);
        printf("Dispersions: %ld %ld %ld\n", cal_x.max - cal_x.min,
                        cal_y.max - cal_y.min,
                        cal_z.max - cal_z.min);

        eeprom_save();
}

