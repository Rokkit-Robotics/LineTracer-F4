#include "gyroscope_high.h"

#include <stm32f4xx.h>
#include <arch/antares.h>
#include <arch/delay.h>
#include <lib/earlycon.h>

#include "eeprom.h"

#include "shell.h"
#include <stdio.h>
#include <string.h>

#define RING_SIZE 5

static volatile struct gyro_ringbuf {
        struct gyro_data data[RING_SIZE];
        int end;
        int size;
} ringbuf;

static volatile struct gyro_data m_filteredValue, *m_calibratedValue;

static volatile struct gyro_pos m_integratedValue;

ANTARES_INIT_LOW(gyro_high_init)
{
        // init "ringbuffer"
        ringbuf.end = 0;
        ringbuf.size = 0;

        // get calibrated value from eeprom
        m_calibratedValue = (struct gyro_data *) eeprom_getptr(0x20); // let this offset be
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

        int32_t cal_x = 0, cal_y = 0, cal_z = 0;

        for (int i = 0; i < 512; i++) {
                struct gyro_data *raw = gyro_getData();
                cal_x += raw->x;
                cal_y += raw->y;
                cal_z += raw->z;

                delay_ms(2);
                if (i % 8 == 0)
                        early_putc('*');
        }

        cal_x /= 512;
        cal_y /= 512;
        cal_z /= 512;

        m_calibratedValue->x = cal_x;
        m_calibratedValue->y = cal_y;
        m_calibratedValue->z = cal_z;

        printf("\nDone. Results: %ld %ld %ld\n", cal_x, cal_y, cal_z);

        eeprom_save();
}

