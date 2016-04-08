#ifndef INCLUDE_GYROSCOPE_H
#define INCLUDE_GYROSCOPE_H

#include <stdint.h>

struct gyro_data {
        int16_t x;
        int16_t y;
        int16_t z;
};

struct gyro_data *gyro_getData();
struct gyro_data *gyro_getData_imm();

#endif
