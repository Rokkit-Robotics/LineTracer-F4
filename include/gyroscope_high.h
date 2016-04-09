#ifndef INCLUDE_GYROSCOPE_HIGH
#define INCLUDE_GYROSCOPE_HIGH

#include "gyroscope.h"

struct gyro_speed {
        float x;
        float y;
        float z;
};

struct gyro_pos {
        float x;
        float y;
        float z;
};

void gyroscope_process_p(struct gyro_data *data);

void gyroscope_read_speed(struct gyro_speed *out);
void gyroscope_read_pos(struct gyro_pos *out);
void gyroscope_reset(void);

void gyroscope_calibrate(void);

#endif
