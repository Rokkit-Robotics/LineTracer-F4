#ifndef INCLUDE_ENCODER_HIGH_H
#define INCLUDE_ENCODER_HIGH_H

#include <stdint.h>

struct encoder_pos {
        float x;
        float y;
        float theta;
};

void encoder_get_pos(struct encoder_pos *data); // angle in degrees
void encoder_get_pos_radians(struct encoder_pos *data); // angle in radians
void encoder_reset_pos(void);
void encoder_set_angle_correction(float cor);

void encoder_process_p(int32_t d_left, int32_t d_right);


#endif
