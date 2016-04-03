#ifndef INCLUDE_ENCODER_H
#define INCLUDE_ENCODER_H

#include <stdint.h>

#ifndef LEFT
#define LEFT 0
#endif

#ifndef RIGHT
#define RIGHT (!(LEFT))
#endif

// after this time (in ms) we think that encoder speed is 0
#define ENC_ZERO_DELAY 64

#define ENC_RESOLUTION 64

float enc_getSpeed(int channel);
int32_t enc_getPath(int channel);
void enc_reset(int channel);


#endif
