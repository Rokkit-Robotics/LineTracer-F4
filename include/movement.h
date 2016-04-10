#ifndef INCLUDE_MOVEMENT_H
#define INCLUDE_MOVEMENT_H

#include <stdint.h>

void movement_start_line(int32_t speed, float distance, int console);
void movement_start_rotate(int32_t speed, float delta_angle, int console);

#endif
