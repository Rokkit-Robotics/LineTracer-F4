#ifndef INCLUDE_CONTROL_H
#define INCLUDE_CONTROL_H

#include <stdint.h>

typedef void (*control_callback)(uint8_t len, const uint8_t *data);

int control_register(int cmd_id, control_callback callback);
void control_loop(int *control);

#endif
