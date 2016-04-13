#ifndef INCLUDE_RANGEFINDERS_H
#define INCLUDE_RANGEFINDERS_H

#include <stdint.h>

int32_t rangefinders_get(int channel);

void rangefinders_disableInterrupt(void);
void rangefinders_enableInterrupt(void);

void rangefinders_measureStart(void);

#endif
