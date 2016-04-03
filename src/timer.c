#include "timer.h"

#include <stm32f4xx.h>
#include <arch/antares.h>

static volatile int32_t millis = 0;

ANTARES_INIT_LOW(systick_init)
{
        SysTick_Config(SystemCoreClock / 1000);
}

void SysTick_Handler(void)
{
        millis++;
}

int32_t timer_getMillis(void)
{
        return millis;
}
