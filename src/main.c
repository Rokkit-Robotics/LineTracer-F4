#include <arch/antares.h>
#include <arch/delay.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

#include "chassis.h"
#include "encoder.h"
#include "timer.h"

#include <stdio.h>

static GPIO_InitTypeDef GPIO_InitStruct = {
	.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15,
	.GPIO_Speed = GPIO_Speed_100MHz,
	.GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_PuPd = GPIO_PuPd_NOPULL
};


ANTARES_INIT_LOW(gpio_init)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}


#ifdef CONFIG_ANTARES_STARTUP
ANTARES_APP(blinky) {
#else
int main() {
	gpio_init();	
#endif

        chassis_cmd(1); // enable chassis
        /* chassis_write(1024, 1024); */
        /* chassis_write(2500, 2500); */

        while (1) {
                /* printk("Hello World\n"); */
                printf("[%ld] Left encoder speed: %g, path %ld\n", timer_getMillis(), enc_getSpeed(LEFT), enc_getPath(LEFT));
                /* early_putc('H'); */
                /* putchar('H'); */
                GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14);
                delay_ms(40);
        }

        /* GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); */

        for (int i = 0; i < 4096; i++) {
                chassis_write(i, i);
                delay_ms(1);
        }

        for (int i = 4096; i > -4096; i--) {
                chassis_write(i, i);
                delay_ms(1);
        }

        for (int i = -4096; i < 0; i++) {
                chassis_write(i, i);
                delay_ms(1);
        }

        /* GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); */
        /* delay_ms(500); */
        /* GPIO_ResetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); */
        /* delay_ms(500); */
}


