#include <arch/antares.h>
#include <arch/delay.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_rcc.h>

#include "chassis.h"
#include "encoder.h"
#include "timer.h"
#include "shell.h"

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


ANTARES_APP(blinky) {

        printf("Hello!\n");

        while (1) {
                shell_loop();
        }
        /* while (1) { */
                /* int c; */
                /* c = getchar(); */
                /* printf("I received: %d\n", c); */
        /* } */


        float l_prev = 0;
        float r_prev = 0;

        const float K = 0.4;

        /* while (1) { */
                /* printf("%g %g\n", enc_getPath(LEFT), enc_getPath(RIGHT)); */
                /* delay_ms(40); */
        /* } */

        while (1) {
                /* printk("Hello World\n"); */
                float l_spd = enc_getSpeed(LEFT);
                float r_spd = enc_getSpeed(RIGHT);

                /* printf("%g %g\n", l_prev, r_prev); */
                printf("%g %g\n", l_spd, r_spd);

                l_prev = K * l_spd + (1 - K) * l_prev;
                r_prev = K * r_spd + (1 - K) * r_prev;

                /* early_putc('H'); */
                /* putchar('H'); */
                /* GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14); */
                delay_ms(40);
        }

        /* GPIO_SetBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); */
}


