#include "shell.h"

#include "chassis.h"

#include <arch/antares.h>
#include <lib/earlycon.h>

void kbctl_callback(int argc, char *argv[])
{
        printf("WASD - direction\n"
                "Space - stop\n"
                "1234 - speed levels\n");
        printf("Press 'p' to exit.\n");

        int c;
        int speed = 2048;

        do {
                c = early_getc();

                switch (c) {
                        case 'p':
                                break;
                        case '1':
                                speed = 1024;
                                break;
                        case '2':
                                speed = 2048;
                                break;
                        case '3':
                                speed = 3072;
                                break;
                        case '4':
                                speed = 4096;
                                break;
                        case 'w':
                                chassis_write(speed, speed);
                                break;
                        case 'a':
                                chassis_write(-speed, speed);
                                break;
                        case 's':
                                chassis_write(-speed, -speed);
                                break;
                        case 'd':
                                chassis_write(speed, -speed);
                                break;
                        case ' ':
                                chassis_write(0, 0);
                                break;
                        default:
                                break;
                }

        } while (c != 'p');
}


ANTARES_INIT_HIGH(kbctl_init)
{
        shell_register("kbctl", kbctl_callback);
}
