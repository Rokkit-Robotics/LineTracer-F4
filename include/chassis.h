#ifndef INCLUDE_CHASSIS_H
#define INCLUDE_CHASSIS_H

typedef enum {
        CHASSIS_LEFT,
        CHASSIS_RIGHT
} chassis_unit_t;

typedef enum {
        CHASSIS_FORWARD,
        CHASSIS_BACKWARD
} chassis_dir_t;

void chassis_cmd(int enable);

void chassis_set_dir(chassis_unit_t ch, chassis_dir_t dir);
void chassis_set_pwm(chassis_unit_t ch, int pwm);
void chassis_set_enable(chassis_unit_t ch, int enable);

void chassis_write(int left, int right);
void chassis_stop();


#endif
