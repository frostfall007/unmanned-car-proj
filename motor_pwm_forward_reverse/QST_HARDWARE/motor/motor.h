#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void motor_init(void);
void motor_forward(u8 duty_percent);
void motor_reverse(u8 duty_percent);
void motor_stop(void);

#endif
