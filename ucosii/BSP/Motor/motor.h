#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_GPIO_init(void);
void Motor_TIM_init(void);
void Motor_PWM_ALL(int );
void Motor_PWM_SEPRATE(int p1, int p2, int p3, int p4);
#endif 
