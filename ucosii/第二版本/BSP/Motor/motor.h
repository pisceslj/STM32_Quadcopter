#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#define Moto_PwmMax 2000
void Moto_PwmRflash(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);
void Motor_GPIO_init(void);
void Motor_TIM_init(void);
void Motor_PWM_ALL(int );
void Motor_PWM_SEPRATE(int p1, int p2, int p3, int p4);
#endif 
