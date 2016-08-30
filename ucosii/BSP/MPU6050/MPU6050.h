#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f10x.h"
void Init_MPU6050(void);
void MPU6050_WHO_AM_I(void);
void READ_MPU6050_Gyro(void);
void READ_MPU6050_Accel(void);
#endif

