#ifndef __BSP_H
#define __BSP_H

#include "stm32f10x_rcc.h"
#include "bsp_led.h" 				//LED 驱动函数
#include "BlueTooth.h" 			//蓝牙通信函数
#include "GPS.h"						//gps
#include "I2cdev.h"					//i2c
#include "motor.h"					//电机
#include "kalman.h"					//滤波算法
#include "MPU6050.h"				//mpu6050
#include "HMC5883L.h"				//hmc5883l
#include "Rev.h"						//接收器
#include "tim7.h"						//TIM7定时器
#include "usart1.h"					//串口1

void SysTick_init(void);
void BSP_Init(void);        //用于设置系统时钟，初始化硬件

#endif // __BSP_H

