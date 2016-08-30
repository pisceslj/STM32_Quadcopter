#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

//void TIM7_NVIC_Config(void);
//void delay_init(void);
//void delay(uint32_t count);

void SysTick_Init(void);
void Delay_us(__IO u32 nTime);
#define Delay_ms(x) Delay_us(1000*x)	 //µ¥Î»ms

#endif

