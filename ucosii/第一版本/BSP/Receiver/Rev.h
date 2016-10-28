#ifndef __REV_H
#define __REV_H

#include "stm32f10x.h"
#include "includes.h"
void Rev_GPIO_init(void);
void Rev_NVIC_TIM_init(void);
void TIM2_IRQHandler(void);
#endif
