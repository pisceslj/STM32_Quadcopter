#ifndef __TIM7_H
#define __TIM7_H

#include "stm32f10x.h"
void delay_ms(u32 t);
void tim7_init(void);
void TIM7_IRQHandler(void);

#endif
