#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

void BLUETOOTH_GPIO_Config(void);
void USART3_NVIC_Config(void);
void u3_printf(char* fmt,...);
void USART3_IRQHandler(void);
void Command_Read(void);
void RX_BUF_Clear(void);
#endif
