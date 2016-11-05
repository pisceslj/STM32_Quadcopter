#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H
#include"IMU.h"
void BLUETOOTH_GPIO_Config(void);
void USART3_NVIC_Config(void);
void u3_printf(char* fmt,...);
void USART3_IRQHandler(void);
void Command_Read(void);
void RX_BUF_Clear(void);
void BT_Send(S_FLOAT_XYZ* angleTX);
#endif
