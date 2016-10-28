#ifndef _UART_UART_H_
#define _UART_UART_H_
#include "stm32f10x.h"
#include "bsp/usart.h"

void Uart_CheckEvent(void);
void PC_Debug_Show(u8 num,u16 sta);
void Uart1_Send_Buf(u8 *buf,u8 num);
void Uart1_Send_RCdata(void);
void Uart1_Send_AF(void);
void Uart1_Send_AE(void);
extern char flag_send;
void BlueToothCheckEvent(void);
#endif
