#ifndef __REV_H
#define __REV_H
#include "stm32f10x.h"
#include "includes.h"

typedef struct int16_rcget{
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
}RC_GETDATA;

extern RC_GETDATA Rc_Get;//接收到的RC数据,1000~2000

extern unsigned char NRF24L01_RXDATA[32];

void Nrf_Check_Event(void);
void NRF_Send_AF(void);
void NRF_Send_AE(void);
void NRF_Send_OFFSET(void);
void NRF_Send_PID(void);
void NRF_Send_ARMED(void);
void NRF_SEND_test(void);
void NRF_Control(void);



void Rev_GPIO_init(void);
void Rev_NVIC_TIM_init(void);
void TIM2_IRQHandler(void);
#endif
