#include "tim7.h"
#include "MPU6050.h"
#include "BlueTooth.h"

//extern float Angle_X_Final, Angle_Y_Final; 
//extern double Angle_X_Error, Angle_Y_Error;
volatile u32 time = 0;
u32 dms=0;

void delay_ms(u32 t)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	dms=t;
	while(dms);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , DISABLE);
	TIM7->CNT=0;
}
	
void tim7_init()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef  NVIC_TIM7;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	NVIC_TIM7.NVIC_IRQChannel = TIM7_IRQn;                     
  NVIC_TIM7.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_TIM7.NVIC_IRQChannelSubPriority = 1;
  NVIC_TIM7.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_TIM7);
	
		//初始化定时器7 TIM7	 
	TIM_TimeBaseStructure.TIM_Period = 1000-1; 
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1; 	//预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	
    TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
		
    TIM_Cmd(TIM7, ENABLE);																		
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , DISABLE);	
}


