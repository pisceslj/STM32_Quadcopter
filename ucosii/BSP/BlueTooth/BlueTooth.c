#include "BlueTooth.h" 	
#include "control.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ucos_ii.h"
#define USART3_REC_LEN  			200 
#define USART3_TX_LEN  			200 

extern float psp,psd,psi,pcp,pcd;
extern float rsp,rsd,rsi,rcp,rcd;
extern float ysp,ysd,ysi,ycp,ycd;

//串口3中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   
u8 USART3_RX_BUF[USART3_REC_LEN]={0};     //接收缓冲,最大USART_REC_LEN个字节.
u8 USART3_TX_BUF[USART3_TX_LEN]={0};
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART3_RX_STA=0;       //接收状态标记	
u8 Res;//字节接收

//信息解码
u8 ARMED,pidset,send_angle;
//PID设置缓冲
PID P;

void BT_Send(S_FLOAT_XYZ *angleTX)
{
	printf("pitch %f;roll %f;yaw %f\n",angleTX->Y,angleTX->X,angleTX->Z);
}

void BLUETOOTH_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 配置串口3 （USART3） 时钟*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3 ,ENABLE); 
	
	/*串口GPIO端口配置*/
  /* 配置串口3 （USART3 Tx (PD8)）*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);   
	
	/* 配置串口3 USART3 Rx (PD9)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	  
	/* 串口3工作模式（USART3 mode）配置 */
	USART_InitStructure.USART_BaudRate = 9600;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//中断配置
	
	USART_Cmd(USART3, ENABLE);//使能串口
}

void USART3_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//设置中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}

void USART3_IRQHandler(void)
{
	//操作系统中断 防止嵌套任务     重调度
	OSIntEnter();
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x23结尾)
	{
		Res =USART_ReceiveData(USART3);//(USART3->DR);	//读取接收到的数据
		
		if((USART3_RX_STA&0X8000)==0)//接收未完成
		{
				if(Res=='#')
				{
					USART3_RX_STA|=0X8000;
					Command_Read();
				}
				else
				{
					USART3_RX_BUF[USART3_RX_STA++]=Res;
					if(USART3_RX_STA>(USART3_REC_LEN-1))
					{
						USART3_RX_STA=0;
					}//接收数据错误,重新开始接收	  
				}		 
			}   		 
		}
		OSIntExit();		
}

void Command_Read()
{
	char *p,*q;
	char *endptr;
	float temp;
	if(USART3_RX_STA&0x8000)
		{
       /*if(strcmp((const char*)USART3_RX_BUF,"Yes")==0)
				 u3_printf("copy");
			 else if(strcmp((const char*)USART3_RX_BUF,"STOP")==0)
			 {
			 		ARMED=0;
				  u3_printf("Stop Motor");
			 }
			 else if(strcmp((const char*)USART3_RX_BUF,"START")==0)
			 {
					ARMED=1;
				  u3_printf("Start Motor");
			 }
			 else if(strcmp((const char*)USART3_RX_BUF,"ANGLE")==0)
			 {
					send_angle=1;
			 }
			 */
			ARMED=1;
			
			temp = strtod((const char*)USART3_RX_BUF,&endptr);
			
			if(strcmp(endptr,"psp")== 0){psp = temp;u3_printf(" psp changed to%lf",psp);}
			else if(strcmp(endptr,"psd")== 0){psd = temp;u3_printf(" psd changed to%lf",psd);}
			else if(strcmp(endptr,"psi")== 0){psi = temp;u3_printf(" psi changed to%lf",psi);}
			else if(strcmp(endptr,"pcp")== 0){pcp = temp;u3_printf(" pcp changed to%lf",pcp);}
			else if(strcmp(endptr,"pcd")== 0){pcd = temp;u3_printf(" pcd changed to%lf",pcd);}
			
			else if(strcmp(endptr,"rsp")== 0){rsp = temp;u3_printf(" rsp changed to%lf",rsp);}
			else if(strcmp(endptr,"rsd")== 0){rsd = temp;u3_printf(" rsd changed to%lf",rsd);}
			else if(strcmp(endptr,"rsi")== 0){rsi = temp;u3_printf(" rsi changed to%lf",rsi);}
			else if(strcmp(endptr,"rcp")== 0){rcp = temp;u3_printf(" rcp changed to%lf",rcp);}
			else if(strcmp(endptr,"rcd")== 0){rcd = temp;u3_printf(" rcd changed to%lf",rcd);}
			
			else if(strcmp(endptr,"ysp")== 0){ysp = temp;u3_printf(" ysp changed to%lf",ysp);}
			else if(strcmp(endptr,"ysd")== 0){ysd = temp;u3_printf(" ysd changed to%lf",ysd);}
			else if(strcmp(endptr,"ysi")== 0){ysi = temp;u3_printf(" ysi changed to%lf",ysi);}
			else if(strcmp(endptr,"ycp")== 0){ycp = temp;u3_printf(" ycp changed to%lf",ycp);}
			else if(strcmp(endptr,"ycd")== 0){ycd = temp;u3_printf(" ycd changed to%lf",ycd);}
			else {u3_printf("undef info");}
			
			u3_printf("\r\n");
			USART3_RX_STA=0;
			RX_BUF_Clear();
	}
	
	
}

void RX_BUF_Clear()
{
	int i;
	for(i=0;i<USART3_REC_LEN;i++)
	{
		USART3_RX_BUF[i]=0;
	}
}

void u3_printf(char* fmt,...)  
{  
	u16 i,j; 
	va_list ap; 
	va_start(ap,fmt);
	vsprintf((char*)USART3_TX_BUF,fmt,ap);
	va_end(ap);
	i=strlen((const char*)USART3_TX_BUF);		//此次发送数据的长度
	for(j=0;j<i;j++)							//循环发送数据
	{
	  while(USART_GetFlagStatus(USART3,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
		USART_SendData(USART3,USART3_TX_BUF[j]); 
	} 
}

char * strmid(char *dst,const char *src, int n,int m) 
{
    const char *p = src;
    char *q = dst;
    int len = strlen(src);
    if(n>len) n = len-m;    
    if(m<0) m=0;    
    if(m>len) return NULL;
    p += m;
    while(n--) *(q++) = *(p++);
    *(q++)='\0';
    return dst;
}





