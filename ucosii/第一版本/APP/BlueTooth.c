#include "includes.h"

//发向通信线程 ->上位机

extern OS_EVENT * PCQ;
extern void *QMessagePC[QSIZE];
extern INT8U errPCQ;

// 蓝牙通信任务50HZ 
// 优先级5
void Task_BlueTooth(void*p_arg)
{
	(void)p_arg;
	
	//蓝牙初始化 建议用LED指示下
	BLUETOOTH_GPIO_Config();
	USART3_NVIC_Config();
	u3_printf("\r\n Bluetooth init succeed!\r\n");
	
	while(1)
	{
		//消息队列操作
		//中断中实现---接受上位机消息，并发送到消息队列中

		
		//该线程也从发送消息队列中获取数据，@@@
		//发送给上位机
	 OSQAccept(PCQ,&errPCQ);
		if(errPCQ==OS_NO_ERR)
		{
		
			//printf();
		
		}
		else
		{
		
		
		}		
		OSTimeDlyHMSM(0,0,0,20);
	}
}
