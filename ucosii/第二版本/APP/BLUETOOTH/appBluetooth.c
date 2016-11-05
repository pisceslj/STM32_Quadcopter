#include "includes.h"

// 蓝牙通信任务50HZ 
// 优先级5
extern OS_EVENT * ANGLE;  //信号量
extern INT8U errANGLE;
extern int debug;
S_FLOAT_XYZ   *angleTX=&Q_ANGLE;
extern u8 ARMED,pidset,send_angle;
void Task_BlueTooth(void*p_arg)
{
	(void)p_arg;
	u3_printf("\r\n Bluetooth task succeed!\r\n");
	
	while(1)
	{
		if(debug==1) //&& send_angle==1
		{
			//加个信号量
			OSMutexPend (ANGLE,0, &errANGLE);
			BT_Send(angleTX);
			OSMutexPost (ANGLE);
		}
		if(pidset>0)
		{
		 pidset=0;
		}
		//BT_Send(angleTX);
		OSTimeDlyHMSM(0,0,1,0);
	}
}


