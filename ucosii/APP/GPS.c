#include "includes.h"

//GPS任务50HZ(50hz 待定)
//优先级8
void Task_GPS(void *p_arg)
{
		(void)p_arg;	
		while (1)
		{
			//获取GPS@@@
	
			//目前不太清楚@@@    @。@
	 
			OSTimeDlyHMSM(0, 0,0,20);
		}
}