#include "includes.h"

//DEBUG
//任务 2
void Task_LED2(void *p_arg)
{
		(void)p_arg;    //防止编译器提示警告
		while (1)
		{
				LED2( ON );
				OSTimeDlyHMSM(0, 0,0,200);
				LED2( OFF );
				OSTimeDlyHMSM(0, 0,0,200);
		}
}

//任务 3
void Task_LED3(void *p_arg)
{
			(void)p_arg;	 //防止编译器提示警告
			while (1)
			{
					LED3( ON );
					OSTimeDlyHMSM(0, 0,0,300);
					LED3( OFF);
					OSTimeDlyHMSM(0, 0,0,300);
			}
}


