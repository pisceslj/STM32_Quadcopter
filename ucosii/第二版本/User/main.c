#include "includes.h"
__align(8) static OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE]; //定义主任务栈

OS_EVENT * ANGLE;
INT8U errANGLE;
INT8U	prio=4;

int main(void)
{
	BSP_Init();
	OSInit();
	
	ANGLE=OSMutexCreate(prio,&errANGLE);//控制角度访问
	OSTaskCreate(Task_Start,(void *)0,
	&startup_task_stk[STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO);
	printf("bsp init OK\n");
	OSStart();
	return 0;
 }
