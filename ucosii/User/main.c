#include "includes.h"
#include "MainQ.h"
static OS_STK startup_task_stk[STARTUP_TASK_STK_SIZE]; //定义主任务栈

int main(void)
{
	BSP_Init(); //板级支持包初始化
	OSInit();   //操作系统初始化
	
	//消息队列
	MainQ=OSQCreate(QMessageMain,QSIZE);
	PIDQ=OSQCreate(QMessagePID,QSIZE);
	PCQ=OSQCreate(QMessagePC,QSIZE);
	
	OSTaskCreate(Task_Start,(void *)0,&startup_task_stk[STARTUP_TASK_STK_SIZE-1], STARTUP_TASK_PRIO);//创建任务

	OSStart();  //启动调度
	
	return 0;
}
