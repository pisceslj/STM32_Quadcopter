#include "includes.h"

//发向姿态控制

extern OS_EVENT * PIDQ;
extern void *QMessagePID[QSIZE];
extern INT8U errPIDQ;

//姿态控制PID任务100HZ
//优先级7
void Task_atttitude_pid(void *p_arg)
{
 (void)p_arg;	
 while (1)
 {
	 OSQAccept(PIDQ,&errPIDQ);
	 if(errPIDQ==OS_NO_ERR)
	 {
		 
	//四元数转化为欧拉角@@@
	 
	//中断中处理好的接收机PPM数据转换为遥控器值@@@
	 
	//然后根据遥控器油门值、目标位置和上位机命令控制四个电机转速@@@
	 }
	 else
	 {
	 
	 }	 
 OSTimeDlyHMSM(0, 0,0,10);

 }
}
