#include "includes.h"


//姿态解算任务250HZ
//优先级6
void Task_atttitude_computation(void *p_arg)
{
	int i;
  float x,y,magn_x,magn_y;
	//float z,magn_z;
  (void)p_arg;			
	while (1)
 {

	 
	 //向姿态控制发送消息@@@
	 //OSQPost(PIDQ,  void * );
	 
	 
  OSTimeDlyHMSM(0, 0,0,4);
 }
}