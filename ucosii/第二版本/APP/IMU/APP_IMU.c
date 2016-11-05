#include "includes.h"
//姿态解算任务500HZ
//优先级6


void Task_atttitude_computation(void *p_arg)
{
	while (1)
 {
	Prepare_Data();
	Get_Attitude();
  OSTimeDlyHMSM(0,0,0,2);
 }
}

