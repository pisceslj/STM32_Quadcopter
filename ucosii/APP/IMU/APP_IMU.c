#include "includes.h"
//��̬��������500HZ
//���ȼ�6


void Task_atttitude_computation(void *p_arg)
{
	while (1)
 {
	Prepare_Data();
	Get_Attitude();
  OSTimeDlyHMSM(0,0,0,2);
 }
}
