#include "includes.h"
extern int pwmout1, pwmout2, pwmout3, pwmout4; 
extern u8 TIM2CH1_CAPTURE_STA, TIM2CH2_CAPTURE_STA,TIM2CH3_CAPTURE_STA,TIM2CH4_CAPTURE_STA;
extern RC_GETDATA Rc_Get;//接收到的RC数据,1000~2000
extern S_FLOAT_XYZ EXP_ANGLE;
void Task_rc(void *p_arg)
{
 (void)p_arg;	
 while (1)
 {	 
//	CH4：1488-1490 yaw    pwmout1
//	CH3：1487-1490 pitch  pwmout2
//	CH2：997-1001  th   pwmout3
//	CH1：1486-1488 row  pwmout4
	Rc_Get.ROLL= pwmout4;
	Rc_Get.PITCH=pwmout2;
	Rc_Get.YAW=pwmout1;
	Rc_Get.THROTTLE=pwmout3;
	
	EXP_ANGLE.X=(Rc_Get.ROLL-1500)/30;
	EXP_ANGLE.Y=(Rc_Get.PITCH-1500)/30;
	EXP_ANGLE.Z=(Rc_Get.YAW-1500)/30;

	
 OSTimeDlyHMSM(0, 0,0,10);

 }
}


