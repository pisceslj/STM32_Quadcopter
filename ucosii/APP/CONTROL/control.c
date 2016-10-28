#include "includes.h"
extern int pwmout1, pwmout2, pwmout3, pwmout4; 
extern RC_GETDATA Rc_Get;//接收到的RC数据,1000~2000
extern S_FLOAT_XYZ EXP_ANGLE,DIF_ANGLE,Q_ANGLE;

extern PID PID_ROL,PID_PIT,PID_YAW;



extern OS_EVENT * ANGLE;  //信号量
extern INT8U errANGLE;
//姿态控制PID任务100HZ
//优先级7
void Task_atttitude_pid(void *p_arg)
{
	u3_printf("pid start OK\n");
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

	
	EXP_ANGLE.X=(Rc_Get.ROLL-1500)/20;
	EXP_ANGLE.Y=(Rc_Get.PITCH-1500)/20;
	EXP_ANGLE.Z=(Rc_Get.YAW-1500)/20;

	OSMutexPend(ANGLE,0, &errANGLE); 
	
	DIF_ANGLE.X=EXP_ANGLE.X-Q_ANGLE.X;
	DIF_ANGLE.Y=EXP_ANGLE.Y-Q_ANGLE.Y;
	DIF_ANGLE.Z=EXP_ANGLE.Z-Q_ANGLE.Z;
	 
	OSMutexPost(ANGLE);
	 
	CONTROL(DIF_ANGLE.X,DIF_ANGLE.Y,DIF_ANGLE.Z);
  OSTimeDlyHMSM(0, 0, 0, 10);
 }
}