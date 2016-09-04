#include "includes.h"

/*************************变量******************************/
extern int pwmout1, pwmout2, pwmout3, pwmout4; 				//输出占空比
extern u16 USART3_RX_STA;
extern u16 USART2_RX_STA;

extern uint16_t GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT,ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT,MP6050_Temperature;//X,Y,Z轴，温度
extern float Angle_X_Error, Angle_Y_Error,Angle_Z_Error;
extern int Magn_x,Magn_y,Magn_z;
extern float angle;		//正北方向(约地磁南极)为标准 方位角

//姿态解算任务250HZ
//优先级6
void Task_atttitude_computation(void *p_arg)
{
 int i;
 float x,y,z,magn_x,magn_y,magn_z;
	(void)p_arg;	
		
	//I2C初始化
	I2C_GPIO_Configuration();
	OSTimeDlyHMSM(0,0,0,500);
	Init_MPU6050();
	//u3_printf("\r\n MPU6050 init succeed!\r\n");		
		//I2C初始化
	I2C_GPIO_Configuration();
	OSTimeDlyHMSM(0,0,0,500);
	Init_MPU6050();
	//u3_printf("\r\n MPU6050 init succeed!\r\n");
		
	//接收机初始化
	Rev_GPIO_init();
	Rev_NVIC_TIM_init();
	//u3_printf("\r\n Receiver init succeed!\r\n");
		
	//e-switcher初始化
	Motor_PWM_ALL(100);
	OSTimeDlyHMSM(0,0,0,2000);
	Motor_PWM_ALL(0);
	OSTimeDlyHMSM(0,0,0,2000);
	//u3_printf("\r\n e-switcher init succeed!\r\n");
		
	//检测接收机
	if((pwmout1<1500)&&(pwmout1>1480)&&(pwmout2<990)&&(pwmout2>1010)&&(pwmout4<1500)&&(pwmout4>1480)&&(pwmout3<1500)&&(pwmout3>1480))
	{
	//	u3_printf("\r\n Receiver works good!\r\n");
	}
	else
	{
	//	u3_printf("\r\n Receiver doesn't work!\r\n");
	}
	
	Init_MPU6050();
	Init_HMC5883L();
	
	/********电子罗盘必须水平测试方位角误差***************/
		for(i=0;i<2000;i++){
			Read_HMC5883L();
			magn_x=Magn_x;
			magn_y=Magn_y;    
			magn_z=Magn_z;
			if(magn_x<32764) x=magn_x/16384;
			else              x=(32768-magn_x)/16384;
				
			if(magn_y<32764) y=magn_y/16384;
			else              y=(32768-magn_y)/16384;//1-(Accel_y-49152)/16384;	
			angle+=atan2(y,x) * (180 / 3.14159265);
		}
		angle=angle/2000;
		
		/********/
	
		for(i=0;i<2000;i++){
			READ_MPU6050_Gyro();
			READ_MPU6050_Accel();	
			Read_HMC5883L();
			Angle_Calcu();
		  
			Angle_X_Error += Angle_X_Final;
			Angle_Y_Error += Angle_Y_Final;
			Angle_Z_Error	+= Angle_Z_Final;
		}
//			Angle_X_Error/=2000;
//			Angle_Y_Error/=2000;
//			Angle_Z_Error/=2000;
	while (1)
 {
	//获取参数
	READ_MPU6050_Gyro();
	READ_MPU6050_Accel();	
	Read_HMC5883L();
	//卡尔曼滤波
	 
	 Angle_Calcu();
	 
	 //向姿态控制发送消息@@@
	 //OSQPost(PIDQ,  void * );
	 
	 
  OSTimeDlyHMSM(0, 0,0,4);
 }
}

