#include "includes.h"

/********************定义任务栈****************************/
OS_STK task_bluetooth_stk[TASK_STK_SIZE];
OS_STK task_atttitude_computation_stk[TASK_STK_SIZE];
OS_STK task_atttitude_pid_stk[TASK_STK_SIZE];
OS_STK task_gps_stk[TASK_STK_SIZE];

OS_STK task_led2_stk[TASK_LED2_STK_SIZE];
OS_STK task_led3_stk[TASK_LED3_STK_SIZE]; 


/*************************变量******************************/
extern int pwmout1, pwmout2, pwmout3, pwmout4; 				//输出占空比
extern u16 USART3_RX_STA;
extern u16 USART2_RX_STA;

extern uint16_t GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT,ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT,MP6050_Temperature;//X,Y,Z轴，温度
extern float Angle_X_Error, Angle_Y_Error,Angle_Z_Error;
extern int Magn_x,Magn_y,Magn_z;
extern float angle;	//正北方向(约地磁南极)为标准 方位角

//发向主任务

extern OS_EVENT * MainQ;
extern void *QMessageMain[QSIZE];
extern INT8U errMainQ;

//发向姿态控制

extern OS_EVENT * PIDQ;
extern void *QMessagePID[QSIZE];
extern INT8U errPIDQ;

//发向通信线程 ->上位机

extern OS_EVENT * PCQ;
extern void *QMessagePC[QSIZE];
extern INT8U errPCQ;


 //主任务20HZ
void Task_Start(void *p_arg)
{
 (void)p_arg; // 'p_arg' 并没有用到，防止编译器提示警告

	 
	//读取flash中预设参数@@@
 
 
	
	//创建其他任务@@@
	OSTaskCreate(Task_BlueTooth,(void *)0, //创建蓝牙通信任务
	&task_bluetooth_stk[TASK_STK_SIZE-1], TASK_BLUETOOTH_PRIO);
	
	OSTaskCreate(Task_atttitude_computation,(void *)0, //创建姿态解算任务
	&task_atttitude_computation_stk[TASK_STK_SIZE-1], TASK_ATTITUDE_COMPUTATION_PRIO);
	
	OSTaskCreate(Task_atttitude_pid,(void *)0, //创建姿态控制任务
	&task_atttitude_pid_stk[TASK_STK_SIZE-1], TASK_ATTITUDE_PID_PRIO);
	
	OSTaskCreate(Task_atttitude_pid,(void *)0, //创建gps任务
	&task_gps_stk[TASK_STK_SIZE-1], TASK_GPS_PRIO);
	
	
	//DEBUG
	OSTaskCreate(Task_LED2,(void *)0, //创建任务 2
	&task_led2_stk[TASK_LED2_STK_SIZE-1], TASK_LED2_PRIO);

	OSTaskCreate(Task_LED3,(void *)0, //创建任务 3
	&task_led3_stk[TASK_LED3_STK_SIZE-1], TASK_LED3_PRIO);
 //读取flash中预设参数@@@
 
 


 while (1)
 {
	 
	 
	 //从消息队列中获取上位机命令做出响应,数据放入消息队列并发送给上位机
	 //接受信息 主要是PID参数的接收
	 OSQAccept(MainQ,&errMainQ);	 
	 
	 if(errMainQ==OS_NO_ERR)
	 {
		  //接受到信息处理@@@
		 
	 }
	 else
	 {
	 //未接收到信息@@@
	 }
	 
	 //向上位机发送消息@@@ 
	 //目前不知道要发送什么信息@。@
	 //OSQPost(PCQ,  void * );
	 
	 
	 //向姿态控制发出信息@@@
	 //OSQPost(PIDQ,  void * );
	 
	 
	 //
	LED1( ON );
	OSTimeDlyHMSM(0, 0,0,50);
 }
}



// 蓝牙通信任务50HZ 
// 优先级5
void Task_BlueTooth(void*p_arg)
{
	(void)p_arg;
	
	//蓝牙初始化 建议用LED指示下
	BLUETOOTH_GPIO_Config();
	USART3_NVIC_Config();
	u3_printf("\r\n Bluetooth init succeed!\r\n");
	
	while(1)
	{
		//消息队列操作
		//中断中实现---接受上位机消息，并发送到消息队列中

		
		//该线程也从发送消息队列中获取数据，@@@
		//发送给上位机
	 OSQAccept(PCQ,&errPCQ);
		if(errPCQ==OS_NO_ERR)
		{
		
			//printf();
		
		}
		else
		{
		
		
		}		
		OSTimeDlyHMSM(0,0,0,20);
	}
}
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

//GPS任务50HZ(50hz 待定)
//优先级8
void Task_GPS(void *p_arg)
{
 (void)p_arg;	
 while (1)
 {
	//获取GPS@@@
	
	 //目前不太清楚@@@    @。@
	 
	 
 OSTimeDlyHMSM(0, 0,0,20);

 }
}




//DEBUG
//任务 2
void Task_LED2(void *p_arg)
{
 (void)p_arg;

 while (1)
 {
 LED2( ON );
 OSTimeDlyHMSM(0, 0,0,200);
 LED2( OFF);
 OSTimeDlyHMSM(0, 0,0,200);
 }
}

//任务 3
void Task_LED3(void *p_arg)
{
 (void)p_arg;

 while (1)
 {
 LED3( ON );
 OSTimeDlyHMSM(0, 0,0,300);
 LED3( OFF);
 OSTimeDlyHMSM(0, 0,0,300);
 }
}