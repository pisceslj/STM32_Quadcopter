#include "includes.h"
u8 SYS_INIT_OK = 0;
extern u8 debug;
extern float zoffset;

void BSP_Init()
{
	int i;
	float temp;
	
	/*************************
	
	传感器与硬件初始化
	
	**************************/
	
	/* 配置系统时钟为 72M  3.5库中启动文件已调用*/
	SysTick_init(); /* 初始化并使能 SysTick 定时器 */
	tim7_init();/*初始化并使能tim7*/
	LED_GPIO_Config(); /* LED 端口初始化 指示作用*/

	
	//串口1
	USART1_Config();
	USART1_NVIC_Config();
	
	//蓝牙串口3
	BLUETOOTH_GPIO_Config();
	USART3_NVIC_Config();

	//接收机tim2
	Rev_GPIO_init();
	Rev_NVIC_TIM_init();
	
	//电机
	Motor_GPIO_init();
	Motor_TIM_init();

	Motor_PWM_ALL(100);
	delay_ms(2000);
	Motor_PWM_ALL(0);
	delay_ms(2000);
	
	//PID
	Pid_init();
	
	//I2C
	I2C_GPIO_Configuration();
	
	//gy86 MPU6050 HMC5883L
	MPU6050_Init(); 
	Init_HMC5883L();

	printf("bsp init OK next\n");
	/****************************************/

		
		
	

	/*********************
	
	这一部分主要是为了稳定解算的角度，测定偏移，调整Z轴角度到0
	
	**********************/
	
	GYRO_OFFSET_OK=0; 
	ACC_OFFSET_OK=0;
	debug=0;
	
  i=0;
	while(i<1500)
	{
		Prepare_Data();
		Read_HMC5883L();
		Get_Attitude();
		i++;
	}
	
	i=0;
	while(i<10)
	{
		Prepare_Data();
		Read_HMC5883L();
		Get_Attitude();
		
		temp+=Q_ANGLE.Z;
		i++;
	}
	 zoffset=temp/10;
	
	debug=1;
	/**********************/
	
	
	printf("data init OK\n");
	u3_printf("bsp  and  data init OK\n");

	
	
	
	/***************
	片上资源闪存初始化和PID数据存取测试
	****************/
	//闪存解锁和初始化
	FLASH_Unlock();//FLASH解锁
	EE_Init();
	
	
	printf("flash init OK\n");
	u3_printf("flash init OK \n");
	
//	EE_READ_ACC_OFFSET();
//	EE_READ_GYRO_OFFSET();

		printf("data init OK\n");

		EE_READ_PID_Send();
		//已经过测试成功读取到闪存中数据
		printf("pid test end OK\n");
		u3_printf("pid test end OK\n");
}
	


void SysTick_init(void)
{
 SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);//初始化并使能 SysTick 定时器 3.5库宏定义不同
}


