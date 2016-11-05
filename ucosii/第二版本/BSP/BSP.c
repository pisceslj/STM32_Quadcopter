#include "includes.h"
u8 SYS_INIT_OK = 0;
void BSP_Init()
{
	/* 配置系统时钟为 72M  3.5库中启动文件已调用*/
	
	
	SysTick_init(); /* 初始化并使能 SysTick 定时器 */
	tim7_init();/*初始化并使能tim7*/
	LED_GPIO_Config(); /* LED 端口初始化 指示作用*/
		//u3_printf("SYSTICK INIT OK!\n");
	
	//蓝牙
	BLUETOOTH_GPIO_Config();
	USART3_NVIC_Config();
	//u3_printf("BT INIT OK!\n");
	
	//电机
	Motor_GPIO_init();
	Motor_TIM_init();
	//u3_printf("MOTOR INIT OK!\n");
//	Motor_PWM_ALL(100);
//	delay_ms(2000);
//	Motor_PWM_ALL(0);
//	delay_ms(2000);
	
	
	
	//PID
	Pid_init();
	//u3_printf("PID INIT OK!\n");
	
	
	//接收机
	Rev_GPIO_init();
	Rev_NVIC_TIM_init();
	//u3_printf("REV INIT OK!\n");


	
	//串口1
	USART1_Config();
	USART1_NVIC_Config();
	//u3_printf("USART INIT OK!\n");
	
	  printf("bsp init OK\n");
	
	//I2C
		I2C_GPIO_Configuration();
	//u3_printf("I2C INIT OK!\n");
	
	//MPU6050

	MPU6050_Init();
	
//	while(1)
//	{		
//	Prepare_Data();
//	Get_Attitude();
//	delay_ms(500);
//	}
	
	
	//u3_printf("MPU6050 INIT OK!\n");
	
//	FLASH_Unlock();//FLASH解锁
//	EE_Init();
//	EE_READ_ACC_OFFSET();
//	EE_READ_GYRO_OFFSET();
//	  EE_READ_PID();//从FLASH读取MPU6050参数
		GYRO_OFFSET_OK=0;
	  ACC_OFFSET_OK=0;
	
		
}
void SysTick_init(void)
{
 SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);//初始化并使能 SysTick 定时器 3.5库宏定义不同
}

