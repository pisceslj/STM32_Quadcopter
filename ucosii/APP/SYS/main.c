/*************************************************
Copyright:www.wellmakers.com
Author:路洋
Date:2014-04-25
Version:V1.2
Description: 
   可以连接V1.2版本的PC上位机
	 可以连接V1.2版本的手机蓝牙上位机
	 没有加入四轴的方向控制，只有油门控制（只可以控制四轴高度）
	 主要程序都在stm32f10x_it.c的TIM3_IRQHandler中断里通过时间片轮转执行
	 更多说明和上位机等下载在http://www.wellmakers.com/753/
**************************************************/
/* Includes ------------------------------------------------------------------*/
//#define FLY //直接放在工程的C/C++选项卡里定义宏了，这里这句不需要了
//不定义 FLY电机不用通过蓝牙解锁，油门给1500 方便上位机调试
#include "stm32f10x.h"
#include "BSP/BSP.h"
#include "app/uart/uart1.h"
#include "app/rc/rc.h"
#include "control.h"
#include <stdio.h>
#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)
u8 WaitForStart = 1;
////////////////////////////////////////////////////////////////////////////////
/*************************************************
Copyright:www.wellmakers.com
Date:2014-04-25
Description:系统初始化
**************************************************/
void SYS_INIT(void)
{
	LED_INIT();
	Moto_Init();
	Pid_init();
	Tim3_Init(500);				//1000=1MS,500=0.5MS
	Uart1_Init(115200);	
	Nvic_Init();					//中断初始化
	ANOTech_taobao_com_I2C1_INIT(0xA6,400000,2,1,1,1,1);//用了匿名的硬件I2C库 
	
#ifdef FLY
	//等待蓝牙解锁标志位
	while(WaitForStart){
		LED1_ONOFF();//解锁成功，闪烁LED
	}
#else
	//测试模式，直接解锁
		ARMED=1;//电机解锁
		Rc_Get.THROTTLE = 1500;//油门给到中值
#endif
	MPU6050_Init();//初始化MPU6050
	FLASH_Unlock();//FLASH解锁
	EE_INIT();
//	EE_READ_ACC_OFFSET();
//	EE_READ_GYRO_OFFSET();
	EE_READ_PID();//从FLASH读取MPU6050参数
	
  LED_FLASH();
	//设置自检标志位为0，自检好之后置1
	GYRO_OFFSET_OK=0;
	ACC_OFFSET_OK=0;
	
}
/*************************************************
Copyright:www.wellmakers.com
Date:2014-04-25
Description:主函数
**************************************************/
int main(void)
{		 
	SYS_INIT_OK=0;//等待初始化
	SYS_INIT();	
	SYS_INIT_OK=1;//初始化完成
  while (1)
  {		
#ifdef FLY		
		BlueToothCheckEvent();
#endif
	}
}
////////////////////////////////////////////////////////////////////////////////

