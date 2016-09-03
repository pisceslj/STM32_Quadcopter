#include "includes.h"

void BSP_Init()
{
	/* 配置系统时钟为72M  3.5库中启动文件已调用*/
	
	SysTick_init(); /* 初始化并使能 SysTick 定时器 */
	//LED_GPIO_Config(); /* LED 端口初始化 指示作用*/
}

void SysTick_init(void)
{
	SysTick_Config(SystemCoreClock/OS_TICKS_PER_SEC);//初始化并使能 SysTick 定时器 3.5库宏定义不同
}