#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#include "stm32f10x.h"
#include "stm32f10x_conf.h"  
#include "stm32f10x_rcc.h"	//SysTick定时器相关

#include "ucos_ii.h" 				//uC/OS-II 系统函数头文件

#include "BSP.h" 						//与板级初始化相关的函数
#include "app.h" 						//用户任务函数

#include <stdio.h>
#include <stdarg.h>

#endif //__INCLUDES_H__
