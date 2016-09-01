# STM32_Quadcopter
This is an embeded project for our major design!!!
This structure is as follows:

本工程的代码结构如下：
 _____________________________________________________________
|
|
|--STARTUP |--startup_stm32f10x_hd.s       |--
|           
|--CMSIS   |--core_cm3.c                   |--
|          |--core_cm3.h                   
|          |--stm32f10x.h                  |--
|          |--system_stm32f10x.c           |--
|          |--system_stm32f10x.h
|
|--FWLB    |--misc.c                       |--
|          |--stm32f10x_adc.c
|          |--stm32f10x_bkp.c
|          |--stm32f10x_can.c
|          |--.....
|
|--USER    |--main.c                        |--初始化及创建任务
|          |--stm32f10x_conf.h              |--函数库引用配置文件
|          |--stm32f10x_it.c                |--主要中断服务程序
|
|--DOC     |--GPIO.txt
|          |--IIC.txt
|          |--README.md
|
|--BSP     |--BSP.c                         |--BSP及SysTick初始化及使能
|          |--BlueTooth.c
|          |--GPS.c
|          |--I2cdev.c
|          |--motor.c
|          |--kalman.c
|          |--MPU6050.c
|          |--Rev.c
|          |--tim7.c  
|          |--usart1.c
|          |--HMC5883L.c
|          |--Pid.c
|
|--APP     |--app.c
|          
|--SOURCES |--os_core.c
|          |--os_flag.c
|          |--os_mbox.c
|          |--os_mem.c
|          |--os_mutex.c
|          |--.......
|
|--PROTS   |--os_cpu_c.c
|          |--os_dbg.c
|          |--os_cpu_a.asm
|
|_________________________________________________________
