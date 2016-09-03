#ifndef __APP_CFG_H__
#define __APP_CFG_H__


/*******************设置任务优先级*******************/
#define STARTUP_TASK_PRIO 4
#define TASK_BLUETOOTH_PRIO 5
#define TASK_ATTITUDE_COMPUTATION_PRIO 6
#define TASK_ATTITUDE_PID_PRIO 7
#define TASK_GPS_PRIO 8

//DEBUG
#define TASK_LED2_PRIO 9
#define TASK_LED3_PRIO 10
/************设置栈大小（单位为 OS_STK ）************/
#define STARTUP_TASK_STK_SIZE 80
#define TASK_STK_SIZE 80
#define TASK_LED2_STK_SIZE 80
#define TASK_LED3_STK_SIZE 80

#endif
