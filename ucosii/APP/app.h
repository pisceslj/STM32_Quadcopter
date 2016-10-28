#ifndef _APP_H_
#define _APP_H_

/**************** 用户任务声明 *******************/
void Task_Start(void *p_arg);
void Task_BlueTooth(void *p_arg);
void Task_atttitude_computation(void *p_arg);
void Task_atttitude_pid(void *p_arg);
//void Task_GPS(void *p_arg);

////DEBUG
//void Task_LED2(void *p_arg);
//void Task_LED3(void *p_arg);
#endif //_APP_H_
