#ifndef _CONTROL_H_
#define _CONTROL_H_
#include "stm32f10x.h"
typedef struct PID{float shell_P,shell_I,shell_D,core_P,core_I,core_D,shell_out,core_out,IMAX;}PID;

extern u8 ARMED;
extern PID PID_ROL,PID_PIT,PID_YAW;

void CONTROL(float rol_now, float pit_now, float yaw_now);
void  Pid_init(void);
#endif
