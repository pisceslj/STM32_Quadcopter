#ifndef __MAINQ_H__
#define __MAINQ_H__
#include "ucos_ii.h"

/******************消息队列*******************************/


//角度信息
OS_EVENT * MainQ;
void *QMessageMain[QSIZE];
INT8U errMainQ;

//发向姿态控制
OS_EVENT * PIDQ;
void *QMessagePID[QSIZE];
INT8U errPIDQ;


#endif