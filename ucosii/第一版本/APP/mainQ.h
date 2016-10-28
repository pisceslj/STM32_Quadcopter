#ifndef __MAINQ_H__
#define __MAINQ_H__
#include "ucos_ii.h"

/******************消息队列*******************************/
//发向主任务




OS_EVENT * MainQ;
void *QMessageMain[QSIZE];
INT8U errMainQ;

//发向姿态控制
OS_EVENT * PIDQ;
void *QMessagePID[QSIZE];
INT8U errPIDQ;

//发向通信线程 ->上位机
OS_EVENT * PCQ;
void *QMessagePC[QSIZE];
INT8U errPCQ;

#endif