#ifndef _HMC5883L_H_
#define _HMC5883L_H_
#include "stm32f10x.h"
#include "includes.h"
#include  <math.h> 

void Identify_HMC5883L(void);
void Read_HMC5883L(void);//∂¡»°
void HMC5883L_Self_Test(void);//∂¡»°
void Init_HMC5883L(void);
#endif