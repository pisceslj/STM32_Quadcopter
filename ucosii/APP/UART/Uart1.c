#include "uart1.h"
#include "app/rc/rc.h"
#include "bsp/MPU6050.h"
#include "bsp/led.h"
#include "app/imu/imu.h"
#include "control.h"
#include "bsp/bsp.h"
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
u8 ARMED = 0;
char flag_send;

RC_GETDATA Rc_Get;//接收到的RC数据,1000~2000
float RC_Target_ROL=0,RC_Target_PIT=0,RC_Target_YAW=0;
//void RC_FUN(void)
//{
////判断解锁
//	if(ARMED==0 && Rc_Get.THROTTLE < 1100 && Rc_Get.YAW <1100)
//  {ARMED = 1;LED2_ON;}		//解锁
//	if(ARMED==1 && Rc_Get.THROTTLE <1100 && Rc_Get.YAW >1900)
//	{ARMED = 0;LED2_OFF;}		//锁定
//}
void BlueToothDeal(u8 buf_num)
{
	if(Rx_Buf[buf_num][1]==0x80)		//蓝牙上位机指令
	{		
		ARMED=1;
		if(Rx_Buf[buf_num][2]==0x81)flag_send=1;
		//16位数据  低位在前
				Rc_Get.THROTTLE = Rx_Buf[buf_num][2]  ;Rc_Get.THROTTLE <<= 8;Rc_Get.THROTTLE += Rx_Buf[buf_num][3];
				Rc_Get.YAW      = Rx_Buf[buf_num][4]  ;Rc_Get.YAW      <<= 8;Rc_Get.YAW      += Rx_Buf[buf_num][5];
				Rc_Get.ROLL     = Rx_Buf[buf_num][6]  ;Rc_Get.ROLL     <<= 8;Rc_Get.ROLL     += Rx_Buf[buf_num][7];
				Rc_Get.PITCH    = Rx_Buf[buf_num][8]  ;Rc_Get.PITCH    <<= 8;Rc_Get.PITCH    += Rx_Buf[buf_num][9];
				Rc_Get.AUX1     = Rx_Buf[buf_num][10] ;Rc_Get.AUX1     <<= 8;Rc_Get.AUX1     += Rx_Buf[buf_num][11];
				Rc_Get.AUX2     = Rx_Buf[buf_num][12] ;Rc_Get.AUX2     <<= 8;Rc_Get.AUX2     += Rx_Buf[buf_num][13];
				Rc_Get.AUX3     = Rx_Buf[buf_num][14] ;Rc_Get.AUX3     <<= 8;Rc_Get.AUX3     += Rx_Buf[buf_num][15];
				Rc_Get.AUX4     = Rx_Buf[buf_num][16] ;Rc_Get.AUX4     <<= 8;Rc_Get.AUX4     += Rx_Buf[buf_num][17];
				Rc_Get.AUX5     = Rx_Buf[buf_num][18] ;Rc_Get.AUX5     <<= 8;Rc_Get.AUX5     += Rx_Buf[buf_num][19];
		
			//RC_FUN();
		
				RC_Target_ROL = (Rc_Get.ROLL-1500)/30;
				RC_Target_PIT = (Rc_Get.PITCH-1500)/30;
				RC_Target_YAW = (Rc_Get.YAW-1500)/30;
				LED2_ONOFF();
	}
	else if(Rx_Buf[buf_num][1]==0x77)	//加锁解锁
	{
		Rc_Get.THROTTLE = Rx_Buf[buf_num][2]  ;Rc_Get.THROTTLE <<= 8;Rc_Get.THROTTLE += Rx_Buf[buf_num][3];
		Rc_Get.YAW      = Rx_Buf[buf_num][4]  ;Rc_Get.YAW      <<= 8;Rc_Get.YAW      += Rx_Buf[buf_num][5];
			//判断解锁
		if(ARMED==0 && Rc_Get.THROTTLE < 1100 && Rc_Get.YAW <1100)
		{ARMED = 1;LED2_ON;}		//解锁
		if(ARMED==1 && Rc_Get.THROTTLE <1100 && Rc_Get.YAW >1900)
		{ARMED = 0;LED2_OFF;}		//锁定
	}
	else if(Rx_Buf[buf_num][1]==0x70)	//保存PID数据
	{
			PID_ROL.P = (float)((vs16)(Rx_Buf[buf_num][2]<<8) |Rx_Buf[buf_num][3])/100;
			PID_ROL.I = (float)((vs16)(Rx_Buf[buf_num][4]<<8) |Rx_Buf[buf_num][5])/100;
			PID_ROL.D = (float)((vs16)(Rx_Buf[buf_num][6]<<8) |Rx_Buf[buf_num][7]) /10000;
			PID_PIT.P = (float)((vs16)(Rx_Buf[buf_num][8]<<8) |Rx_Buf[buf_num][9])/100;
			PID_PIT.I = (float)((vs16)(Rx_Buf[buf_num][10]<<8)|Rx_Buf[buf_num][11])/100;
			PID_PIT.D = (float)((vs16)(Rx_Buf[buf_num][12]<<8)|Rx_Buf[buf_num][13])/10000;
			PID_YAW.P = (float)((vs16)(Rx_Buf[buf_num][14]<<8)|Rx_Buf[buf_num][15])/100;
			PID_YAW.I = (float)((vs16)(Rx_Buf[buf_num][16]<<8)|Rx_Buf[buf_num][17])/100;
			PID_YAW.D = (float)((vs16)(Rx_Buf[buf_num][18]<<8)|Rx_Buf[buf_num][19])/10000;
			EE_SAVE_PID();	
			Uart1_Put_Char(0x71);//帧头	
			Uart1_Put_Char(0x88);//帧头
		
	}
	else if(Rx_Buf[buf_num][1]==0x71)		//发送PID数据给上位机
	{
		//反馈数据
		EE_READ_PID_Send();
	}
}
void Uart_DataAnl(u8 buf_num)		//串口缓存数据分析
{
 if(Rx_Buf[buf_num][1]==0x8A)		//串口收到的是上位机的遥控数据
	{
		Uart1_Put_Char(0x30+buf_num);
	}
}
void BlueToothCheckEvent(void)
{
	//0-9位 校验和 
	if(Rx_Ok0)
	{
		Rx_Ok0 = 0;
//		u8 sum = 0;
//		for(int i=0;i<20;i++)
//			sum += Rx_Buf[0][i];
//		if(sum == Rx_Buf[0][20])		//和校验通过
//		{
			BlueToothDeal(0);
//		}
		
	}
	if(Rx_Ok1)
	{
		Rx_Ok1 = 0;
//		u8 sum = 0;
//		for(int i=0;i<20;i++)
//			sum += Rx_Buf[1][i];
//		if(sum == Rx_Buf[1][20])		//和校验通过
//		{
			BlueToothDeal(1);
//		}
	}	
}
void Uart_CheckEvent(void)
{
	if(Rx_Ok0)
	{
		Rx_Ok0 = 0;
		u8 sum = 0;
		for(int i=0;i<31;i++)
			sum += Rx_Buf[0][i];
		if(sum == Rx_Buf[0][31])		//和校验通过
		{
			Uart_DataAnl(0);
		}
	}
	if(Rx_Ok1)
	{
		Rx_Ok1 = 0;
		u8 sum = 0;
		for(int i=0;i<31;i++)
			sum += Rx_Buf[1][i];
		if(sum == Rx_Buf[1][31])		//和校验通过
		{
			Uart_DataAnl(1);
		}
	}
}

void Uart1_Send_Buf(u8 *buf,u8 len)		//发送buf,长度len,返回字节和sum
{
	while(len)
	{
		Uart1_Put_Char(*buf);
		buf++;
		len--;
	}
}
void Uart1_Send_RCdata(void)
{
	uint8_t sum = 0;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAE);
	sum += Uart1_Put_Char(28);
	sum += Uart1_Put_Int16(Rc_Get.THROTTLE);
	sum += Uart1_Put_Int16(Rc_Get.YAW);
	sum += Uart1_Put_Int16(Rc_Get.ROLL);
	sum += Uart1_Put_Int16(Rc_Get.PITCH);
	sum += Uart1_Put_Int16(Rc_Get.AUX1);
	sum += Uart1_Put_Int16(Rc_Get.AUX2);
	sum += Uart1_Put_Int16(Rc_Get.AUX3);
	sum += Uart1_Put_Int16(Rc_Get.AUX4);
	sum += Uart1_Put_Int16(0);
	sum += Uart1_Put_Int16(0);
	sum += Uart1_Put_Int16(0);
	sum += Uart1_Put_Int16(0);
	sum += Uart1_Put_Int16(0);
	sum += Uart1_Put_Int16(0);
	Uart1_Put_Char(sum);
}

void PC_Debug_Show(u8 num,u16 sta)//sta=0 熄灭 sta=1 点亮  >1 取反
{
	static uint8_t led_s[6] = {0,0,0,0,0,0};
	uint8_t sum = 0;
	if(0<num && num<7)
	{
		sum += Uart1_Put_Char(0x88);
		sum += Uart1_Put_Char(0xAD);
		sum += Uart1_Put_Char(0x02);
		sum += Uart1_Put_Char(num);
		if(sta==0)
			sum += Uart1_Put_Char(0x00);
		else if(sta==1)
			sum += Uart1_Put_Char(0x01);
		else 
		{
			if(led_s[num])	led_s[num] = 0;
			else 			led_s[num] = 1;
			sum += Uart1_Put_Char(led_s[num]);
		}
		Uart1_Put_Char(sum);
	}
	else if(6<num && num<13)
	{
		sum += Uart1_Put_Char(0x88);
		sum += Uart1_Put_Char(0xAD);
		sum += Uart1_Put_Char(0x03);
		sum += Uart1_Put_Char(num);
		sum += Uart1_Put_Int16(sta);
		Uart1_Put_Char(sum);
	}
}
void Uart1_Send_AF(void)
{
	uint8_t sum = 0;
	uint16_t _temp;
	sum += Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAF);
	sum += Uart1_Put_Char(0x1c);
	sum += Uart1_Put_Char(BYTE1(MPU6050_ACC_LAST.X));
	sum += Uart1_Put_Char(BYTE0(MPU6050_ACC_LAST.X));
	sum += Uart1_Put_Char(BYTE1(MPU6050_ACC_LAST.Y));
	sum += Uart1_Put_Char(BYTE0(MPU6050_ACC_LAST.Y));
	sum += Uart1_Put_Char(BYTE1(MPU6050_ACC_LAST.Z));
	sum += Uart1_Put_Char(BYTE0(MPU6050_ACC_LAST.Z));
	sum += Uart1_Put_Char(BYTE1(MPU6050_GYRO_LAST.X));
	sum += Uart1_Put_Char(BYTE0(MPU6050_GYRO_LAST.X));
	sum += Uart1_Put_Char(BYTE1(MPU6050_GYRO_LAST.Y));
	sum += Uart1_Put_Char(BYTE0(MPU6050_GYRO_LAST.Y));
	sum += Uart1_Put_Char(BYTE1(MPU6050_GYRO_LAST.Z));
	sum += Uart1_Put_Char(BYTE0(MPU6050_GYRO_LAST.Z));
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	
	_temp = (int)(Q_ANGLE.X*100);
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	_temp = (int)(Q_ANGLE.Y*100);
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
  _temp = (int)(Q_ANGLE.Z*100);
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	
	//sum += Uart1_Put_Char(0);
  //sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	Uart1_Put_Char(sum);
}

void Uart1_Send_AE(void)
{
	uint8_t sum=0;
	uint16_t _temp;
	sum = 0;	
	sum =  Uart1_Put_Char(0x88);
	sum += Uart1_Put_Char(0xAE);
	sum += Uart1_Put_Char(0x1c);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	sum += Uart1_Put_Char(0);
	
	_temp = TIM2->CCR1/10;
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	_temp = TIM2->CCR2/10;
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	_temp = TIM2->CCR3/10;
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	_temp = TIM2->CCR4/10;
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	
	_temp = 2500/6;
	sum += Uart1_Put_Char(BYTE1(_temp));
	sum += Uart1_Put_Char(BYTE0(_temp));
	
	
	Uart1_Put_Char(sum);
	
}
