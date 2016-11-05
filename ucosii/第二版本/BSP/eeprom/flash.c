//#include "includes.h"

//#define EE_6050_ACC_X_OFFSET_ADDR	0
//#define EE_6050_ACC_Y_OFFSET_ADDR	1
//#define EE_6050_ACC_Z_OFFSET_ADDR	2
//#define EE_6050_GYRO_X_OFFSET_ADDR	3
//#define EE_6050_GYRO_Y_OFFSET_ADDR	4
//#define EE_6050_GYRO_Z_OFFSET_ADDR	5
//#define EE_PID_ROL_P	6
//#define EE_PID_ROL_I	7
//#define EE_PID_ROL_D	8
//#define EE_PID_PIT_P	9
//#define EE_PID_PIT_I	10
//#define EE_PID_PIT_D	11
//#define EE_PID_YAW_P	12
//#define EE_PID_YAW_I	13
//#define EE_PID_YAW_D	14
////ErrorStatus  HSEStartUpStatus;
////FLASH_Status FlashStatus;
//extern S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//零漂
//uint16_t VirtAddVarTab[NumbOfVar] = {0xAA00, 0xAA01, 0xAA02, 0xAA03, 0xAA04, 0xAA05, 0xAA06, 0xAA07, 0xAA08, 0xAA09, 
//																		 0xAA0A,0xAA0B, 0xAA0C, 0xAA0D, 0xAA0E,};
//uint16_t temp;
//void EE_INIT(void)
//{
//	EE_Init();
//}
//void EE_SAVE_ACC_OFFSET(void)
//{
//	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], ACC_OFFSET.X);
//	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], ACC_OFFSET.Y);
//	EE_WriteVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], ACC_OFFSET.Z);
//}
//void EE_READ_ACC_OFFSET(void)
//{
//	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_X_OFFSET_ADDR], &ACC_OFFSET.X);
//	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_Y_OFFSET_ADDR], &ACC_OFFSET.Y);
//	EE_ReadVariable(VirtAddVarTab[EE_6050_ACC_Z_OFFSET_ADDR], &ACC_OFFSET.Z);
//}
//void EE_SAVE_GYRO_OFFSET(void)
//{
//	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], GYRO_OFFSET.X);
//	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], GYRO_OFFSET.Y);
//	EE_WriteVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], GYRO_OFFSET.Z);
//}
//void EE_READ_GYRO_OFFSET(void)
//{
//	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_X_OFFSET_ADDR], &GYRO_OFFSET.X);
//	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_Y_OFFSET_ADDR], &GYRO_OFFSET.Y);
//	EE_ReadVariable(VirtAddVarTab[EE_6050_GYRO_Z_OFFSET_ADDR], &GYRO_OFFSET.Z);
//}
//void EE_SAVE_PID(void)
//{
//	u16 _temp;
//	_temp = PID_ROL.P * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_P],_temp);
//	_temp = PID_ROL.I * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_I],_temp);
//	_temp = PID_ROL.D * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_ROL_D],_temp);
//	_temp = PID_PIT.P * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_P],_temp);
//	_temp = PID_PIT.I * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_I],_temp);
//	_temp = PID_PIT.D * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_PIT_D],_temp);
//	_temp = PID_YAW.P * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_P],_temp);
//	_temp = PID_YAW.I * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_I],_temp);
//	_temp = PID_YAW.D * 100;
//	EE_WriteVariable(VirtAddVarTab[EE_PID_YAW_D],_temp);
//}
//void EE_READ_PID(void)
//{
//	u16 _temp;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_P],&_temp);
//	PID_ROL.P = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_I],&_temp);
//	PID_ROL.I = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_D],&_temp);
//	PID_ROL.D = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_P],&_temp);
//	PID_PIT.P = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_I],&_temp);
//	PID_PIT.I = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_D],&_temp);
//	PID_PIT.D = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_P],&_temp);
//	PID_YAW.P = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_I],&_temp);
//	PID_YAW.I = (float)_temp / 100;
//	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_D],&_temp);
//	PID_YAW.D = (float)_temp / 100;
//}
//void EE_READ_PID_Send(void)		//从flash中获取 并且发送
//{
//	u16 _temp;
//	
//	Uart1_Put_Char(0x70);//帧头
//	Uart1_Put_Char(0x8A);//帧头
//	Uart1_Put_Char(0x1C);//帧头
//	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_P],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_I],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_ROL_D],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_P],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_I],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_PIT_D],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_P],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_I],&_temp);
//	Uart1_Put_Char(BYTE1(_temp));
//	Uart1_Put_Char(BYTE0(_temp));
//	EE_ReadVariable(VirtAddVarTab[EE_PID_YAW_D],&_temp);
//	for(_temp=0;_temp<22;_temp++)
//	{
//		Uart1_Put_Char((0));
//	}
//	
//}