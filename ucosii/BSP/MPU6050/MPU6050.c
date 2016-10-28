#include "MPU6050.h"
#include "tim7.h"
#include "includes.h"

u8						mpu6050_buffer[14];					//iic读取后存放数据
S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//零漂
u8						GYRO_OFFSET_OK = 1;
u8						ACC_OFFSET_OK = 1;
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST,GYRO_RADIAN_OLD;		//最新一次读取值与上一次读数

int debug;


/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器
*******************************************************************************/
void MPU6050_Dataanl(void)
{


	MPU6050_ACC_LAST.X=((((int16_t)mpu6050_buffer[1]) << 8) | mpu6050_buffer[0])- ACC_OFFSET.X;
	MPU6050_ACC_LAST.Y=((((int16_t)mpu6050_buffer[3]) << 8) | mpu6050_buffer[2])- ACC_OFFSET.Y;
	MPU6050_ACC_LAST.Z=((((int16_t)mpu6050_buffer[5]) << 8) | mpu6050_buffer[4])- ACC_OFFSET.Z;
	//跳过温度ADC
	MPU6050_GYRO_LAST.X=((((int16_t)mpu6050_buffer[9]) << 8) | mpu6050_buffer[8])-GYRO_OFFSET.X;
	MPU6050_GYRO_LAST.Y=((((int16_t)mpu6050_buffer[11]) << 8) | mpu6050_buffer[10])-GYRO_OFFSET.Y;
	MPU6050_GYRO_LAST.Z=((((int16_t)mpu6050_buffer[13]) << 8) | mpu6050_buffer[12])-GYRO_OFFSET.Z;

	

	if(!GYRO_OFFSET_OK)
	{
		static int32_t	tempgx=0,tempgy=0,tempgz=0;
		static uint16_t cnt_g=0;
// 		LED1_ON;
		if(cnt_g==0)
		{
			GYRO_OFFSET.X=0;
			GYRO_OFFSET.Y=0;
			GYRO_OFFSET.Z=0;
			tempgx = 0;
			tempgy = 0;
			tempgz = 0;
			cnt_g = 1;
			return;
		}
		tempgx+= MPU6050_GYRO_LAST.X;
		tempgy+= MPU6050_GYRO_LAST.Y;
		tempgz+= MPU6050_GYRO_LAST.Z;
		if(cnt_g==1000)
		{
			GYRO_OFFSET.X=tempgx/cnt_g;
			GYRO_OFFSET.Y=tempgy/cnt_g;
			GYRO_OFFSET.Z=tempgz/cnt_g;
			cnt_g = 0;
			GYRO_OFFSET_OK = 1;
//			EE_SAVE_GYRO_OFFSET();//保存数据
			return;
		}
		cnt_g++;
	}
	if(!ACC_OFFSET_OK)
	{
		static int32_t	tempax=0,tempay=0,tempaz=0;
		static uint16_t cnt_a=0;
// 		LED1_ON;
		if(cnt_a==0)
		{
			ACC_OFFSET.X = 0;
			ACC_OFFSET.Y = 0;
			ACC_OFFSET.Z = 0;
			tempax = 0;
			tempay = 0;
			tempaz = 0;
			cnt_a = 1;
			return;
		}
		tempax+= MPU6050_ACC_LAST.X;
		tempay+= MPU6050_ACC_LAST.Y;
		tempaz+= MPU6050_ACC_LAST.Z;
		if(cnt_a==1000)
		{
			ACC_OFFSET.X=tempax/cnt_a;
			ACC_OFFSET.Y=tempay/cnt_a;
			ACC_OFFSET.Z=tempaz/cnt_a-8192;
			cnt_a = 0;
			ACC_OFFSET_OK = 1;
			printf("OK \n");
			printf("OK \n");
			debug=1;
//			EE_SAVE_ACC_OFFSET();//保存数据
			return;
		}
		cnt_a++;		
	}
}


/**************************实现函数********************************************
//将iic读取到得数据分拆,放入相应寄存器,更新MPU6050_Last
*******************************************************************************/
void MPU6050_Read(void)
{
	
	 mpu6050_buffer[0]=I2C_ReadByte(MPU6050_Addr,ACCEL_XOUT_L);
	 mpu6050_buffer[1]=I2C_ReadByte(MPU6050_Addr,ACCEL_XOUT_H);
       //读取计算X轴数据
	 mpu6050_buffer[2]=I2C_ReadByte(MPU6050_Addr,ACCEL_YOUT_L);
	 mpu6050_buffer[3]=I2C_ReadByte(MPU6050_Addr,ACCEL_YOUT_H);
			 //读取计算Y轴数据
	 mpu6050_buffer[4]=I2C_ReadByte(MPU6050_Addr,ACCEL_ZOUT_L);
	 mpu6050_buffer[5]=I2C_ReadByte(MPU6050_Addr,ACCEL_ZOUT_H);
			  //读取计算Z轴数据
   mpu6050_buffer[6]=I2C_ReadByte(MPU6050_Addr,TEMP_OUT_L); 
	 mpu6050_buffer[7]=I2C_ReadByte(MPU6050_Addr,TEMP_OUT_H); 
	
	 mpu6050_buffer[8]=I2C_ReadByte(MPU6050_Addr,GYRO_XOUT_L); 
   mpu6050_buffer[9]=I2C_ReadByte(MPU6050_Addr,GYRO_XOUT_H);
	   //读取计算X轴数据
   mpu6050_buffer[10]=I2C_ReadByte(MPU6050_Addr,GYRO_YOUT_L);
   mpu6050_buffer[11]=I2C_ReadByte(MPU6050_Addr,GYRO_YOUT_H);
   //读取计算Y轴数据
   mpu6050_buffer[12]=I2C_ReadByte(MPU6050_Addr,GYRO_ZOUT_L);
   mpu6050_buffer[13]=I2C_ReadByte(MPU6050_Addr,GYRO_ZOUT_H);
   //读取计算Z轴数据
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_Init(void)
{
	I2C_WriteByte(MPU6050_Addr,PWR_MGMT_1, 0x00);	//解除休眠状态
	
	I2C_WriteByte(MPU6050_Addr,SMPLRT_DIV, 0x07);    //陀螺仪采样率125HZ
	I2C_WriteByte(MPU6050_Addr,CONFIG, 0x03);        //5Hz 
	
	I2C_WriteByte(MPU6050_Addr,GYRO_CONFIG, 0x10);   //陀螺仪1000度/S 65.5LSB/g
	I2C_WriteByte(MPU6050_Addr,ACCEL_CONFIG, 0x08);  //加速度+-4g  8192LSB/g


	delay_ms(2000);
	MPU6050_WHO_AM_I();
}

void MPU6050_WHO_AM_I(void)
{
	u8 dev=0;
		if(dev=I2C_ReadByte(MPU6050_Addr,WHO_AM_I),dev==0x68)
  { 
    	printf("\r设备MP6050识别成功，id=0x%x\r\n\r",dev);
  }
	else{ printf("\r错误!无法设别设备\r\n\r");}
}