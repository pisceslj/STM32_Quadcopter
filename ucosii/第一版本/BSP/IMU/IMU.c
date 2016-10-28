#include "MPU6050.h"
#include "math.h"
#include "includes.h"
#include "IMU.h"

#define FILTER_NUM 20

S_FLOAT_XYZ ACC_AVG;			//平均值滤波后的ACC
S_FLOAT_XYZ GYRO_I;				//陀螺仪积分
S_FLOAT_XYZ EXP_ANGLE;		//期望角度
S_FLOAT_XYZ DIF_ANGLE;		//期望角度与实际角度差
S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度

int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	//加速度滑动窗口滤波数组

extern int debug;

extern OS_EVENT * ANGLE;  //信号量
extern INT8U errANGLE;

void Prepare_Data(void)
{
	static uint8_t filter_cnt=0;
	int32_t temp1=0,temp2=0,temp3=0;
	uint8_t i;
	
	MPU6050_Read();
	MPU6050_Dataanl();
	
	ACC_X_BUF[filter_cnt] = MPU6050_ACC_LAST.X;//更新滑动窗口数组
	ACC_Y_BUF[filter_cnt] = MPU6050_ACC_LAST.Y;
	ACC_Z_BUF[filter_cnt] = MPU6050_ACC_LAST.Z;
	for(i=0;i<FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	ACC_AVG.X = temp1 / FILTER_NUM;
	ACC_AVG.Y = temp2 / FILTER_NUM;
	ACC_AVG.Z = temp3 / FILTER_NUM;
	filter_cnt++;
	if(filter_cnt==FILTER_NUM)	filter_cnt=0;
}

void Get_Attitude(void)
{
	float x,y,z,Gyro_x,Gyro_y,Gyro_z;
  static int q=0,i=0;
	x=ACC_AVG.X/8192;
	y=ACC_AVG.Y/8192;
	z=ACC_AVG.Z/8192;
	
	Gyro_x=MPU6050_GYRO_LAST.X/32.8;
	Gyro_y=MPU6050_GYRO_LAST.Y/32.8;
	Gyro_z=MPU6050_GYRO_LAST.Z/32.8;
	

//if(debug)
//	{
//		if(i==500)
//	{

//			printf("%d\n",MPU6050_GYRO_LAST.X);
//			printf("%d\n",MPU6050_GYRO_LAST.Y);
//			printf("%d\n",MPU6050_GYRO_LAST.Z);
//		  printf("G\n");
//			printf("%f\n",Gyro_x);
//			printf("%f\n",Gyro_y);
//			printf("%f\n",Gyro_z);
//		  printf("\n");

//		i=0;
//	}
//	i++;
//		
//	}
if(debug)
{
	GYRO_I.X += Gyro_x*0.002*2.8963;//0.002是时间间隔,两次prepare的执行周期
	GYRO_I.Y += Gyro_y*0.002*2.8963;
	GYRO_I.Z += Gyro_z*0.002*2.8963;
	
	
//if(debug)
//	{
//		if(i==500)
//	{
//		  printf("G\n");
//			printf("%f\n",GYRO_I.X);
//			printf("%f\n",GYRO_I.Y);
//			printf("%f\n",GYRO_I.Z);
//		  printf("\n");

//		i=0;
//	}
//	i++;
//		
//	}
	
	IMUupdate(Gyro_x*0.0174,
						Gyro_y*0.0174,
						Gyro_z*0.0174,
						x,y,z);	//*0.0174转成弧度
}
}
////////////////////////////////////////////////////////////////////////////////
#define Kp 10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
	static int i=0;
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
	//float q0q3 = q0*q3;
  float q1q1 = q1*q1;
	//float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
		
  norm = sqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex * Ki;								  //对误差进行积分
  eyInt = eyInt + ey * Ki;
  ezInt = ezInt + ez * Ki;

  // adjusted gyroscope measurements
  gx = gx + Kp*ex + exInt;					   							//将误差PI后补偿到陀螺仪，即补偿零点漂移
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;				   							//这里的gz由于没有观测者进行矫正会产生漂移，表现出来的就是积分自增或自减

  // integrate quaternion rate and normalise						   //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;


	//加个信号量
	OSMutexPend (ANGLE,0, &errANGLE);
  Q_ANGLE.Y  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  Q_ANGLE.X = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
  Q_ANGLE.Z = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3; // yaw
	
	Q_ANGLE.Z *=3.1236;
	
	//Q_ANGLE.Z = GYRO_I.Z;
	
	
	if(debug)
	{
		if(i==500)
	{

	
		i=0;
	}
	i++;	
	}
	
	OSMutexPost (ANGLE);
}
