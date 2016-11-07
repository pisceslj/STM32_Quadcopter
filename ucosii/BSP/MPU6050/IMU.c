#include "IMU.h"
#include "MPU6050.h"
#include "math.h"
#include "includes.h"

//#define RtA 		57.324841				//弧度到角度
//#define AtR    	0.0174533				//度到角度
//#define Acc_G 	0.0011963				//加速度变成G
//#define Gyro_G 	0.0152672				//角速度变成度
//#define Gyro_Gr	0.0002663				//角速度变成弧度速度
#define FILTER_NUM 20

S_FLOAT_XYZ ACC_AVG;			//平均值滤波后的ACC
S_FLOAT_XYZ GYRO_I;				//陀螺仪积分
S_FLOAT_XYZ EXP_ANGLE;		//期望角度
S_FLOAT_XYZ DIF_ANGLE;		//期望角度与实际角度差
S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度
S_FLOAT_XYZ OFFSET_ANGLE;			// offset

int16_t	ACC_X_BUF[FILTER_NUM],ACC_Y_BUF[FILTER_NUM],ACC_Z_BUF[FILTER_NUM];	//加速度滑动窗口滤波数组
float zoffset;
extern int16_t Magn_x,Magn_y,Magn_z;
extern u8 debug;
extern float Magn_fx,Magn_fy,Magn_fz;
extern float anglez_offset;
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
	
	Magn_fx=Magn_x/1090;
	Magn_fy=Magn_y/1090;
	Magn_fz=Magn_z/1090;


	GYRO_I.X += Gyro_x*0.002*2.8963;//0.002是时间间隔,两次prepare的执行周期
	GYRO_I.Y += Gyro_y*0.002*2.8963;
	GYRO_I.Z += Gyro_z*0.002*2.8963;
	
//	IMUupdate(Gyro_x*0.0174,
//						Gyro_y*0.0174,
//						Gyro_z*0.0174,
//						x,y,z);	//*0.0174转成弧度
 		
	 AHRSupdate(Gyro_x*0.0174,
						Gyro_y*0.0174,
						Gyro_z*0.0174,
						x,y,z,
						Magn_x,Magn_y,Magn_z);	//*0.0174转成弧度

}
////////////////////////////////////////////////////////////////////////////////
#define Kp 20.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.005f                          // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.001f                   // half the sample period采样周期的一半

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float temp1,temp2,temp3,temp0;
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
  temp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  temp1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  temp2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  temp3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	q0=temp0;
	q1=temp1;
	q2=temp2;
	q3=temp3;


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
	
	Q_ANGLE.Z = atan2f(Magn_y,Magn_x) * 57.3; // angle in degrees
	Q_ANGLE.Z-=anglez_offset;
	if(Q_ANGLE.Z>180) Q_ANGLE.Z=360-Q_ANGLE.Z;
	OSMutexPost (ANGLE);
	
	if(debug)
	{
		if(i==250)
	{
		
	printf("X:%f  Y:%f   Z: %f\n",Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z);
		i=0;
	}
	i++;	
	}
}


// Header files


void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {				
        float norm;
        float hx, hy, hz, bx, bz;
        float vx, vy, vz, wx, wy, wz;
        float ex, ey, ez;
				int i;
        // auxiliary variables to reduce number of repeated operations

        float q0q0 = q0*q0;
        float q0q1 = q0*q1;
        float q0q2 = q0*q2;
        float q0q3 = q0*q3;
        float q1q1 = q1*q1;
        float q1q2 = q1*q2;
        float q1q3 = q1*q3;
        float q2q2 = q2*q2;   
        float q2q3 = q2*q3;
        float q3q3 = q3*q3;          
        
        // normalise the measurements

        norm = sqrt(ax*ax + ay*ay + az*az);       
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;

        norm = sqrt(mx*mx + my*my + mz*mz);          
        mx = mx / norm;
        my = my / norm;
        mz = mz / norm;         

        
        // compute reference direction of flux

        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);

        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);

        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         

        bx = sqrt((hx*hx) + (hy*hy));
				
        bz = hz;        

        
        // estimated direction of gravity and flux (v and w)

        vx = 2*(q1q3 - q0q2);

        vy = 2*(q0q1 + q2q3);

        vz = q0q0 - q1q1 - q2q2 + q3q3;

        wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);

        wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);

        wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  

        
        // error is sum of cross product between reference direction of fields and direction measured by sensors

        ex = (ay*vz - az*vy) + (my*wz - mz*wy);

        ey = (az*vx - ax*vz) + (mz*wx - mx*wz);

        ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

        
        // integral error scaled integral gain

        exInt = exInt + ex*Ki;

        eyInt = eyInt + ey*Ki;

        ezInt = ezInt + ez*Ki;

        
        // adjusted gyroscope measurements

        gx = gx + Kp*ex + exInt;

        gy = gy + Kp*ey + eyInt;

        gz = gz + Kp*ez + ezInt;

        
        // integrate quaternion rate and normalise
				temp0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
				temp1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
				temp2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
				temp3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

				q0=temp0;
				q1=temp1;
				q2=temp2;
				q3=temp3;
				
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
	Q_ANGLE.Z-=zoffset;
	if(Q_ANGLE.Z > 180)
			Q_ANGLE.Z-=360;
	else if(Q_ANGLE.Z<-180)
			Q_ANGLE.Z+=360;
	
	OSMutexPost(ANGLE);                                                 
		
	 	if(debug)
	{
			if(i>=250)
		{
			
		printf("X:%f  Y:%f   Z: %f\n",Q_ANGLE.X,Q_ANGLE.Y,Q_ANGLE.Z);
			i=0;
		}
		i++;	
	}

}



