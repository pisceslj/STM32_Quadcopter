#include "kalman.h"
#include "MPU6050.h"
#include "math.h"

float Accel_x;	     //X轴加速度值暂存
float Accel_y;	     //Y轴加速度值暂存
float Accel_z;	     //Z轴加速度值暂存

float Gyro_x;		 			//X轴陀螺仪数据暂存
float Gyro_y;        //Y轴陀螺仪数据暂存
float Gyro_z;		 		//Z轴陀螺仪数据暂存

float magn_x;		 		//X轴磁场数据暂存
float magn_y;       //Y轴磁场数据暂存
float magn_z;       //Z轴磁场数据暂存
float angle;				//电子罗盘方位角

//float Angle_gy;    //由角速度计算的倾斜角度
float Angle_x_temp;  //由加速度计算的x倾斜角度
float Angle_y_temp;  //由加速度计算的y倾斜角度
float Angle_z_temp;  //由磁场计算的Z倾斜角度

float Angle_X_Final; //X最终倾斜角度
float Angle_Y_Final; //Y最终倾斜角度
float Angle_Z_Final; //Z最终倾斜角度

extern uint16_t GYRO_XOUT,GYRO_YOUT,GYRO_ZOUT,ACCEL_XOUT,ACCEL_YOUT,ACCEL_ZOUT;
extern int Magn_x,Magn_y,Magn_z;

//角度计算
void Angle_Calcu(void)	 
{
	//范围为2g时，换算关系：16384 LSB/g
	//deg = rad*180/3.14
	float x,y,z,hx,hy;
	
	Accel_x = ACCEL_XOUT; //x轴加速度值暂存
	Accel_y = ACCEL_YOUT; //y轴加速度值暂存
	Accel_z = ACCEL_ZOUT; //z轴加速度值暂存
	Gyro_x  = GYRO_XOUT;  //x轴陀螺仪值暂存
	Gyro_y  = GYRO_YOUT;  //y轴陀螺仪值暂存
	Gyro_z  = GYRO_ZOUT;  //z轴陀螺仪值暂存
	magn_x=Magn_x;
  magn_y=Magn_y;    
  magn_z=Magn_z;	


	//处理x轴加速度
	if(Accel_x<32764) x=Accel_x/16384;
	else              x=(32768-Accel_x)/16384;
	
	//处理y轴加速度
	if(Accel_y<32764) y=Accel_y/16384;
	else              y=(32768-Accel_y)/16384;//1-(Accel_y-49152)/16384;
	
	//处理z轴加速度
	if(Accel_z<32764) z=Accel_z/16384;
	else              z=(32768-Accel_z)/16384;
	
	
	//用加速度计算三个轴和水平面坐标系之间的夹角
	Angle_x_temp=(atan(y/z))*180/3.14f;//pitch
	Angle_y_temp=(atan(x/z))*180/3.14f;//roll
	
	//磁场处理
	if(magn_x<32764) x=magn_x/16384;
	else              x=(32768-magn_x)/16384;
		
	if(magn_y<32764) y=magn_y/16384;
	else              y=(32768-magn_y)/16384;//1-(Accel_y-49152)/16384;
	
	if(magn_z<32764) z=magn_z/16384;
	else              z=(32768-magn_z)/16384;
	
		//角度的正负号											
	if(Accel_x<32764) Angle_y_temp = +Angle_y_temp;
	if(Accel_x>32764) Angle_y_temp = -Angle_y_temp;
	if(Accel_y<32764) Angle_x_temp = +Angle_x_temp;
	if(Accel_y>32764) Angle_x_temp = -Angle_x_temp;
//	if(Accel_z<32764) {}
//	if(Accel_z>32764) {}
		
	//补偿公式 a pitch b roll
	//Xr=Xcosα+Ysinαsinβ-Zcosβsinα
	//Yr=Ycosβ+Zsinβ
	hx=x*cos(Angle_x_temp)+y*sin(Angle_x_temp)*sin(Angle_y_temp)-z*cos(Angle_y_temp)*sin(Angle_x_temp);
	hy=y*cos(Angle_y_temp)+z*sin(Angle_y_temp);
	
		//利用磁场计算 yaw最后一个角
	Angle_z_temp= atan2(hy,hx) * (180 / 3.14159265);//+180
	Angle_z_temp-=angle;
	
	

	
	//角速度
	//向前运动
	if(Gyro_x<32768) Gyro_x=-(Gyro_x/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_x>32768) Gyro_x=+(65535-Gyro_x)/16.4;
	//向前运动
	if(Gyro_y<32768) Gyro_y=-(Gyro_y/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_y>32768) Gyro_y=+(65535-Gyro_y)/16.4;
	//向前运动
	if(Gyro_z<32768) Gyro_z=-(Gyro_z/16.4);//范围为1000deg/s时，换算关系：16.4 LSB/(deg/s)
	//向后运动
	if(Gyro_z>32768) Gyro_z=+(65535-Gyro_z)/16.4;
	
	//Angle_gy = Angle_gy + Gyro_y*0.025;  //角速度积分得到倾斜角度.越大积分出来的角度越大
	Kalman_Filter_X(Angle_x_temp,Gyro_x);  //卡尔曼滤波计算Y倾角
	Kalman_Filter_Y(Angle_y_temp,Gyro_y);  //卡尔曼滤波计算Y倾角
	Kalman_Filter_Y(Angle_z_temp,Gyro_z);  //卡尔曼滤波计算Z倾角													  
} 




//卡尔曼参数		
float Q_angle = 0.001;  //预测因素（两个）  噪声 协方差  
float Q_gyro  = 0.003;	//预测因素（两个）  噪声 协方差

/*****************
[  cov(X,X)  cov(Y,X) ]
	 cov(X,Y)  cov(Y,Y)
主对角线  就是上面的两个值
不相关  斜对角线为 0 
*******************/

float R_angle = 0.5;		//观测值 噪声 的方差

/*****************
也是协方差 不过因素就一个（加速计）
所以退化成一个数
*******************/

float dt      = 0.01;//dt为kalman滤波器采样时间;

char  C_0     = 1;// 相当于 H 二维向量{1,0}

float Q_bias=0, Angle_err;
/*****************
陀螺仪静差（静止时还是有数的  Q_Bias）
加速计与 陀螺仪角度误差
*******************/


/*****************
卡尔曼增量用的（跟据公式和此题条件 卡尔曼增量为二维向量 就是 下面的K1，K2 ）
我们假定H固定二维向量{1,0}根据公式
都是些中间变量
先求二维矩阵
再求分母
*******************/
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;

/********
t1 t2 是下一步用观测值求X（k|k）时用到的中间变量，因为也是进行矩阵乘法；
*********/

/*****************
下面的二维数组就是估计协方差矩阵
Pdot【4】则是保存矩阵乘法过程的中间变量 ，最后会赋值给矩阵的 
*******************/
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

void Kalman_Filter_X(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_X_Final += (Gyro - Q_bias) * dt; //先验估计
	
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= -PP[1][1];
	Pdot[2]= -PP[1][1];
	Pdot[3]= Q_gyro;
	
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_X_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0]; 
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	//以上两个为卡尔曼增益
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];


	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差，更新协方差 为下次迭代做准备
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	//又是矩阵计算，得到结果是 X（K|K）最优估计值
		
	Angle_X_Final += K_0 * Angle_err;	 //后验估计
	Q_bias        += K_1 * Angle_err;	 //后验估计
	//为了PID控制 求出角速度
	Gyro_x         = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}
/****变量***/
float PPY[2][2] = { { 1, 0 },{ 0, 1 } };
float Q_biasy=0;
void Kalman_Filter_Y(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_Y_Final += (Gyro - Q_biasy) * dt; //先验估计
	
	Pdot[0]=Q_angle - PPY[0][1] - PPY[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= - PPY[1][1];
	Pdot[2]= - PPY[1][1];
	Pdot[3]=Q_gyro;
	
	PPY[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PPY[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PPY[1][0] += Pdot[2] * dt;
	PPY[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Y_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PPY[0][0];
	PCt_1 = C_0 * PPY[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PPY[0][1];


	PPY[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PPY[0][1] -= K_0 * t_1;
	PPY[1][0] -= K_1 * t_0;
	PPY[1][1] -= K_1 * t_1;
		
	Angle_Y_Final	+= K_0 * Angle_err;	 //后验估计
	Q_biasy	+= K_1 * Angle_err;	 //后验估计
	Gyro_y   = Gyro - Q_biasy;	 //输出值(后验估计)的微分=角速度
}


float PPZ[2][2] = { { 1, 0 },{ 0, 1 } };
float Q_biasz=0;
void Kalman_Filter_Z(float Accel,float Gyro) //卡尔曼函数		
{
	Angle_Z_Final += (Gyro - Q_biasz) * dt; //先验估计
	
	Pdot[0]=Q_angle - PPZ[0][1] - PPZ[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]= - PPZ[1][1];
	Pdot[2]= - PPZ[1][1];
	Pdot[3]=Q_gyro;
	
	PPZ[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PPZ[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PPZ[1][0] += Pdot[2] * dt;
	PPZ[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - Angle_Z_Final;	//zk-先验估计
	
	PCt_0 = C_0 * PPZ[0][0];
	PCt_1 = C_0 * PPZ[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PPZ[0][1];


	PPZ[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PPZ[0][1] -= K_0 * t_1;
	PPZ[1][0] -= K_1 * t_0;
	PPZ[1][1] -= K_1 * t_1;
		
	Angle_Z_Final	+= K_0 * Angle_err;	 //后验估计
	Q_biasz	+= K_1 * Angle_err;	 //后验估计
	Gyro_z   = Gyro - Q_biasz;	 //输出值(后验估计)的微分=角速度
}

