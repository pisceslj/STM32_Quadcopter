
#define Kp 100.0f                        // 比例修正陀螺仪静差
#define Ki 0.002f                // 积分修正角度的静差
#define halfT 0.0005f                // 采样周期；下次进行运算的时间间隔

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;          // 四元数的元素，代表估计方向
float exInt = 0, eyInt = 0, ezInt = 0;        // 按比例缩小积分误差

float Yaw,Pitch,Roll;  //偏航角，俯仰角，翻滚角

//只有单位四元数才可以表示旋转，至于为什么，因为这就是四元数表示旋转的约束条件。
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az) {
float norm;
float vx, vy, vz;
float ex, ey, ez;      
   
//单位化加速度计
norm = sqrt(ax*ax + ay*ay + az*az);      
ax = ax / norm;
ay = ay / norm;
az = az / norm;

/****
飞行器上次计算得到的姿态（四元数）换算成“方向余弦矩阵”中的第三列的三个元素。
根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
所以这里的halfvx、halfvy、halfvz，
其实就是用上一次飞行器机体姿态（四元数）换算出来的在当前的机体坐标系上的重力单位向量。
叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
***/


// 这是根据方向余弦矩阵的出来的，
//vxyz是陀螺积分后的姿态来推算出的重力向量,
//最开始的估计方向的重力 估计得 q=【1 0 0 0 】代表  机体没转动OK
vx = 2*(q1*q3 - q0*q2);
vy = 2*(q0*q1 + q2*q3);
vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

//差成算出来的误差  就是陀螺仪测得重力方向 和 加速计测得重力方向的误差
ex = (ay*vz - az*vy);
ey = (az*vx - ax*vz);
ez = (ax*vy - ay*vx);

// integral error scaled integral gain 用叉积误差来做PI修正陀螺零偏
//叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
exInt = exInt + ex*Ki;
eyInt = eyInt + ey*Ki;
ezInt = ezInt + ez*Ki;


//PI修正陀螺  KP比例+上面的积分修正 
gx = gx + Kp*ex + exInt;
gy = gy + Kp*ey + eyInt;
gz = gz + Kp*ez + ezInt;

//四元数的微分方程
//一阶龙格库塔法更新四元数，halfT：陀螺采样的间隔 
q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

// 单位化四元数 只有单位化才能表征旋转OK
norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
q0 = q0 / norm;
q1 = q1 / norm;
q2 = q2 / norm;
q3 = q3 / norm;

//单位化后 意义在于单位化四元数在空间旋转时是不会拉伸的；
//公式 是由 世界坐标与机体坐标 的 方向余弦矩阵 得来的，用四元数表示了下；
Pitch  = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch 
Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
Yaw = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;  //YAW
}
