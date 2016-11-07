#include <STC12C5A60S2.H> 
#include <stdio.h>   //Keil library 
#include <INTRINS.H>
typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;

/**********为了匿名四轴上位机的协议定义的变量****************************/
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

uchar data_to_send[23];
int Acc_X;      //显示X轴角度
int Acc_Y;
int Acc_Z;
int Gyr_X;   //显示X轴角速度
int Gyr_Y;
int Gyr_Z;
uchar Baby[25]={"good morning everyones"};



//****************************************
// 定义51单片机端口
//****************************************
sbit    SCL=P2^0;   //IIC时钟引脚定义
sbit    SDA=P2^1;   //IIC数据引脚定义
//****************************************
// 定义MPU6050内部地址
//****************************************
#define SMPLRT_DIV  0x19 //陀螺仪采样率，典型值：0x07(125Hz)
#define CONFIG   0x1A //低通滤波频率，典型值：0x06(5Hz)
#define GYRO_CONFIG  0x1B //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define ACCEL_CONFIG 0x1C //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H  0x41
#define TEMP_OUT_L  0x42
#define GYRO_XOUT_H  0x43
#define GYRO_XOUT_L  0x44 
#define GYRO_YOUT_H  0x45
#define GYRO_YOUT_L  0x46
#define GYRO_ZOUT_H  0x47
#define GYRO_ZOUT_L  0x48
#define PWR_MGMT_1  0x6B //电源管理，典型值：0x00(正常启用)
#define WHO_AM_I  0x75 //IIC地址寄存器(默认数值0x68，只读)
#define SlaveAddress 0xD0 //IIC写入时的地址字节数据，+1为读取
//函数声明
//****************************************
int GetData(uchar REG_Address);
void init_uart();
void  SeriPushSend(uchar send_data);
void Uart1_Put_Buf(uchar *data_to_send,uchar _cnt);
void Data_Send_Senser(void);



//MPU6050操作函数
void  InitMPU6050();             //初始化MPU6050
void  Delay5us();
void  I2C_Start();
void  I2C_Stop();
void  I2C_SendACK(bit ack);
bit   I2C_RecvACK();
void  I2C_SendByte(uchar dat);
uchar I2C_RecvByte();
uchar Single_ReadI2C(uchar REG_Address);      //读取I2C数据
void  Single_WriteI2C(uchar REG_Address,uchar REG_data); //向I2C写入数据
//****************************************
//**************************************
//延时5微秒(STC90C52RC@12M)
//不同的工作环境,需要调整此函数
//当改用1T的MCU时,请调整此延时函数
//**************************************
void Delay5us()
{
_nop_();_nop_();_nop_();_nop_();
_nop_();_nop_();_nop_();_nop_();
_nop_();_nop_();_nop_();_nop_();
_nop_();_nop_();_nop_();_nop_();
_nop_();_nop_();_nop_();_nop_();
_nop_();_nop_();_nop_();_nop_();
}
//**************************************
//I2C起始信号
//**************************************
void I2C_Start()
{
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
}
//**************************************
//I2C停止信号
//**************************************
void I2C_Stop()
{
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
}
//**************************************
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(bit ack)
{
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
}
//**************************************
//I2C接收应答信号
//**************************************
bit I2C_RecvACK()
{
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
    return CY;
}
//**************************************
//向I2C总线发送一个字节数据
//**************************************
void I2C_SendByte(uchar dat)
{
    uchar i;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    I2C_RecvACK();
}
//**************************************
//从I2C总线接收一个字节数据
//**************************************
uchar I2C_RecvByte()
{
    uchar i;
    uchar dat = 0;
    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    return dat;
}
//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_WriteI2C(uchar REG_Address,uchar REG_data)
{
    I2C_Start();                  //起始信号
    I2C_SendByte(SlaveAddress);   //发送设备地址+写信号
    I2C_SendByte(REG_Address);    //内部寄存器地址，
    I2C_SendByte(REG_data);       //内部寄存器数据，
    I2C_Stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
uchar Single_ReadI2C(uchar REG_Address)
{
uchar REG_data;
I2C_Start();                   //起始信号
I2C_SendByte(SlaveAddress);    //发送设备地址+写信号
I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始 
I2C_Start();                   //起始信号
I2C_SendByte(SlaveAddress+1);  //发送设备地址+读信号
REG_data=I2C_RecvByte();       //读出寄存器数据
I2C_SendACK(1);                //接收应答信号
I2C_Stop();                    //停止信号
return REG_data;
}

//*********************************************************
//主程序
//*********************************************************
void main()
{ 
init_uart();
InitMPU6050(); //初始化MPU6050
while(1)
{
  //    32768.0*180先将从MPU6050中读取到的16位的数据转换10进制的角度
     Acc_X=GetData(ACCEL_XOUT_H);   //显示X轴角度
  Acc_Y=GetData(ACCEL_YOUT_H);  //显示Y轴角度
  Acc_Z=GetData(ACCEL_ZOUT_H);  //显示Z轴角度
     Gyr_X=GetData(GYRO_XOUT_H);   //显示X轴角速度
  Gyr_Y=GetData(GYRO_YOUT_H);  //显示Y轴角速度
  Gyr_Z=GetData(GYRO_ZOUT_H);     //显示Z轴角速度

  Data_Send_Senser();

//  Uart1_Put_Buf(Baby,25);       //测试串口程序有没有用
//  SeriPushSend(0x0d); 
//        SeriPushSend(0x0a);//换行，回车
}
}



//**************************************
//初始化MPU6050
//**************************************
void InitMPU6050()
{
Single_WriteI2C(PWR_MGMT_1, 0x00); //解除休眠状态
Single_WriteI2C(SMPLRT_DIV, 0x07);
Single_WriteI2C(CONFIG, 0x06);
Single_WriteI2C(GYRO_CONFIG, 0x18);
Single_WriteI2C(ACCEL_CONFIG, 0x01);
}
//**************************************
//合成数据
//**************************************
int GetData(uchar REG_Address)
{
uchar H,L;
H=Single_ReadI2C(REG_Address);
L=Single_ReadI2C(REG_Address+1);
return (H<<8)+L;   //合成数据
}
//**************************************
//在匿名上位机上显示数据
//**************************************
void init_uart()
{
  TMOD=0x20;    
TH1=0xfd;    
TL1=0xfd;  
TR1=1;
    SM0=0;
  SM1=1;
REN=1;
EA=1;
ES=1; 
// TI=1;

}
void  SeriPushSend(uchar send_data)
{
     ES=0;
    SBUF=send_data;  
while(!TI);
TI=0;
ES=1;   
}
void Uart1_Put_Buf(uchar *data_to_send,uchar _cnt)
{ 
    uchar i;
for(i=0;i<_cnt;i++)
{
        SeriPushSend(data_to_send);
    }
}


void Data_Send_Senser(void)  //按照协议的要求处理数据
{
uchar  _cnt=0;
uchar  i=0;
uchar sum = 0;

data_to_send[_cnt++]=0xAA;
data_to_send[_cnt++]=0xAA;
data_to_send[_cnt++]=0x02;
data_to_send[_cnt++]=0;
data_to_send[_cnt++]=BYTE1(Acc_X);  //高8位
data_to_send[_cnt++]=BYTE0(Acc_X);  //低8位
data_to_send[_cnt++]=BYTE1(Acc_Y);
data_to_send[_cnt++]=BYTE0(Acc_Y);
data_to_send[_cnt++]=BYTE1(Acc_Z);
data_to_send[_cnt++]=BYTE0(Acc_Z);
data_to_send[_cnt++]=BYTE1(Gyr_X);
data_to_send[_cnt++]=BYTE0(Gyr_X);
data_to_send[_cnt++]=BYTE1(Gyr_Y);
data_to_send[_cnt++]=BYTE0(Gyr_Y);
data_to_send[_cnt++]=BYTE1(Gyr_Z);
data_to_send[_cnt++]=BYTE0(Gyr_Z);
data_to_send[_cnt++]=0;
data_to_send[_cnt++]=0;
data_to_send[_cnt++]=0;
data_to_send[_cnt++]=0;
data_to_send[_cnt++]=0;
data_to_send[_cnt++]=0;

data_to_send[3] = _cnt-4;

for(i=0;i<_cnt;i++)
  sum += data_to_send;
data_to_send[_cnt++] = sum;
Uart1_Put_Buf(data_to_send,_cnt);
}
