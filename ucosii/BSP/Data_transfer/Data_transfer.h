#ifndef _DATA_TRANSFER_H_
#define _DATA_TRANSFER_H_
#include "stm32f10x.h"

typedef struct dt_flag{
		int send_senser;
	  int send_status;
		int send_rcdata;
	  int send_motopwm;
	  int send_power;
}dt_flag_t;

void Data_Exchange(void);
void Send_Data(u8 *dataToSend , u8 length);
static void Send_Check(u8 head, u8 check_sum);
void Data_Receive_Prepare(u8 data);
void Data_Receive_Anl(u8 *data_buf,u8 num);
void Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z,s32 bar);
void Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void Send_Power(u16 votage, u16 current);
void Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);

#endif

