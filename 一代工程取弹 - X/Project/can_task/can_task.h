#ifndef __CAN_TASK_H__
#define __CAN_TASK_H__
#include "main.h"


/****************CAN1*****************/
extern int8_t ele1_flag;      //�ж��ϵ�
extern int8_t ele2_flag;      //�ж��ϵ�
//extern int16_t rst_flag;     //�ж�3510��ʼ��е�Ƕ�
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
	CAN_3508_M5_ID = 0x205,
    CAN_3508_M6_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

u8 Can_Send_Msg(u8* msg,u8 len);
void Motor_201_204_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 );	//�����ٶ�
void Motor_205_208_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 );	//�����ٶ�
void Gimbal_20x_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 );   //can1����
void Can1_motor_handle(CanRxMsg *rx_message);   //���ݴ��� 0x201--208

/****************CAN2*****************/
extern uint16_t Mse[8];
void Mes_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 );	//can2 ������������
void Motor_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 );	//�������can2 ����
void Gimbal_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 ); //��̨����can2����
void Motor_201_204_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 );	//CAN2 ���̷�������

void Can2_motor_handle(CanRxMsg *rx_message);

/****************CAN2����*****************/
extern int16_t Message[8];
void Can2_Mes_handle(CanRxMsg *rx_message);
void Can1_Mes_handle(CanRxMsg *rx_message);

#endif


