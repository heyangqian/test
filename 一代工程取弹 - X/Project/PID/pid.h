#ifndef _PID_H_
#define _PID_H_
#include "main.h"
#include "stm32f4xx.h"
typedef struct
{
	float SetNum;       //�趨ֵ
	float ActualNum;    //ʵ��ֵ
	float Act_num;     //ʵ��ֵ����ֵ
	float Set_num;      //�趨ֵ����ֵ
	float error[3];          //ʵ�����趨���ֵ��3�Σ�
	float output;      //���ֵ������ִ�����ı����� 
	float output_last;   //��һ�ε����ֵ
	float integral;     //����ֵ
	float Kp;      //��������
	float Ki;      //���ֺͲ���
	float Kd;      //΢�ֲ���
	float Ti;      //����ʱ��
	float Td;      //΢��ʱ��
	float differential[3];//���Ĳ�ֵ
	float P_out;   //�������
	float I_out;   //�������
	float D_out;   //΢�����
	int16_t angle_now;   //��ǰ��е�Ƕ�
	int16_t angle_last;  // ��һ�λ�е�Ƕ�
	int16_t first_angle; //�ϵ���һ�λ�е�Ƕ�
	float err;       //���λ�е�Ƕ����
//	struct
//	{
//		float err[7];
//		float ec_err[7];
//		float NB,NM,NS,ZO,PS,PM,PB;
//		
//	} Fuzzy;
 
} PIDxtype;
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

extern int8_t Add_flag;     //λ�û����Ա�������ʼֵ����־λ
extern PIDxtype Claw_p_pid;   //ץ��צ��λ��pid��һ����
extern PIDxtype Claw_s_pid;   //ץ��צ���ٶ�pid��һ����
extern PIDxtype Claw2_p_pid;   //ץ��צ��λ��pid��һ����
extern PIDxtype Claw2_s_pid;   //ץ��צ���ٶ�pid��һ����
extern PIDxtype Send_p_pid;    //�͵�λ��pid
extern PIDxtype Send_s_pid;    //�͵��ٶ�pid

extern PIDxtype x_p_pid;      //ץ��xƽ��λ��pid
extern PIDxtype x_s_pid;      //ץ��xƽ���ٶ�pid
//extern PIDxtype x2_p_pid;
//extern PIDxtype x2_s_pid;

extern float Pit;
void pid_MInit(PIDxtype *pid, float Kp , float Ki , float Kd);  //���pid��ʼ��
void pid_GInit(PIDxtype *pid, float Kp , float Ti , float Td);  //��̨�����ʼ��
void pid_Posit(PIDxtype *pid,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout);//λ��ʽ   �ٶȻ�  ���ַ���
void Data_sends(int16_t data1,int16_t data2,int16_t data3,int16_t data4,int16_t data5,int16_t data6);   //��������
void Increment_Angle(PIDxtype *PId_p,int16_t an_now);//���Ա�������λ�û���
void pid_Increment(PIDxtype *PId_i,float actualnum ,int16_t setnum,int16_t maxout);  //����pid  
void speed_accelerate(float setspeed,float *speed_transit,int16_t acc,int16_t  dec);// ���ٶ�
void acclerate(float setnum,float *actnum,int16_t acc,int16_t dec);  //��д���ٶ�
void pid_incomplete(PIDxtype *PId_i,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout);  // ����ȫ΢��
#endif

