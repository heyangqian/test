#ifndef _CHASSIS_H_
#define _CHASSIS_H_
#include "main.h"
#include "pid.h"

typedef struct
{ 	
	float Vx;
	float Vy;
	float Vz;
	float Vx_set;
	float Vy_set;
	float Vz_set;
	float set_wheel_speed1;
	float set_wheel_speed2;
	float set_wheel_speed3;
	float set_wheel_speed4;
	float set_wheel_speed5;
	float set_wheel_speed6;
	float wheel_speed1;
	float wheel_speed2;
	float wheel_speed3;
	float wheel_speed4;
	float wheel_speed5;
	float wheel_speed6;
	PIDxtype Motor1;
	PIDxtype Motor2;
	PIDxtype Motor3;
	PIDxtype Motor4;
	PIDxtype Motor5;
	PIDxtype Motor6;
} ChassisTypeDef;
extern ChassisTypeDef motor_chassis;
extern int8_t cmv_flag;
extern int16_t ci_flag;
void Chassis_task(void *pvParameter);
void Chassis_Motor_Init(void);  //���̵��pid��ʼ��
void Chassis_Init(ChassisTypeDef *Chassis);   //���̳�ʼ��
void wheel_speed_calculate(ChassisTypeDef *Chassis,float set_Vx,float set_Vy,float set_Vz,float size);  //���̹�ʽ
void chassis_move(float set_Vx,float set_Vy,float set_Vz,float size);  //�����˶�
void chassis_circle(int16_t setspeed,int16_t dir);  //���̻�Բ
void chassis_anglecircle(int16_t setspeed,int16_t dir);  //���̽ǶȻ�Բ
void chassis_rotate_circle(float set_Vx,float set_Vy,float set_Vz,float size,int16_t dir);//��ת��Բ
void chassis_rotate_forword(float set_Vx,float set_Vy,float set_Vz,float size,int16_t dir);//��תǰ��
#endif
