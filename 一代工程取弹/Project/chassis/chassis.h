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
void Chassis_Motor_Init(void);  //底盘电机pid初始化
void Chassis_Init(ChassisTypeDef *Chassis);   //底盘初始化
void wheel_speed_calculate(ChassisTypeDef *Chassis,float set_Vx,float set_Vy,float set_Vz,float size);  //底盘公式
void chassis_move(float set_Vx,float set_Vy,float set_Vz,float size);  //底盘运动
void chassis_circle(int16_t setspeed,int16_t dir);  //底盘画圆
void chassis_anglecircle(int16_t setspeed,int16_t dir);  //底盘角度画圆
void chassis_rotate_circle(float set_Vx,float set_Vy,float set_Vz,float size,int16_t dir);//旋转画圆
void chassis_rotate_forword(float set_Vx,float set_Vy,float set_Vz,float size,int16_t dir);//旋转前进
#endif
