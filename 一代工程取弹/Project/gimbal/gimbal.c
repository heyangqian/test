#include "gimbal.h"
#include "start.h"
#include "main.h"
#include "can1.h"
#include "pid.h"
#include "chassis.h"
#include "motor.h"
#include "can_task.h"

	/*FreeRTOS系统*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "semphr.h" 


gyro_measure_t gyro_yaw;
PIDxtype gimbal_yaw_speed;
PIDxtype gimbal_yaw_angle;
PIDxtype motor_yaw_angle;
PIDxtype gimbal_pitch_speed;
PIDxtype gimbal_pitch_angle;

/***************Chassis*****************
	函数名：GimbalCloud_Task
	作  用： 云台任务
	参	数：
******************************************/
float pitch_speed = 3000;
int32_t angle_yaw = 0;
int16_t angle_pitch = 7500;   //步兵1 1550   英雄 2500   步兵2 7500
//void Gimbal_task(void *pvParameter)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 1; 
//	xLastWakeTime = xTaskGetTickCount();
//	while(1)
//	{
////		if(rc_ctrl.rc .ch2 > 0)
////		{
////			angle_yaw+=30;
////		}
////		else if(rc_ctrl.rc .ch2 < 0)
////		{
////			angle_yaw-=30;
////		}
////		pid_incomplete(&gimbal_yaw_angle,gyro_yaw.temp_yaw_angle,angle_yaw,500,10000,16384);
////		pid_incomplete(&gimbal_yaw_speed,gyro_yaw.Angular_velocity,gimbal_yaw_angle.output,500,10000,30000);
////		Gimbal_Can2TX(0,-gimbal_yaw_speed.output,0,0);//方向相反负号输出
////		printf("1");
//		vTaskDelayUntil(&xLastWakeTime, xFrequency);
//	}
//	
//}


void gyro_Init(gyro_measure_t *xJxun)  //小黑块数据初始化
{
	xJxun->angle_Business_value = 0;
	xJxun->angle_remainder =0;
	xJxun->Angular_velocity = 0;
	xJxun->Last_yaw_angle = 0;
	xJxun->temp_yaw_angle = 0;
	xJxun->This = 0;
	xJxun->yaw_angle_nums = 0;
}
void Gimbal_Motor_Init()     //云台电机初始化
{
	pid_MInit(&gimbal_yaw_angle,0.585,0,53);   //位置环     陀螺仪yaw角度 0.585 0 53       6623pitch机械角度  
	pid_MInit(&gimbal_yaw_speed,26,0.43,8);   //速度环        8  0.8  13    陀螺仪yaw角速度   26 0.43 8     陀螺仪pitch角速度  
	pid_GInit(&gimbal_pitch_angle,0.9,100,0.5);   //  步兵M1.8 0.15 3  7500G0.9 100 0.5   英雄 G0.9 100 1
	pid_GInit(&gimbal_pitch_speed,20,33,1.5);   // 步兵 M9 0.2 5    7500G20 33 1.5    英雄 G20 36 1.8
	pid_GInit(&motor_yaw_angle,3,0,0);//yaw机械角度 底盘跟随
}
void get_gyro_measuer(gyro_measure_t *Jxun,CanRxMsg *msg)		//小黑块数据
{
	Jxun->temp_yaw_angle = (int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16)
														| (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3]);
	Jxun->Last_yaw_angle = Jxun->This;
	Jxun->This = (float)(Jxun->temp_yaw_angle*0.01);
	Jxun->Angular_velocity = 1600*(Jxun->This - Jxun->Last_yaw_angle);
	Jxun->yaw_angle_nums += (Jxun->This - Jxun->Last_yaw_angle)*100 ;
}

