#include "gimbal.h"
#include "start.h"
#include "main.h"
#include "can1.h"
#include "pid.h"
#include "chassis.h"
#include "motor.h"
#include "can_task.h"

	/*FreeRTOSϵͳ*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  
#include "task.h"
#include "semphr.h" 


gyro_measure_t gyro_yaw;
PIDxtype gimbal_yaw_speed;
PIDxtype gimbal_yaw_angle;
PIDxtype motor_yaw_angle;
PIDxtype gimbal_pitch_speed;
PIDxtype gimbal_pitch_angle;

/***************Chassis*****************
	��������GimbalCloud_Task
	��  �ã� ��̨����
	��	����
******************************************/
float pitch_speed = 3000;
int32_t angle_yaw = 0;
int16_t angle_pitch = 7500;   //����1 1550   Ӣ�� 2500   ����2 7500
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
////		Gimbal_Can2TX(0,-gimbal_yaw_speed.output,0,0);//�����෴�������
////		printf("1");
//		vTaskDelayUntil(&xLastWakeTime, xFrequency);
//	}
//	
//}


void gyro_Init(gyro_measure_t *xJxun)  //С�ڿ����ݳ�ʼ��
{
	xJxun->angle_Business_value = 0;
	xJxun->angle_remainder =0;
	xJxun->Angular_velocity = 0;
	xJxun->Last_yaw_angle = 0;
	xJxun->temp_yaw_angle = 0;
	xJxun->This = 0;
	xJxun->yaw_angle_nums = 0;
}
void Gimbal_Motor_Init()     //��̨�����ʼ��
{
	pid_MInit(&gimbal_yaw_angle,0.585,0,53);   //λ�û�     ������yaw�Ƕ� 0.585 0 53       6623pitch��е�Ƕ�  
	pid_MInit(&gimbal_yaw_speed,26,0.43,8);   //�ٶȻ�        8  0.8  13    ������yaw���ٶ�   26 0.43 8     ������pitch���ٶ�  
	pid_GInit(&gimbal_pitch_angle,0.9,100,0.5);   //  ����M1.8 0.15 3  7500G0.9 100 0.5   Ӣ�� G0.9 100 1
	pid_GInit(&gimbal_pitch_speed,20,33,1.5);   // ���� M9 0.2 5    7500G20 33 1.5    Ӣ�� G20 36 1.8
	pid_GInit(&motor_yaw_angle,3,0,0);//yaw��е�Ƕ� ���̸���
}
void get_gyro_measuer(gyro_measure_t *Jxun,CanRxMsg *msg)		//С�ڿ�����
{
	Jxun->temp_yaw_angle = (int32_t)(msg->Data[0]<<24)|(int32_t)(msg->Data[1]<<16)
														| (int32_t)(msg->Data[2]<<8) | (int32_t)(msg->Data[3]);
	Jxun->Last_yaw_angle = Jxun->This;
	Jxun->This = (float)(Jxun->temp_yaw_angle*0.01);
	Jxun->Angular_velocity = 1600*(Jxun->This - Jxun->Last_yaw_angle);
	Jxun->yaw_angle_nums += (Jxun->This - Jxun->Last_yaw_angle)*100 ;
}

