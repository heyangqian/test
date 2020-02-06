#ifndef _GIMBAL_H_
#define _GIMBAL_H_
#include "main.h"
#include "stm32f4xx.h"
#include "pid.h"
typedef struct
{
    float yaw_angle_nums;
    float Angular_velocity;//С�ڿ��ٶ�
    int temp_yaw_angle;//�Ƕ�
    int angle_remainder;
    int angle_Business_value;
    float Last_yaw_angle;
    float This;
}gyro_measure_t;

extern gyro_measure_t gyro_yaw;
extern PIDxtype gimbal_yaw_speed;
extern PIDxtype gimbal_yaw_angle;
extern PIDxtype gimbal_pitch_speed;
extern PIDxtype gimbal_pitch_angle;
extern PIDxtype motor_yaw_angle;
void Gimbal_task(void *pvParameter);
void Gimbal_Motor_Init(void);
void get_gyro_measuer(gyro_measure_t *Jxun,CanRxMsg *msg);		//С�ڿ�����
void gyro_Init(gyro_measure_t *xJxun);  //С�ڿ��ʼ��


#endif

