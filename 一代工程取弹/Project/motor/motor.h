#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
#include "stm32f4xx.h"

//定义 x平移设定值
#define x_set_res 0
#define x_set_left -1000000
#define x_set_right 850000
//定移爪子抓弹设定值
#define claw_set_res 0
#define claw_set_graw 103000
#define claw_set_throw 38000
#define claw_set_back 5000
//定义送弹设定值
#define send_set_close 0
#define send_set_open -330000
//定移x横移限制
#define x_limit_left 1030000
#define x_limit_right 880000
#define x_limit_res -10000
//定义爪子抓弹限制
#define claw_limit_graw 105000
#define claw_limit_res -3000
#define claw_limit_throw 40000
#define claw_limit_back 2000
//定义送弹限制
#define send_open -350000
#define send_close 10000


#define M_MODE 1        //  1  普通抓弹   2 x横移抓弹（按键）  3 x横移抓弹（一键）
typedef struct
{
	int32_t motor_angle;
	int32_t motor_angle_first;
	int32_t motor_speed;
	int32_t motor_torque;
	float motor_temperate;
}Motor_Message;

extern Motor_Message Motor_y[8];
extern Motor_Message Motor_x[8];
extern int8_t mot_flag;
extern int8_t mox_flag;
extern int8_t fir_flag;
extern int16_t lock_flag;

void Motor_task(void *pvParameter);
void Motor_x_task(void *pvParameter);
void angle_first(Motor_Message *Motor,int16_t motor_angle);
void get_buttle_pid_Init(void);
void get_buttle_pid(void);
void get_buttle_pidNum_Init(void);
void get_buttle_position(void);
void x_set_change(void);
void claw_set_change(void);
void send_set_change(void);
void x_limit(void);
void claw_limit(void);
void send_limit(void);
#endif

