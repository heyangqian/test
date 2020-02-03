#ifndef _MOTOR_H_
#define _MOTOR_H_
#include "main.h"
#include "stm32f4xx.h"

#define M_MODE 1        //  1  ÆÕÍ¨×¥µ¯   2 xºáÒÆ×¥µ¯£¨°´¼ü£©  3 xºáÒÆ×¥µ¯£¨Ò»¼ü£©
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

