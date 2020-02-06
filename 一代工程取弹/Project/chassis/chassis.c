#include "chassis.h"
#include "start.h"
#include "main.h"
#include "can1.h"
#include "pid.h"
#include "gimbal.h"
#include "motor.h"
#include "can_task.h"
#include "gas.h"


	/*FreeRTOS系统*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "semphr.h" 


#define  PI  3.1415

/***************Chassis*****************
	函数名：Chassis_Task
	作  用： 底盘任务
	参	数：
******************************************/
int8_t cmv_flag = 0;
int16_t ci_flag = 0;
void Chassis_task(void *pvParameter)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; 
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		//普通底盘移动
		chassis_move(rc_ctrl.rc .ch0 * 3,rc_ctrl.rc .ch1 * 3,-rc_ctrl.rc .ch2,3);
//		printf("3");
		//带小炮底盘跟随与移动
//		angle_first(&Motor_x[5],Motor_x[5].motor_angle);
//		if(Motor_x[5].motor_angle - Motor_x[5].motor_angle_first >= 4000 )
//		{
//			Motor_x[5].motor_angle = Motor_x[5].motor_angle - 8192;
//		}
//		else if(Motor_x[5].motor_angle - Motor_x[5].motor_angle_first <= -4000)
//		{
//			Motor_x[5].motor_angle = Motor_x[5].motor_angle_first + 8192;
//		}
//		pid_Posit(&motor_yaw_angle,Motor_x[5].motor_angle - Motor_x[5].motor_angle_first,0,500,13000,30000);
//		chassis_rotate_forword(rc_ctrl.rc .ch0 * 10,rc_ctrl.rc .ch1 * 10,-motor_yaw_angle.output,2,rc_ctrl.rc .s2);  //方向相反，负号输出
		
		
		//遥控器控制抓取第二箱时底盘移动（一代）
//		if(rc_ctrl.rc.s1 == 1 && cmv_flag == 0 && gas_flag == 0)   
//		{
//			if(rc_ctrl.rc.s2 == 1)
//			{
//				chassis_move(0,-1000,0,0);
//				ci_flag++;
//				if(ci_flag >= 80)
//				{
//					cmv_flag = 1;
//					ci_flag = 0;
//				}
//			}
//			else if(rc_ctrl.rc.s2 == 3)
//			{
//				chassis_move(0,1000,0,0);
//				ci_flag++;
//				if(ci_flag >= 80)
//				{
//					cmv_flag = 1;
//					ci_flag = 0;
//				}
//			}
//		}
//		else
//		{
//			chassis_move(rc_ctrl.rc .ch0 * 3,rc_ctrl.rc .ch1 * 3,-rc_ctrl.rc .ch2,3);
//		}
		
		
//		printf("%d",GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_11) );

		ele1_flag = 0;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

/***************Chassis*****************
	函数名：Chassis_Init
	作  用： 底盘数据初始化
	参	数：函数指针
******************************************/

void Chassis_Motor_Init()
{
	pid_MInit(&motor_chassis.Motor1,10,1.1,8);  //电机1初始化
	pid_MInit(&motor_chassis.Motor2,10,1.1,8);  //电机2初始化
	pid_MInit(&motor_chassis.Motor3,10,1.1,8);  //电机3初始化
	pid_MInit(&motor_chassis.Motor4,10,1.1,8);  //电机4初始化
	pid_MInit(&motor_chassis.Motor5,10,1.1,8);  //电机5初始化
	pid_MInit(&motor_chassis.Motor6,10,1.1,8);  //电机6初始化
}
ChassisTypeDef motor_chassis;
void Chassis_Init(ChassisTypeDef *Chassis)
{
	Chassis->Vx = 0;
	Chassis->Vx_set = 0;
	Chassis->Vy = 0;
	Chassis->Vy_set = 0;
	Chassis->Vz = 0;
	Chassis->Vz_set = 0;
	Chassis->set_wheel_speed1 = 0;
	Chassis->set_wheel_speed2 = 0;
	Chassis->set_wheel_speed3 = 0;
	Chassis->set_wheel_speed4 = 0;
	Chassis->wheel_speed1 = 0;
	Chassis->wheel_speed2 = 0;
	Chassis->wheel_speed3 = 0;
	Chassis->wheel_speed4 = 0;
}
/***************Chassis*****************
	函数名：wheel_speed_calculate
	作  用： 底盘速度计算
	参	数：函数指针
			set_Vx    x方向速度
			set_Vy    y方向速度
			set_Vz    角速度(欧米噶)
******************************************/
//void wheel_speed_calculate(ChassisTypeDef *Chassis,float set_Vx,float set_Vy,float set_Vz,float size) //6轮
//{
//	Chassis->set_wheel_speed1 = set_Vy + set_Vx + set_Vz * size;              //前左
//	Chassis->set_wheel_speed2 = -(set_Vy - set_Vx - set_Vz * size);              //前右
//	Chassis->set_wheel_speed3 = set_Vy;              //中左
//	Chassis->set_wheel_speed4 = -(set_Vy);             //中右
//	Chassis->set_wheel_speed5 = set_Vy - set_Vx + set_Vz * size;              //后左
//	Chassis->set_wheel_speed6 = -(set_Vy + set_Vx - set_Vz * size);              //后右
//}

void wheel_speed_calculate(ChassisTypeDef *Chassis,float set_Vx,float set_Vy,float set_Vz,float size)  //4轮
{
	Chassis->set_wheel_speed1 = set_Vy + set_Vx - set_Vz * size;              //前左
	Chassis->set_wheel_speed2 = -(set_Vy - set_Vx + set_Vz * size);              //前右
	Chassis->set_wheel_speed3 = set_Vy - set_Vx - set_Vz * size;              //后左
	Chassis->set_wheel_speed4 = -(set_Vy + set_Vx + set_Vz * size);              //后右
}

/***************Chassis*****************
	函数名：chassis_move
	作  用： 底盘运动
	参	数：
			set_Vx    x方向速度
			set_Vy    y方向速度
			set_Vz    角速度(欧米噶)
******************************************/
//void chassis_move(float set_Vx,float set_Vy,float set_Vz,float size)  //6轮
//{
//	wheel_speed_calculate(&motor_chassis,set_Vx,set_Vy,set_Vz,size);
//	speed_accelerate(motor_chassis.set_wheel_speed1,&motor_chassis.wheel_speed1,50,150);
//	speed_accelerate(motor_chassis.set_wheel_speed2,&motor_chassis.wheel_speed2,50,150);
//	speed_accelerate(motor_chassis.set_wheel_speed3,&motor_chassis.wheel_speed3,50,150);
//	speed_accelerate(motor_chassis.set_wheel_speed4,&motor_chassis.wheel_speed4,50,150);
//	speed_accelerate(motor_chassis.set_wheel_speed5,&motor_chassis.wheel_speed5,50,150);
//	speed_accelerate(motor_chassis.set_wheel_speed6,&motor_chassis.wheel_speed6,50,150);
//	pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.wheel_speed1,100,13000,16384);
//	pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.wheel_speed2,100,13000,16384);
//	pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.wheel_speed3,100,13000,16384);
//	pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.wheel_speed4,100,13000,16384);
//	pid_incomplete(&motor_chassis.Motor5,Motor_x[4].motor_speed,motor_chassis.wheel_speed5,100,13000,16384);
//	pid_incomplete(&motor_chassis.Motor6,Motor_x[5].motor_speed,motor_chassis.wheel_speed6,100,13000,16384);
//	Motor_201_204_Can1TX(motor_chassis.Motor1.output,motor_chassis.Motor2.output,motor_chassis.Motor3.output,motor_chassis.Motor4.output);
//	Motor_205_208_Can1TX(motor_chassis.Motor5.output,motor_chassis.Motor6.output,0,0);
//	ele1_flag = 0;
//}

void chassis_move(float set_Vx,float set_Vy,float set_Vz,float size)  //4轮
{
	wheel_speed_calculate(&motor_chassis,set_Vx,set_Vy,set_Vz,size);
	speed_accelerate(motor_chassis.set_wheel_speed1,&motor_chassis.wheel_speed1,15,15);
	speed_accelerate(motor_chassis.set_wheel_speed2,&motor_chassis.wheel_speed2,15,15);
	speed_accelerate(motor_chassis.set_wheel_speed3,&motor_chassis.wheel_speed3,15,15);
	speed_accelerate(motor_chassis.set_wheel_speed4,&motor_chassis.wheel_speed4,15,15);
	pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.wheel_speed1,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.wheel_speed2,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.wheel_speed3,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.wheel_speed4,100,13000,16384);
	Motor_201_204_Can1TX(motor_chassis.Motor1.output,motor_chassis.Motor2.output,motor_chassis.Motor3.output,motor_chassis.Motor4.output);
	ele1_flag = 0;
}
/***************Chassis*****************
	函数名：chassis_circle
	作  用： 爱的魔力转圈圈
	参	数：
			setspeed    设定转圈速度
******************************************/
float speed_x = 0,speed_y = 0;
int16_t s_flag = 0;
void chassis_circle(int16_t setspeed,int16_t dir)
{
	wheel_speed_calculate(&motor_chassis,speed_x,speed_y,0,100);
	pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.set_wheel_speed1,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.set_wheel_speed2,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.set_wheel_speed3,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.set_wheel_speed4,100,13000,16384);
	Motor_201_204_Can1TX(motor_chassis.Motor1.output,motor_chassis.Motor2.output,motor_chassis.Motor3.output,motor_chassis.Motor4.output);
	ele1_flag = 0;
	if(s_flag == 0)
	{
		if(dir == 1)
		{
			speed_x = 0;
//			speed_y = -setspeed;
			speed_accelerate(-setspeed,&speed_y,6,10);
			if(speed_y == -setspeed)
			{
				s_flag = 1;
			}
		}
		else if(dir == 2)
		{
			speed_x = 0;
//			speed_y = setspeed;
			speed_accelerate(setspeed,&speed_y,6,10);
			if(speed_y == setspeed)
			{
				s_flag = 1;
			}
//			printf("0\r\n");
		}
		else
		{
			s_flag = 0;
			speed_accelerate(0,&speed_x,6,10);
			speed_accelerate(0,&speed_y,6,10);
		}
	}
	if(s_flag == 1)
	{
		if(dir == 1)
		{
			speed_x+=2;
			speed_y+=2;
			if(speed_x == setspeed && speed_y ==0)
			{
				s_flag = 2;
			}
		}
		else if(dir == 2)
		{
			speed_x+=2;
			speed_y-=2;
			if(speed_x == setspeed && speed_y == 0)
			{
				s_flag = 2;
			}
//			printf("1\r\n");
		}
		else
		{
			s_flag = 0;
			speed_accelerate(0,&speed_x,6,10);
			speed_accelerate(0,&speed_y,6,10);
		}
	}
	if(s_flag == 2)
	{
		if(dir == 1)
		{
			speed_x-=2;
			speed_y+=2;
			if(speed_x == 0 && speed_y ==setspeed)
			{
				s_flag = 3;
			}
			
		}
		else if(dir == 2)
		{
			speed_x-=2;
			speed_y-=2;
			if(speed_x == 0 && speed_y == -setspeed)
			{
				s_flag = 3;
			}
//			printf("2\r\n");
		}
		else
		{
			s_flag = 0;
			speed_accelerate(0,&speed_x,6,10);
			speed_accelerate(0,&speed_y,6,10);
		}
	}
	if(s_flag == 3)
	{
		if(dir == 1)
		{
			speed_x-=2;
			speed_y-=2;
			if(speed_x == -setspeed && speed_y == 0)
			{
				s_flag = 4;
			}
		}
		else if(dir == 2)
		{
			speed_x-=2;
			speed_y+=2;
			if(speed_x == -setspeed && speed_y == 0)
			{
				s_flag = 4;
			}
//			printf("3\r\n");
		}
		else
		{
			s_flag = 0;
			speed_accelerate(0,&speed_x,6,10);
			speed_accelerate(0,&speed_y,6,10);
		}
	}
	if(s_flag == 4)
	{
		if(dir == 1)
		{
			speed_x+=2;
			speed_y-=2;
			if(speed_x == 0 && speed_y == -setspeed)
			{
				s_flag = 0;
			}
		}
		else if(dir == 2)
		{
			speed_x+=2;
			speed_y+=2;
			if(speed_x == 0 && speed_y == setspeed)
			{
				s_flag = 0;
			}
//			printf("4\r\n");
		}
		else
		{
			s_flag = 0;
			speed_accelerate(0,&speed_x,6,10);
			speed_accelerate(0,&speed_y,6,10);
		}
	}
}
/***************Chassis*****************
	函数名：chassis_anglecircle
	作  用： 爱的魔法转圈圈
	参	数：
			setspeed    设定转圈速度
******************************************/
float x_speed = 0,y_speed = 0;
uint16_t dir_last = 0,Ang = 0;
void chassis_anglecircle(int16_t setspeed,int16_t dir)
{
	wheel_speed_calculate(&motor_chassis,x_speed,y_speed,0,100);
	pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.set_wheel_speed1,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.set_wheel_speed2,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.set_wheel_speed3,100,13000,16384);
	pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.set_wheel_speed4,100,13000,16384);
	Motor_201_204_Can1TX(motor_chassis.Motor1.output,motor_chassis.Motor2.output,motor_chassis.Motor3.output,motor_chassis.Motor4.output);
	ele1_flag = 0;
	if(dir != dir_last)
	{
		Ang = 0;
	}
	if(dir == 1)
	{
		x_speed = sin((PI * Ang / 180)) * setspeed;
		y_speed = cos((PI * Ang / 180)) * setspeed;
	}
	if(dir == 2)
	{
		x_speed = sin(-(PI * Ang / 180))* setspeed;
		y_speed = cos(-(PI * Ang / 180))* setspeed;
	}
	if(dir == 3)
	{
//		x_speed = 0;
//		y_speed = 0;
		speed_accelerate(0,&x_speed,8,10);
		speed_accelerate(0,&y_speed,8,10);
	}
	Ang++;
	if(Ang ==360)
	{
		Ang = 0;
	}
	dir_last = dir;
}
/***************Chassis*****************
	函数名：chassis_rotate_circle
	作  用： 我们饶了这么一圈才遇见
	参	数：
			set_Vx   遥控器输入x轴速度
			set_Vy   遥控器输入y轴速度
			set_Vz   遥控器输入z轴速度
			dir      方向（顺时针  逆时针）
******************************************/

float Vx = 0,Vy = 0,Vz = 0,set_speed = 0;
float first_yaw = 0 , last_yaw = 0, err_yaw = 0;
float rotate_flag = 0;
void chassis_rotate_circle(float set_Vx,float set_Vy,float set_Vz,float size,int16_t dir)
{
	if(ele1_flag == 0)
	{
		Vx = 0;
		Vy = 0;
		Vz = 0;
		rotate_flag = 0;
	}
	wheel_speed_calculate(&motor_chassis,Vx,Vy,Vz,100);
	if(dir == 1 || dir == 2)
	{
		pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.set_wheel_speed1,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.set_wheel_speed2,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.set_wheel_speed3,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.set_wheel_speed4,100,13000,16384);
	}
	else
	{
		speed_accelerate(motor_chassis.set_wheel_speed1,&motor_chassis.wheel_speed1,50,150);
		speed_accelerate(motor_chassis.set_wheel_speed2,&motor_chassis.wheel_speed2,50,150);
		speed_accelerate(motor_chassis.set_wheel_speed3,&motor_chassis.wheel_speed3,50,150);
		speed_accelerate(motor_chassis.set_wheel_speed4,&motor_chassis.wheel_speed4,50,150);
		pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.wheel_speed1,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.wheel_speed2,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.wheel_speed3,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.wheel_speed4,100,13000,16384);
	}
	Motor_201_204_Can1TX(motor_chassis.Motor1.output,motor_chassis.Motor2.output,motor_chassis.Motor3.output,motor_chassis.Motor4.output);
	ele1_flag = 0;
	if(rotate_flag == 2)
	{
		err_yaw = Motor_x[5].motor_angle - last_yaw ;
		if(err_yaw >= 5000)
		{
			err_yaw -= 8192;
		}
		if(err_yaw <= -5000)
		{
			err_yaw += 8192;
		}
		Vz = set_Vz;
	}
	if(rotate_flag == 1)
	{
		err_yaw = Motor_x[5].motor_angle - first_yaw;
		rotate_flag = 2;
	}
	if(Motor_x[5].motor_angle != 0 && rotate_flag == 0)
	{
		first_yaw = Motor_x[5].motor_angle;
		rotate_flag = 1;
	}
	last_yaw = Motor_x[5].motor_angle;
	set_speed = sqrt(set_Vx*set_Vx + set_Vy*set_Vy);
	if(dir == 1)
	{
		Vx = sin((2*PI * err_yaw / 8192)) * set_speed;
		Vy = cos((2*PI * err_yaw / 8192)) * set_speed;
	}
	if(dir == 2)
	{
		Vx = sin(-(2*PI * err_yaw / 8192)) * set_speed;
		Vy = cos(-(2*PI * err_yaw / 8192)) * set_speed;
	}
	if(dir == 3 || dir == 0)
	{
		Vx = set_Vx;
		Vy = set_Vy;
		Vz = set_Vz;
	}
	
}

/***************Chassis*****************
	函数名：chassis_rotate_forword
	作  用： 底盘跟随+小陀螺前进
	参	数：
			set_Vx   遥控器输入x轴速度
			set_Vy   遥控器输入y轴速度
			set_Vz   遥控器输入z轴速度
			dir      方向（1顺时针  2逆时针 3和0底盘跟随）
******************************************/
int16_t flag_rotate = 0;
float speed_set = 0;
void chassis_rotate_forword(float set_Vx,float set_Vy,float set_Vz,float size,int16_t dir)
{
	if(ele1_flag == 0)
	{
		Vx = 0;
		Vy = 0;
		Vz = 0;
		flag_rotate = 0;
	}
	wheel_speed_calculate(&motor_chassis,Vx,Vy,Vz,size);
	if(dir == 1 || dir == 2)
	{
		pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.set_wheel_speed1,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.set_wheel_speed2,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.set_wheel_speed3,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.set_wheel_speed4,100,13000,16384);
	}
	else
	{
		speed_accelerate(motor_chassis.set_wheel_speed1,&motor_chassis.wheel_speed1,50,150);
		speed_accelerate(motor_chassis.set_wheel_speed2,&motor_chassis.wheel_speed2,50,150);
		speed_accelerate(motor_chassis.set_wheel_speed3,&motor_chassis.wheel_speed3,50,150);
		speed_accelerate(motor_chassis.set_wheel_speed4,&motor_chassis.wheel_speed4,50,150);
		pid_incomplete(&motor_chassis.Motor1,Motor_x[0].motor_speed,motor_chassis.wheel_speed1,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor2,Motor_x[1].motor_speed,motor_chassis.wheel_speed2,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor3,Motor_x[2].motor_speed,motor_chassis.wheel_speed3,100,13000,16384);
		pid_incomplete(&motor_chassis.Motor4,Motor_x[3].motor_speed,motor_chassis.wheel_speed4,100,13000,16384);
	}
	Motor_201_204_Can1TX(motor_chassis.Motor1.output,motor_chassis.Motor2.output,motor_chassis.Motor3.output,motor_chassis.Motor4.output);
	ele1_flag = 0;
	if(flag_rotate == 2)
	{
		err_yaw = Motor_x[5].motor_angle - last_yaw ;
		if(err_yaw >= 5000)
		{
			err_yaw -= 8192;
		}
		if(err_yaw <= -5000)
		{
			err_yaw += 8192;
		}
	}
	if(flag_rotate == 1)
	{
		err_yaw = Motor_x[5].motor_angle - first_yaw;
		rotate_flag = 2;
	}
	if(Motor_x[5].motor_angle != 0 && flag_rotate == 0)
	{
		first_yaw = Motor_x[5].motor_angle;
		flag_rotate = 1;
	}
	last_yaw = Motor_x[5].motor_angle;
	set_speed = sqrt(set_Vx*set_Vx + set_Vy*set_Vy);
	if(dir == 1)
	{
		Vx = cos((2*PI * (err_yaw / 8192))) * set_Vx - sin((2*PI * (err_yaw / 8192)))*set_Vy;
		Vy = sin((2*PI * (err_yaw / 8192))) * set_Vx + cos((2*PI * (err_yaw / 8192)))*set_Vy;
		Vz = 3000;
	}
	if(dir == 2)
	{
		Vx = cos((2*PI * (err_yaw / 8192))) * set_Vx - sin((2*PI * (err_yaw / 8192)))*set_Vy;
		Vy = sin((2*PI * (err_yaw / 8192))) * set_Vx + cos((2*PI * (err_yaw / 8192)))*set_Vy;
		Vz = 3000;
	}
	if(dir == 3 || dir == 0)
	{
		Vx = cos((2*PI * (err_yaw / 8192))) * set_Vx - sin((2*PI * (err_yaw / 8192)))*set_Vy;
		Vy = sin((2*PI * (err_yaw / 8192))) * set_Vx + cos((2*PI * (err_yaw / 8192)))*set_Vy;
		Vz = set_Vz;
	}
}
