#include "motor.h"
#include "pid.h"
#include "can1.h"
#include "can_task.h"
#include "gas.h"

	/*FreeRTOS系统*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "semphr.h" 
Motor_Message Motor_x[8];
Motor_Message Motor_y[8];

/***************DATA*****************
	函数名：angle_first
	作  用： 计算位置环初始值
	参	数： 
******************************************/
int8_t fir_flag = 0;
int16_t motor_angle_first = 0,motor_angle_err = 0,motor_angle_last = 0;
void angle_first(Motor_Message *Motor,int16_t motor_angle)
{
	if(motor_angle != 0 && fir_flag == 0)
	{
		motor_angle_first =  motor_angle;
		fir_flag = 1;
	}
	if(fir_flag == 1)
	{
		Motor->motor_angle_first = motor_angle_first;
	}
	fir_flag = 0;
}

int8_t mot_flag = 0;  //爪子电机到达标志位
//int16_t turn_flag = 0;
int8_t mox_flag = 0; //x平移电机位置到达标志位
int16_t i1_flag = 0,i2_flag = 0,i3_flag = 0,i4_flag = 0;//测试专用(i1_flag程序执行次数标志位，i2_flag 扔箱子专用检测执行次数标志位)
int8_t row_flag = 0;  //扔箱子标准位
int16_t lock_flag = 0; //堵转检测

#if M_MODE == 1
///***************************************************普通抓弹（电机）************************************/
void Motor_task(void *pvParameter)    //抓弹电机控制调试
{
	get_buttle_pid_Init();
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; 
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(rc_ctrl.rc .s1 == 1 || rc_ctrl.rc .s1 == 2 || rc_ctrl.rc .s1 == 3)
		{
			if(ele2_flag == 0)
			{
				get_buttle_pidNum_Init();
				gas_flag = 0;
			}			
			//x轴设定值的改变
			x_set_change();
			//送弹设定值变化
			send_set_change();
			//爪子设定值改变
			claw_set_change();
			//  绝对变增量
			get_buttle_position();
			//x平移实际值限定
			x_limit();
			//送弹实际值限定
			send_limit();
			//爪子实际值限定
			claw_limit();
			//pid计算
			get_buttle_pid();
			 //误差判断
			if(x_p_pid.error[0] <= 3000 && x_p_pid.error[0] >= -3000)
			{
				if(mov_flag == 1)
				{
					mox_flag = 1;       //  x轴左平移完发送，执行爪子抓取
				}
				else if(mov_flag == 2)
				{
					mox_flag = 2;       //  x轴右平移完发送，执行爪子抓取
				}
				else if(mov_flag == 3)
				{
					mox_flag = 0;       //x轴归位后发送，复位
				}
			}
			if(Claw_p_pid.error[0] <= 300 && Claw_p_pid.error[0] >= -300)         
			{
				
				if(gas_flag == 1)
				{
					mot_flag = 1;
					i1_flag++;
					if(gas_flag == 1 && i1_flag >= 150)
					{
						mot_flag = 2;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
				}
				else if(gas_flag == 2)
				{
					mot_flag = 3;
					i1_flag++;
					if(gas_flag == 2 && i1_flag >= 350)
					{
						mot_flag = 4;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
//					printf("%d\r\n",i1_flag);
				}
				else if(gas_flag == 3)
				{
					i1_flag++;
					if(gas_flag == 3 && i1_flag>=1 && i1_flag<=10)
					{
						mot_flag = 5;
					}
					else if(gas_flag == 3 && i1_flag>=200)
					{
						mot_flag = 6;
						i1_flag = 0;
						i2_flag = 0;
					}
				}
				else if(gas_flag == 4)
				{
					mot_flag = 7;
					i1_flag++;
					if(gas_flag == 4 && i1_flag>=300)
					{
						mot_flag = 7;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
//					printf("%d\r\n",i1_flag);
				}
			}
			if(row_flag == 0 && i2_flag >= 100)   //  扔箱子检测
			{
				i2_flag = 0;
			}
			else if(row_flag == 1 && i2_flag >=13 && i2_flag <100)
			{
				mot_flag = 5;
				row_flag = 0;
			}
			
		}
		else
		{
			if(ele2_flag == 0)
			{
				get_buttle_pidNum_Init();
				gas_flag = 0;
				mot_flag = 0;
				mov_flag = 0;
			}
			//绝对变增量
			get_buttle_position();
			// x横移实际值限定
			x_limit();
			//送弹实际值限定
			send_limit();
			//爪子实际值限定
			claw_limit();
			//pid计算
			get_buttle_pid();
			gas_flag = 0;
			mot_flag = 0;
			mov_flag = 0;
		}
		ele2_flag = 0;
//		printf("2");
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

#elif M_MODE == 2

/**********************************************************x平移抓弹(全按键)************************************/
int xxxx = 0;
void Motor_task(void *pvParameter)
{
	get_buttle_pid_Init();
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; 
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(rc_ctrl.rc .s1 == 1 || rc_ctrl.rc .s1 == 3 )
		{
			if(ele2_flag == 0)
			{
				get_buttle_pidNum_Init();
				gas_flag = 0;
			}	
			//x轴设定值的改变
			x_set_change();
			//爪子设定值的改变
			claw_set_change();
			//送弹设定值变化
			send_set_change();
			//绝对变增量
			get_buttle_position();
			//x平移实际值限定
			x_limit();
			//送弹实际值限定
			send_limit();
			//爪子实际值限定
			claw_limit();
			//pid计算
			get_buttle_pid();
			 //误差判断
			if(x_p_pid.error[0] <= 3000 && x_p_pid.error[0] >= -3000)
			{
				if(mov_flag == 1)
				{
					mox_flag = 1;       //  x轴左平移完发送，执行爪子抓取
				}
				else if(mov_flag == 2)
				{
					mox_flag = 2;       //  x轴右平移完发送，执行爪子抓取
				}
				else if(mov_flag == 3)
				{
					mox_flag = 0;       //x轴归位后发送，复位
				}
			}
			if(Claw_p_pid.error[0] <= 300 && Claw_p_pid.error[0] >= -300)         
			{
				
				if(gas_flag == 1)
				{
					mot_flag = 1;
					i1_flag++;
					if(gas_flag == 1 && i1_flag >= 150)
					{
						mot_flag = 2;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
				}
				else if(gas_flag == 2)
				{
					mot_flag = 3;
					i1_flag++;
					if(gas_flag == 2 && i1_flag >= 350)
					{
						mot_flag = 4;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
//					printf("%d\r\n",i1_flag);
				}
				else if(gas_flag == 3)
				{
					i1_flag++;
					if(gas_flag == 3 && i1_flag>=3 && i1_flag<=10)
					{
						mot_flag = 5;
					}
					else if(gas_flag == 3 && i1_flag>=200)
					{
						mot_flag = 6;
						i1_flag = 0;
						i2_flag = 0;
					}
				}
				else if(gas_flag == 4)
				{
					mot_flag = 7;
					i1_flag++;
					if(gas_flag == 4 && i1_flag>=300)
					{
						mot_flag = 7;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
//					printf("%d\r\n",i1_flag);
				}
			}
			if(row_flag == 0 && i2_flag >= 100)   //  扔箱子检测
			{
				i2_flag = 0;
			}
			else if(row_flag == 1 && i2_flag >=13 && i2_flag <100)
			{
				mot_flag = 5;
				row_flag = 0;
			}
			
		}
		else
		{
			if(ele2_flag == 0)
			{
				get_buttle_pidNum_Init();
				gas_flag = 0;
				mot_flag = 0;
				mov_flag = 0;
			}
			//绝对变增量
			get_buttle_position();
			x_limit();
			//送弹实际值限定
			send_limit();
			//爪子实际值限定
			claw_limit();
			//pid计算
			get_buttle_pid();
			gas_flag = 0;
			mot_flag = 0;
			mov_flag = 0;
		}
		ele2_flag = 0;
//		xxxx++;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}


#else

/**********************************************************x平移抓弹（一键）***********************************/
int xxxx = 0;
void Motor_task(void *pvParameter)
{
	get_buttle_pid_Init();
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; 
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		if(rc_ctrl.rc .s1 == 1)
		{
			if(ele2_flag == 0)
			{
				get_buttle_pidNum_Init();
				gas_flag = 0;
			}	
			//x轴设定值的改变
			x_set_change();
			//爪子设定值变化
			claw_set_change();
			//送弹设定值变化
			send_set_change();
			//绝对变增量
			get_buttle_position();
			//x平移实际值限定
			x_limit();
			//送弹实际值限定
			send_limit();
			//爪子实际值限定
			claw_limit();
			//pid计算
			get_buttle_pid();
			 //误差判断
			if(x_p_pid.error[0] <= 3000 && x_p_pid.error[0] >= -3000)
			{
				if(mov_flag == 1)
				{
					mox_flag = 1;       //  x轴左平移完发送，执行爪子抓取
				}
				else if(mov_flag == 2)
				{
					mox_flag = 2;       //  x轴右平移完发送，执行爪子抓取
				}
				else if(mov_flag == 3)
				{
					mox_flag = 0;       //x轴归位后发送，复位
				}
			}
			if(Claw_p_pid.error[0] <= 300 && Claw_p_pid.error[0] >= -300)         
			{
				if(gas_flag == 1)
				{
					mot_flag = 1;
					i1_flag++;
					if(gas_flag == 1 && i1_flag >= 150)
					{
						mot_flag = 2;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
				}
				else if(gas_flag == 2)
				{
					mot_flag = 3;
					i1_flag++;
					if(gas_flag == 2 && i1_flag >= 350)
					{
						mot_flag = 4;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
//					printf("%d\r\n",i1_flag);
				}
				else if(gas_flag == 3)
				{
					i1_flag++;
					if(gas_flag == 3 && i1_flag>=1 && i1_flag<=10)
					{
						mot_flag = 5;
					}
					else if(gas_flag == 3 && i1_flag>=200)
					{
						mot_flag = 6;
						i1_flag = 0;
						i2_flag = 0;
					}
				}
				else if(gas_flag == 4)
				{
					mot_flag = 7;
					i1_flag++;
					if(gas_flag == 4 && i1_flag>=300)
					{
						mot_flag = 7;
	//					printf("%d\r\n",i1_flag);
						i1_flag = 0;
						i2_flag = 0;
					}
//					printf("%d\r\n",i1_flag);
				}
			}
			if(row_flag == 0 && i2_flag >= 100)   //  扔箱子检测
			{
				i2_flag = 0;
			}
			else if(row_flag == 1 && i2_flag >=13 && i2_flag <100)
			{
				mot_flag = 5;
				row_flag = 0;
			}
		}
		else
		{
			if(ele2_flag == 0)
			{
				get_buttle_pidNum_Init();
				gas_flag = 0;
				mot_flag = 0;
				mov_flag = 0;
			}
			//绝对变增量
			get_buttle_position();
			//x平移实际值限定
			x_limit();
			//送弹实际值限定
			send_limit();
			//爪子实际值限定
			claw_limit();
			//pid计算
			get_buttle_pid();
			gas_flag = 0;
			mot_flag = 0;
			mov_flag = 0;
		}
		ele2_flag = 0;
//		xxxx++;
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

#endif

/***************PID*****************
	函数名：get_buttle_pid_Init
	作  用：取弹pid初始化
******************************************/
void get_buttle_pid_Init()
{
	pid_MInit(&Claw_p_pid,3.8,0.03,80);  //爪子
	pid_MInit(&Claw_s_pid,1.25,0,0);
	pid_MInit(&Claw2_s_pid,1.25,0,0);   //一个位置环两个速度环输出
	pid_MInit(&x_p_pid,3.8,0.05,50);   //x横移
	pid_MInit(&x_s_pid,1.2,0,0);
	pid_MInit(&Send_p_pid,3.8,0.05,50);  //送弹
	pid_MInit(&Send_s_pid,1.2,0,0);
}
/***************PID*****************
	函数名：get_buttle_position
	作  用： 取弹电机绝对变增量
******************************************/
void get_buttle_position()  
{
	Increment_Angle(&x_p_pid,Motor_y[0].motor_angle);   
	Increment_Angle(&Claw_p_pid,Motor_y[2].motor_angle);
	Increment_Angle(&Send_p_pid,Motor_y[1].motor_angle);
}
/***************PID*****************
	函数名：get_buttle_pid
	作  用： 取弹电机pid计算
******************************************/
void get_buttle_pid()   //取弹电机pid计算
{
	pid_incomplete(&x_p_pid,x_p_pid.Act_num,x_p_pid.SetNum,3000,15000,13000);     //x平移位置环
	pid_incomplete(&x_s_pid,Motor_y[0].motor_speed,x_p_pid.output,1000,15000,15000);   //x平移速度环
	pid_incomplete(&Claw_p_pid,Claw_p_pid.Act_num,Claw_p_pid.SetNum,3000,15000,13000);   //爪子1位置环
	pid_incomplete(&Claw_s_pid,Motor_y[2].motor_speed,Claw_p_pid.output,1000,11000,16384);  //爪子1速度环
	pid_incomplete(&Claw2_s_pid,Motor_y[3].motor_speed,-Claw_p_pid.output,1000,11000,16384);   // 一个位置环输出作为两个速度环设定值
	pid_incomplete(&Send_p_pid,Send_p_pid.Act_num,Send_p_pid.SetNum,3000,15000,11000);     //送弹位置环
	pid_incomplete(&Send_s_pid,Motor_y[1].motor_speed,Send_p_pid.output,1000,15000,15000);   //送弹速度环
	Motor_201_204_Can2TX(x_s_pid.output,Send_s_pid.output,Claw_s_pid.output,Claw2_s_pid.output);   //输出发送
}
/***************PID*****************
	函数名：get_buttle_pidNum_Init
	作  用：取弹电机pid设定与实际值初始化
******************************************/
void get_buttle_pidNum_Init()   //取弹电机pid设定与实际值初始化
{
	x_p_pid.SetNum = 0;
	x_p_pid.ActualNum = 0;
	Send_p_pid.ActualNum = 0;
	Send_p_pid.SetNum = 0;
	Claw_p_pid.ActualNum = 0;
	Claw_p_pid.SetNum = 0;
	Claw2_p_pid.ActualNum = 0;
}

/**********************change***************
	函数名：x_set_change
	作用：x横移设定值的变化
**********************************************/
void x_set_change()
{
	if(mov_flag == 0)         // 0. 初始状态 0 
	{
		Claw_p_pid.SetNum = claw_set_res;
		x_p_pid.SetNum = x_set_res;
	}
	else if(mov_flag == 1)         //1. 设定值-880000    最大上限  1050000
	{
		x_p_pid.SetNum = x_set_left;
	}
	else if(mov_flag == 2)         // 2. 设定值880000   最大上限  890000
	{
		x_p_pid.SetNum = x_set_right;
	}
}
/**********************change***************
	函数名：claw_set_change
	作用：爪子设定值的变化
**********************************************/
void claw_set_change()
{
	if(gas_flag == 0 && mov_flag == 0)
	{
		Claw_p_pid.SetNum = claw_set_res;
		x_p_pid.SetNum = x_set_res;
	}
	else if(gas_flag == 1)
	{
		Claw_p_pid.SetNum = claw_set_graw;////////////////改//////////
	}
	else if(gas_flag == 2)
	{
		Claw_p_pid.SetNum = claw_set_res;
	}
	else if(gas_flag == 3)
	{
		Claw_p_pid.SetNum = claw_set_throw;
		row_flag = 1;
		i2_flag++;
	}
	//可修改
	else if(gas_flag == 4)
	{
		Claw_p_pid.SetNum = claw_set_back;
	}
	else if(gas_flag == 5)
	{
		Claw_p_pid.SetNum = claw_set_res;
	}
}
/**********************change***************
	函数名：send_set_change
	作用：送弹设定值的变化
**********************************************/
void send_set_change()
{
	if(mos_flag == 0)         // 0. 初始状态 0 
	{
		Send_p_pid.SetNum = send_set_close;
	}
	else if(mos_flag == 1)         //1. 设定值-330000
	{
		Send_p_pid.SetNum = send_set_open;
	}
}
/**********************limit***************
	函数名：x_limit
	作用：x横移最值限定
**********************************************/
void x_limit()
{
	if(x_p_pid.ActualNum <= x_limit_left && mov_flag == 1)
	{
		x_p_pid.Act_num = x_limit_left;
	}
	else if(x_p_pid.ActualNum >= x_limit_right && mov_flag == 2)
	{
		x_p_pid.Act_num = x_limit_right;
	}
	else if(x_p_pid.ActualNum <= x_limit_res && mov_flag == 3)
	{
		x_p_pid.Act_num = x_limit_res;
	}
	else
	{
		x_p_pid.Act_num = x_p_pid.ActualNum;
	}
}
/**********************limit***************
	函数名：claw_limit
	作用：爪子最值限定
**********************************************/
void claw_limit()
{
	if(Claw_p_pid.ActualNum >= claw_limit_graw && gas_flag == 1)////////////////改//////////
	{
		Claw_p_pid.Act_num = claw_limit_graw;/////////////////改////////////////
	}
	else if(Claw_p_pid.ActualNum <= claw_limit_res && gas_flag == 2)
	{
		Claw_p_pid.Act_num = claw_limit_res;
	}
	else if(Claw_p_pid.ActualNum >= claw_limit_throw && gas_flag == 3)
	{
		Claw_p_pid.Act_num = claw_limit_throw;
	}
	else if(Claw_p_pid.ActualNum <= claw_limit_back && gas_flag == 4)
	{
		Claw_p_pid.Act_num = claw_limit_back;
	}
	else if(Claw_p_pid.ActualNum <= claw_limit_res && gas_flag == 5)
	{
		Claw_p_pid.Act_num = claw_limit_res;
	}
	else
	{
		Claw_p_pid.Act_num = Claw_p_pid.ActualNum;
	}
}
/**********************limit***************
	函数名：send_limit
	作用：送弹最值限定
**********************************************/
void send_limit()
{
	if(Send_p_pid.ActualNum <= send_open && mos_flag == 1)
	{
		Send_p_pid.Act_num = send_open;
	}
	else if(Send_p_pid.ActualNum >= send_close && mos_flag == 0)
	{
		Send_p_pid.Act_num = send_close;
	}
	else
	{
		Send_p_pid.Act_num = Send_p_pid.ActualNum;
	}
}

///***************PID*****************
//	函数名：pid_ActualNum_Limit
//	作  用：取弹电机实际值的限定
//	参	数：pid指针
//			Limit_Num 限定值
//			flag_Num 标志位取值
//			int8_t trend 变化趋势（1为增限制最大值）（2为减限制最小值）
//**********************************************************************/
//void pid_ActualNum_Limit(PIDxtype *p_Pid,float Limit_Num,int8_t flag_Num,int8_t trend)
//{
//	if(p_Pid->ActualNum <= -900000 && mov_flag == flag_Num)
//	{
//		x_p_pid.Act_num = -900000;
//	}
//	else
//	{
//		x_p_pid.Act_num = x_p_pid.ActualNum;
//	}
//}


