#include "gas.h"
#include "can_task.h"
#include "pid.h"
#include "motor.h"
#include "chassis.h"
#include "vl53l0x.h"
#include "VL53L0X_task.h"
#include "vl53l0x_gen.h"
#include "vl53l0x_gen_2.h"
#include "usart6.h"
	/*FreeRTOS系统*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "semphr.h" 
//int x = 0,y = 0;//（测试小能手）
int te1,te2,te3,te4;  //测试时使用
int8_t gas_flag = 0; // 爪子转动标志位
int8_t mov_flag = 0; // x轴平移标志位
int8_t Mes_flag = 0; //接收数据标志位（测试使用）
int8_t end_flag = 0; // 动作结束标志位
int8_t res_flag = 0; // 复位标志位
int8_t mos_flag = 0; // 送弹标志位
void Gas_task(void *pvParameter)    //气缸及电机标志位转化
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; 
	xLastWakeTime = xTaskGetTickCount();
	while(1)
	{
		#if G_MODE == 1
/*********************************************普通抓弹(按键测试)*******************************/
		if(rc_ctrl.rc .s1 == 1)     
		{
			if(rc_ctrl.rc .s2 == 1)
			{
				
				gas_flag = 1;
				if(mot_flag == 1)
				{
					MINGLE_ON();
				}
				
			}
			else if(rc_ctrl.rc .s2 == 3)
			{
				gas_flag = 2;
				UP_DOWN_ON();
			}
			else if(rc_ctrl.rc .s2 == 2)
			{
				gas_flag = 3;
				if(mot_flag == 5)
				{
					MINGLE_OFF();
				}
			}
		}
//		Data_sends(Motor_y[2].motor_speed,Motor_y[3].motor_speed,0,0,0,0);

		#elif G_MODE == 2

/************************************************普通抓弹(一键)****************************************/
		if(rc_ctrl.rc .s1 == 1)
		{
			if(rc_ctrl.rc .s2 == 1)
			{
				if(mot_flag == 0)
				{
					UP_DOWN_ON();
					gas_flag = 1;
				}
				if(mot_flag == 1)
				{
					MINGLE_ON();
				}
				else if(mot_flag == 2)
				{
					gas_flag = 2;
				}
				else if(mot_flag == 3)
				{
					gas_flag = 2;
				}
				else if(mot_flag == 4)
				{
					MINGLE_ON();
					gas_flag = 3;
					
				}
				else if(mot_flag == 5)
				{
					MINGLE_OFF();
				}
				else if(mot_flag == 6)
				{
					gas_flag = 4;
				}
				else if(mot_flag == 7)
				{
					gas_flag = 5;
				}
			}
			else
			{
				UP_DOWN_ON();
				gas_flag = 0;
				mot_flag = 0;
			}
		}
		else
		{
			MINGLE_OFF();
			gas_flag = 0;
			mot_flag = 0;
		}

//		printf("%f   %f  %f \r\n",Claw_p_pid.ActualNum,Claw_p_pid.SetNum ,Pit);
//		Data_sends(Claw_s_pid.ActualNum,Claw_s_pid.SetNum,0,0,0,0);	

		#elif G_MODE == 3 
///************************************************十字抓弹(low底盘移动+一键抓弹(按键))**************************************/
		if(rc_ctrl.rc .s1 == 1)
		{
			UP_DOWN_ON();
			if(rc_ctrl.rc .s2 == 3)
			{
				if(mot_flag == 0)
				{
					gas_flag = 1;
				}
				if(mot_flag == 1)
				{
					MINGLE_ON();
					gas_flag = 1;
					if(lock_flag >= 800)
					{
						MINGLE_OFF();
					}
				}
				else if(mot_flag == 2)
				{
					gas_flag = 2;
					if(lock_flag >= 800)
					{
						MINGLE_OFF();
					}
				}
				else if(mot_flag == 3)
				{
					gas_flag = 2;
					if(lock_flag >= 800)
					{
						MINGLE_OFF();
					}
				}
				else if(mot_flag == 4)
				{
					gas_flag = 3;
				}
				else if(mot_flag == 5)
				{
					MINGLE_OFF();
					gas_flag = 3;
				}
				else if(mot_flag == 6)
				{
					gas_flag = 4;
				}
				else if(mot_flag == 7)
				{
					gas_flag = 5;
				}
			}
			else if(rc_ctrl.rc .s2 == 1)
			{
				Y_ON();
				gas_flag = 0;
				mot_flag = 0;
			}
			else
			{
				Y_OFF();
				UP_DOWN_ON();
				gas_flag = 0;
				mot_flag = 0;
			}
		}
		else if(rc_ctrl.rc .s1 == 2)//送弹
		{
			mos_flag = 1;
		}
		else
		{
			if(rc_ctrl.rc .s2 == 3)
			{
				HELP_ON();
			}
			else if(rc_ctrl.rc .s2 == 2)
			{
				HELP_OFF();
			}
			Y_OFF();
			MINGLE_OFF();
			gas_flag = 0;
			mot_flag = 0;
			mos_flag = 0;
		}
//		printf("%f   %f  %f \r\n",Claw_p_pid.ActualNum,Claw_p_pid.SetNum ,Pit);
//		Data_sends(Claw_s_pid.ActualNum,Claw_s_pid.SetNum,0,0,0,0);
//		printf("1");
		
		
		#elif G_MODE == 4
/**************************************************x平移抓弹（全按键）*********************************/
		if(rc_ctrl.rc .s1 == 1)
		{
//			if(rc_ctrl.rc.s2 == 1)
//			{
			if(Mes_flag != rc_ctrl.rc.s2)
			{
				res_flag = 0;
			}
			if(mot_flag == 0)
			{
				if(rc_ctrl.rc.s2 == 1 && res_flag == 0)
				{
					gas_flag = 1;
					res_flag = 1;
				}
				else if(rc_ctrl.rc.s2 == 3 && res_flag == 0)//执行爪子转动程序
				{
					gas_flag = 1;
					res_flag = 1;
				}
				else if(rc_ctrl.rc.s2 == 2 && res_flag == 0)
				{
					gas_flag = 1;
					res_flag = 1;
				}
			}
			else if(mot_flag == 1) //闭合爪子
			{
				MINGLE_ON();
			}
			else if(mot_flag == 2) //收回爪子 
			{
				gas_flag = 2;
			}
			else if(mot_flag == 3)    //开始漏弹延时
			{
				gas_flag = 2;
				if(mov_flag == 1 && mox_flag == 1)  //爪子已取弹，判断开始第二次平移
				{
					mov_flag = 2;
//						res_flag = 1;
				}
				else if(mox_flag == 2 && mov_flag == 2) //判断开始第三次平移
				{
					mov_flag = 0;
//						res_flag = 1;
				}
			}
			else if(mot_flag == 4)  //漏弹完毕 准备弹箱子
			{
				gas_flag = 3;
			}
			else if(mot_flag == 5)  //爪子未达到扔弹位置，中途夹子开
			{
				MINGLE_OFF();
			}
			else if(mot_flag == 6)  //爪子到达扔弹位置 ，准备归位
			{					
				if(res_flag == 1 && mov_flag <= 2) //复位检测
				{
					gas_flag = 0;
					mot_flag = 0;
					res_flag = 2;
				}
				if(mov_flag == 0)
				{
					gas_flag = 4;
				}
			}
			else if(mot_flag == 7)  // 爪子归位，消除位置差
			{
				if(mov_flag == 0)  //第三次平移完成后归位检测
				{
					gas_flag = 5;
					mov_flag = 0;
					end_flag = 1;
					if(end_flag == 1)   //抓弹结束 全部清零
					{
						gas_flag = 0;
						mot_flag = 0;
						mox_flag = 0;
						res_flag = 2;
						cmv_flag = 0;
					}
				}
			}
			Mes_flag = rc_ctrl.rc.s2;
		} 
		else if(rc_ctrl.rc .s1 == 2)
		{
			UP_DOWN_ON();
			MINGLE_OFF();
			gas_flag = 0;
			mot_flag = 0;
			mov_flag = 0;
			mox_flag = 0;
		}
		else if(rc_ctrl.rc .s1 == 3)
		{
			mov_flag = 1;
		}
//		printf("%d %f \r\n",Motor_y[0].motor_angle,x_p_pid.ActualNum);
//		Data_sends(x_p_pid.SetNum,x_p_pid.ActualNum,0,0,0,0);
		
//		Motor_201_204_Can2TX(15000,0,0,0);
//		printf("%d \r\n",Motor_y[0].motor_angle);
//		printf("%f   %f \r\n",x_p_pid.SetNum,Claw_p_pid.SetNum);   //打印数据

		#else
/**************************************************x平移抓弹（一键）**********************************/
		if(rc_ctrl.rc .s1 == 1)
		{
			if(rc_ctrl.rc.s2 == 1)
			{
				if(mot_flag == 0)
				{
					//判断x平移的方向
					if(end_flag == 0 && mov_flag == 0)
					{
						mov_flag = 1; //开始第一次平移
					}
					if(mov_flag == 1 && mox_flag == 1)//执行爪子转动程序
					{
						gas_flag = 1;
					}
					else if(mox_flag == 2 && mov_flag == 2)
					{
						gas_flag = 1;
					}
				}
				else if(mot_flag == 1) //闭合爪子
				{
					MINGLE_ON();
				}
				else if(mot_flag == 2) //收回爪子 
				{
					gas_flag = 2;
				}
				else if(mot_flag == 3)    //开始漏弹延时
				{
					gas_flag = 2;
					if(mov_flag == 1 && mox_flag == 1)  //爪子已取弹，判断开始第二次平移
					{
						mov_flag = 2;
						res_flag = 1;
					}
					else if(mox_flag == 2 && mov_flag == 2) //判断开始第三次平移
					{
						mov_flag = 0;
						res_flag = 1;
					}
				}
				else if(mot_flag == 4)  //漏弹完毕 准备弹箱子
				{
					gas_flag = 3;
				}
				else if(mot_flag == 5)  //爪子未达到扔弹位置，中途夹子开
				{
					MINGLE_OFF();
				}
				else if(mot_flag == 6)  //爪子到达扔弹位置 ，准备归位
				{
					if(res_flag == 1 && mov_flag <= 2) //复位检测
					{
						gas_flag = 0;
						mot_flag = 0;
						res_flag = 0;
					}
					if(mov_flag == 0)
					{
						gas_flag = 4;
					}
				}
				else if(mot_flag == 7)  // 爪子归位，消除位置差
				{
					if(mov_flag == 0)  //第三次平移完成后归位检测
					{
						gas_flag = 5;
						mov_flag = 0;
						end_flag = 1;
						if(end_flag == 1)   //抓弹结束 全部清零
						{
							gas_flag = 0;
							mot_flag = 0;
							mox_flag = 0;
							res_flag = 0;
						}
					}
				}
			}
			else
			{
				UP_DOWN_ON();
				gas_flag = 0;
				mot_flag = 0;
				mov_flag = 0;
				mox_flag = 0;
				end_flag = 0;
				res_flag = 0;
			}
		} 
		else
		{
			MINGLE_OFF();
			gas_flag = 0;
			mot_flag = 0;
			mov_flag = 0;
			mox_flag = 0;
		}
//		printf("%f   %f \r\n",x_p_pid.SetNum,Claw_p_pid.SetNum);   //打印数据找问题
		
		#endif
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}
		
		
void Printf_task(void *pvParameter)
{
	while(1)	
	{
//		pow(3,3);
//		Data_sends(Motor_y[0].motor_speed,0,0,0,0,0);
//		printf("%f   %f\r\n",x_s_pid.output,x2_s_pid.output);
//		Data_sends(x_p_pid.SetNum,x_p_pid.ActualNum,0,0,0,0);
//		printf("%d %f \r\n",Motor_y[0].motor_angle,x_p_pid.ActualNum);
//		printf("%d %d %d %d   \r\n",rc_ctrl.rc.ch0,rc_ctrl.rc.ch1,rc_ctrl.rc.ch2,rc_ctrl.rc.ch3);
//		GPIO_SetBits(GPIOA, GPIO_Pin_1);
//		printf("%d",GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1));
//		Motor_201_204_Can1TX(1000,0,0,0);
		
		if(laster_range.laster_range_date >=35.0f && laster_range.laster_range_date <=50.0f)
		{
			GPIO_SetBits(GPIOE, GPIO_Pin_10);
		}
		else
		{
			GPIO_ResetBits(GPIOE, GPIO_Pin_10);
		}
//		printf("	d11111111111111111111111 %8.1fcm		\r\n",vl5310x.distance);//打印测量距离
//		vl53l0x_test();
		vTaskDelay(5);
	}
		
}


