#include "motor.h"
#include "pid.h"
#include "can1.h"
#include "can_task.h"
#include "gas.h"

	/*FreeRTOSϵͳ*/
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  
#include "task.h"
#include "semphr.h" 
Motor_Message Motor_x[8];
Motor_Message Motor_y[8];

/***************DATA*****************
	��������angle_first
	��  �ã� ����λ�û���ʼֵ
	��	���� 
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

int8_t mot_flag = 0;  //צ�ӵ�������־λ
//int16_t turn_flag = 0;
int8_t mox_flag = 0; //xƽ�Ƶ��λ�õ����־λ
int16_t i1_flag = 0,i2_flag = 0,i3_flag = 0,i4_flag = 0;//����ר��(i1_flag����ִ�д�����־λ��i2_flag ������ר�ü��ִ�д�����־λ)
int8_t row_flag = 0;  //�����ӱ�׼λ
int16_t lock_flag = 0; //��ת���

#if M_MODE == 1
///***************************************************��ͨץ���������************************************/
void Motor_task(void *pvParameter)    //ץ��������Ƶ���
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
			//x���趨ֵ�ĸı�
			x_set_change();
			//�͵��趨ֵ�仯
			send_set_change();
			//צ���趨ֵ�ı�
			claw_set_change();
			//  ���Ա�����
			get_buttle_position();
			//xƽ��ʵ��ֵ�޶�
			x_limit();
			//�͵�ʵ��ֵ�޶�
			send_limit();
			//צ��ʵ��ֵ�޶�
			claw_limit();
			//pid����
			get_buttle_pid();
			 //����ж�
			if(x_p_pid.error[0] <= 3000 && x_p_pid.error[0] >= -3000)
			{
				if(mov_flag == 1)
				{
					mox_flag = 1;       //  x����ƽ���귢�ͣ�ִ��צ��ץȡ
				}
				else if(mov_flag == 2)
				{
					mox_flag = 2;       //  x����ƽ���귢�ͣ�ִ��צ��ץȡ
				}
				else if(mov_flag == 3)
				{
					mox_flag = 0;       //x���λ���ͣ���λ
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
			if(row_flag == 0 && i2_flag >= 100)   //  �����Ӽ��
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
			//���Ա�����
			get_buttle_position();
			// x����ʵ��ֵ�޶�
			x_limit();
			//�͵�ʵ��ֵ�޶�
			send_limit();
			//צ��ʵ��ֵ�޶�
			claw_limit();
			//pid����
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

/**********************************************************xƽ��ץ��(ȫ����)************************************/
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
			//x���趨ֵ�ĸı�
			x_set_change();
			//צ���趨ֵ�ĸı�
			claw_set_change();
			//�͵��趨ֵ�仯
			send_set_change();
			//���Ա�����
			get_buttle_position();
			//xƽ��ʵ��ֵ�޶�
			x_limit();
			//�͵�ʵ��ֵ�޶�
			send_limit();
			//צ��ʵ��ֵ�޶�
			claw_limit();
			//pid����
			get_buttle_pid();
			 //����ж�
			if(x_p_pid.error[0] <= 3000 && x_p_pid.error[0] >= -3000)
			{
				if(mov_flag == 1)
				{
					mox_flag = 1;       //  x����ƽ���귢�ͣ�ִ��צ��ץȡ
				}
				else if(mov_flag == 2)
				{
					mox_flag = 2;       //  x����ƽ���귢�ͣ�ִ��צ��ץȡ
				}
				else if(mov_flag == 3)
				{
					mox_flag = 0;       //x���λ���ͣ���λ
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
			if(row_flag == 0 && i2_flag >= 100)   //  �����Ӽ��
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
			//���Ա�����
			get_buttle_position();
			x_limit();
			//�͵�ʵ��ֵ�޶�
			send_limit();
			//צ��ʵ��ֵ�޶�
			claw_limit();
			//pid����
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

/**********************************************************xƽ��ץ����һ����***********************************/
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
			//x���趨ֵ�ĸı�
			x_set_change();
			//צ���趨ֵ�仯
			claw_set_change();
			//�͵��趨ֵ�仯
			send_set_change();
			//���Ա�����
			get_buttle_position();
			//xƽ��ʵ��ֵ�޶�
			x_limit();
			//�͵�ʵ��ֵ�޶�
			send_limit();
			//צ��ʵ��ֵ�޶�
			claw_limit();
			//pid����
			get_buttle_pid();
			 //����ж�
			if(x_p_pid.error[0] <= 3000 && x_p_pid.error[0] >= -3000)
			{
				if(mov_flag == 1)
				{
					mox_flag = 1;       //  x����ƽ���귢�ͣ�ִ��צ��ץȡ
				}
				else if(mov_flag == 2)
				{
					mox_flag = 2;       //  x����ƽ���귢�ͣ�ִ��צ��ץȡ
				}
				else if(mov_flag == 3)
				{
					mox_flag = 0;       //x���λ���ͣ���λ
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
			if(row_flag == 0 && i2_flag >= 100)   //  �����Ӽ��
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
			//���Ա�����
			get_buttle_position();
			//xƽ��ʵ��ֵ�޶�
			x_limit();
			//�͵�ʵ��ֵ�޶�
			send_limit();
			//צ��ʵ��ֵ�޶�
			claw_limit();
			//pid����
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
	��������get_buttle_pid_Init
	��  �ã�ȡ��pid��ʼ��
******************************************/
void get_buttle_pid_Init()
{
	pid_MInit(&Claw_p_pid,3.8,0.03,80);  //צ��
	pid_MInit(&Claw_s_pid,1.25,0,0);
	pid_MInit(&Claw2_s_pid,1.25,0,0);   //һ��λ�û������ٶȻ����
	pid_MInit(&x_p_pid,3.8,0.05,50);   //x����
	pid_MInit(&x_s_pid,1.2,0,0);
	pid_MInit(&Send_p_pid,3.8,0.05,50);  //�͵�
	pid_MInit(&Send_s_pid,1.2,0,0);
}
/***************PID*****************
	��������get_buttle_position
	��  �ã� ȡ��������Ա�����
******************************************/
void get_buttle_position()  
{
	Increment_Angle(&x_p_pid,Motor_y[0].motor_angle);   
	Increment_Angle(&Claw_p_pid,Motor_y[2].motor_angle);
	Increment_Angle(&Send_p_pid,Motor_y[1].motor_angle);
}
/***************PID*****************
	��������get_buttle_pid
	��  �ã� ȡ�����pid����
******************************************/
void get_buttle_pid()   //ȡ�����pid����
{
	pid_incomplete(&x_p_pid,x_p_pid.Act_num,x_p_pid.SetNum,3000,15000,13000);     //xƽ��λ�û�
	pid_incomplete(&x_s_pid,Motor_y[0].motor_speed,x_p_pid.output,1000,15000,15000);   //xƽ���ٶȻ�
	pid_incomplete(&Claw_p_pid,Claw_p_pid.Act_num,Claw_p_pid.SetNum,3000,15000,13000);   //צ��1λ�û�
	pid_incomplete(&Claw_s_pid,Motor_y[2].motor_speed,Claw_p_pid.output,1000,11000,16384);  //צ��1�ٶȻ�
	pid_incomplete(&Claw2_s_pid,Motor_y[3].motor_speed,-Claw_p_pid.output,1000,11000,16384);   // һ��λ�û������Ϊ�����ٶȻ��趨ֵ
	pid_incomplete(&Send_p_pid,Send_p_pid.Act_num,Send_p_pid.SetNum,3000,15000,11000);     //�͵�λ�û�
	pid_incomplete(&Send_s_pid,Motor_y[1].motor_speed,Send_p_pid.output,1000,15000,15000);   //�͵��ٶȻ�
	Motor_201_204_Can2TX(x_s_pid.output,Send_s_pid.output,Claw_s_pid.output,Claw2_s_pid.output);   //�������
}
/***************PID*****************
	��������get_buttle_pidNum_Init
	��  �ã�ȡ�����pid�趨��ʵ��ֵ��ʼ��
******************************************/
void get_buttle_pidNum_Init()   //ȡ�����pid�趨��ʵ��ֵ��ʼ��
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
	��������x_set_change
	���ã�x�����趨ֵ�ı仯
**********************************************/
void x_set_change()
{
	if(mov_flag == 0)         // 0. ��ʼ״̬ 0 
	{
		Claw_p_pid.SetNum = claw_set_res;
		x_p_pid.SetNum = x_set_res;
	}
	else if(mov_flag == 1)         //1. �趨ֵ-880000    �������  1050000
	{
		x_p_pid.SetNum = x_set_left;
	}
	else if(mov_flag == 2)         // 2. �趨ֵ880000   �������  890000
	{
		x_p_pid.SetNum = x_set_right;
	}
}
/**********************change***************
	��������claw_set_change
	���ã�צ���趨ֵ�ı仯
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
		Claw_p_pid.SetNum = claw_set_graw;////////////////��//////////
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
	//���޸�
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
	��������send_set_change
	���ã��͵��趨ֵ�ı仯
**********************************************/
void send_set_change()
{
	if(mos_flag == 0)         // 0. ��ʼ״̬ 0 
	{
		Send_p_pid.SetNum = send_set_close;
	}
	else if(mos_flag == 1)         //1. �趨ֵ-330000
	{
		Send_p_pid.SetNum = send_set_open;
	}
}
/**********************limit***************
	��������x_limit
	���ã�x������ֵ�޶�
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
	��������claw_limit
	���ã�צ����ֵ�޶�
**********************************************/
void claw_limit()
{
	if(Claw_p_pid.ActualNum >= claw_limit_graw && gas_flag == 1)////////////////��//////////
	{
		Claw_p_pid.Act_num = claw_limit_graw;/////////////////��////////////////
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
	��������send_limit
	���ã��͵���ֵ�޶�
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
//	��������pid_ActualNum_Limit
//	��  �ã�ȡ�����ʵ��ֵ���޶�
//	��	����pidָ��
//			Limit_Num �޶�ֵ
//			flag_Num ��־λȡֵ
//			int8_t trend �仯���ƣ�1Ϊ���������ֵ����2Ϊ��������Сֵ��
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


