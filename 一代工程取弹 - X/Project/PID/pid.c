#include "pid.h"
#include "main.h"
#include "chassis.h"
#include "motor.h"
#include "can_task.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

	
PIDxtype Claw_p_pid;  //צ��λ�û�pid
PIDxtype Claw_s_pid;  //צ���ٶȻ�pid
PIDxtype Claw2_p_pid; // צ��2λ�û������ã�
PIDxtype Claw2_s_pid; //צ��2�ٶȻ�

PIDxtype x_p_pid;  //x����λ�û�
PIDxtype x_s_pid;  //x�����ٶȻ�
//PIDxtype x2_p_pid; // x2
//PIDxtype x2_s_pid;
	
PIDxtype Send_p_pid;  //�͵�pidλ�û�
PIDxtype Send_s_pid;  //�͵�pid�ٶȻ�
/***************λ��ʽPID*****************
	��������PID_MInit 
	��  �ã� PID��ʼ��
	��	��������ָ��
			Kp   Ki    Kd
******************************************/
void pid_MInit(PIDxtype *pid, float Kp , float Ki , float Kd)    //��дpid��ʼ��
{
	pid ->ActualNum = 0.0f;
	pid ->error[0] = pid ->error[1] = pid ->error[2] = 0.0f;
	pid ->output = 0.0f;
	pid ->SetNum = 0.0f;
	pid->integral = 0;
	pid->differential[0] = pid->differential[1] = pid->differential[2] = 0.0f;
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->P_out = 0;
	pid->I_out = 0;
	pid->D_out = 0;
}

/***************λ��ʽPID*****************
	��������PID_GInit 
	��  �ã� PID��ʼ��
	��	��������ָ��
			Kp   Ti   Td
******************************************/
void pid_GInit(PIDxtype *pid, float Kp , float Ti , float Td)    //��дpid��ʼ��
{
	pid ->ActualNum = 0.0f;
	pid ->error[0] = pid ->error[1] = pid ->error[2] = 0.0f;
	pid ->output = 0.0f;
	pid ->SetNum = 0.0f;
	pid->integral = 0;
	pid->differential[0] = pid->differential[1] = pid->differential[2] = 0.0f;
	pid->Ti = Ti;
	pid->Td = Td;
	pid->Kp = Kp;
	pid->P_out = 0;
	pid->I_out = 0;
	pid->D_out = 0;
	pid->angle_now = 0;
	pid->angle_last = 0;
}
/***************DATA*****************
	��������Increment_Angle
	��  �ã� ����λ�û�
	��	���� 
******************************************/
//int16_t Mes[8];
int8_t Add_flag = 0;
void Increment_Angle(PIDxtype *PId_p,int16_t an_now)
{
	PId_p->angle_now = an_now;
	PId_p->err = PId_p->angle_now - PId_p->angle_last;
	if(PId_p->err > -3000 && PId_p->err < 3000)
	{
		PId_p->err = PId_p->err;
	}
	else if(PId_p->err <= -3000)
	{
		PId_p->err = PId_p->err + 8192;
	}
	else if(PId_p->err >= 3000)
	{
		PId_p->err = PId_p->err - 8192;
	}
	PId_p->ActualNum = PId_p->ActualNum + PId_p->err;
	PId_p->angle_last = PId_p->angle_now;
}
/***************PID*****************
	��������PID_ Posit
	��  �ã� λ����PID����
	��	��������ָ��
			Kp Ki Kd ������
			P_out ,I_out , D_out �����
******************************************/
//PIDxtype motor_pid_p;
uint8_t X_flag = 0;
float Bit = 1;
void pid_Posit(PIDxtype *PId_p,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout)   //λ����PID
{
	if(PId_p->Ti == 0)
	{
		PId_p->Ki = 0;
	}
	else
	{
		PId_p->Ki = PId_p->Kp/PId_p->Ti;
	}
	PId_p->Kd = PId_p->Kp*PId_p->Td;
	PId_p->error[2] = PId_p->error[1];
    PId_p->error[1] = PId_p->error[0];
	PId_p->ActualNum = actualnum ;
	PId_p->SetNum = setnum;
	PId_p->error[0] = (PId_p->SetNum) - (PId_p->ActualNum);
	if(ele1_flag == 0 || ele2_flag == 0)      //�Ƿ��ϵ�
	{
		X_flag = 0;
	}
	if(X_flag == 0)       //��������  �ж��Ƿ��ۼ�
	{
		PId_p->integral = 0;
		if(fabs(PId_p->error[0]) <= err_limit)
		{
			X_flag = 1;
		}
	}
	if(X_flag !=0)        //�����ۼ�
	{
		Bit = (err_limit - fabs(PId_p->error[0]))/err_limit;
		if(Bit <= 0)
		{
			Bit = 1;
		}
		PId_p->integral +=((PId_p->error[0] + PId_p->error[1]) / 2);
	}
	LimitMax(PId_p->integral, imaxout);   //15000
	PId_p->differential[2] =  PId_p->differential[1];       //΢��ƽ��
	PId_p->differential[1] =  PId_p->differential[0];
	PId_p->differential[0] = PId_p->error[0] - PId_p->error[1];
	PId_p->P_out = PId_p->Kp * (PId_p->error[0]);
	PId_p->I_out = Bit*PId_p->Ki * (PId_p->integral);
	PId_p->D_out = PId_p->Kd * (PId_p->differential[0]/3 + PId_p->differential[1]/3 + PId_p->differential[2]/3);
	PId_p->output = PId_p->P_out + PId_p->I_out + PId_p->D_out;
	LimitMax(PId_p->output,maxout);      //16384  30000   5000   
	PId_p->output_last = PId_p->output ;
//	ele1_flag = 0;
}
/***************PID*****************
	��������PID_ increment
	��  �ã� ������PID����
	��	��������ָ��
			Kp Ki Kd ������
			P_out ,I_out , D_out �����
******************************************/
//PIDxtype motor_pid_i;
void pid_Increment(PIDxtype *PId_i,float actualnum ,int16_t setnum,int16_t maxout)   //����PID
{
	PId_i->error[2] = PId_i->error[1];
	PId_i->error[1] = PId_i->error[0];
	PId_i->ActualNum = actualnum;
	PId_i->SetNum = setnum;
	PId_i->error[0] = (PId_i->SetNum) - (PId_i->ActualNum);
	PId_i->P_out = PId_i->Kp * (PId_i->error[0] - PId_i->error[1]);
	PId_i->I_out = PId_i->Ki * PId_i->error[0];
	PId_i->differential[2] = PId_i->differential[1];
	PId_i->differential[1] =PId_i->differential[0];
	PId_i->differential[0] = (PId_i->error[0] - 2.0f * PId_i->error[1] + PId_i->error[2]);
	PId_i->D_out = PId_i->Kd * (PId_i->differential[0] - 2*PId_i->differential[1] + PId_i->differential[2]);
	PId_i->output += PId_i->P_out + PId_i->I_out + PId_i->D_out;
	LimitMax(PId_i->output,maxout);      
}
/***************PID*****************
	��������PID_ incomplete
	��  �ã� ����ȫ΢��PID����
	��	��������ָ��
			Kp Ki Kd ������
			P_out ,I_out , D_out �����
******************************************/
uint8_t Y_flag = 0;
float Pit = 0;
void pid_incomplete(PIDxtype *PId_in,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout)   //����ȫPID
{
	PId_in->error[2] = PId_in->error[1];     //���ƽ��
    PId_in->error[1] = PId_in->error[0];
	PId_in->ActualNum = actualnum;
	PId_in->SetNum = setnum;
	PId_in->error[0] = (PId_in->SetNum) - (PId_in->ActualNum);
	if(ele1_flag == 0 && ele2_flag == 0)      //�Ƿ��ϵ�
	{
		Y_flag = 0;
	}
	if(Y_flag == 0)       //��������  �ж��Ƿ��ۼ�
	{
		PId_in->integral = 0;
		if(fabs(PId_in->error[0]) <= err_limit)
		{
			Y_flag = 1;
		}
	}
	if(Y_flag !=0)        //�����ۼ�
	{
//		Pit = (err_limit - fabs(PId_in->error[0]))/err_limit;
//		if(Pit <= 0)
//		{
//			Pit = 1;
//		}
		PId_in->integral +=((PId_in->error[0] + PId_in->error[1]) / 2);
	}
	LimitMax(PId_in->integral, imaxout);
	PId_in->differential[2] =  PId_in->differential[1];       //΢��ƽ��
	PId_in->differential[1] =  PId_in->differential[0];
	PId_in->differential[0] = PId_in->error[0] - PId_in->error[1];
	PId_in->P_out = PId_in->Kp * (PId_in->error[0]);
	PId_in->I_out = PId_in->Ki * (PId_in->integral);
	PId_in->D_out = PId_in->Kd * (PId_in->differential[0]/3 + PId_in->differential[1]/3 + PId_in->differential[2]/3);
	PId_in->output = PId_in->P_out + PId_in->I_out + PId_in->D_out ;
	LimitMax(PId_in->output,maxout);      
//	PId_i->err_last = PId_i->err; 
}

void Fuzzy_calculate(float err,float ec_err)
{
	
}
/***************DATA*****************
	��������Data_sends
	��  �ã� PID���ݷ��Ͳ���
	��	���� data1 2 3 4 5  ����
******************************************/


void Data_sends(int16_t data1,int16_t data2,int16_t data3,int16_t data4,int16_t data5,int16_t data6)
{
	uint8_t _cnt = 0;
	int8_t i = 0;
	uint8_t sum = 0;
    uint8_t testdataosend[20] = {0};
	testdataosend[_cnt++] = 0xAA;
	testdataosend[_cnt++] = 0x30;
	testdataosend[_cnt++] = 0xAF;
	testdataosend[_cnt++] = 0x02;

	testdataosend[_cnt++] = 0;
	testdataosend[_cnt++] = data1 >> 8;
	testdataosend[_cnt++] = data1;

	testdataosend[_cnt++] = data2 >> 8;
	testdataosend[_cnt++] = data2;

	testdataosend[_cnt++] = data3 >> 8;
	testdataosend[_cnt++] = data3;

	testdataosend[_cnt++] = data4 >> 8;
	testdataosend[_cnt++] = data4;

	testdataosend[_cnt++] = data5 >> 8;
	testdataosend[_cnt++] = data5;

	testdataosend[_cnt++] = data6 >> 8;
	testdataosend[_cnt++] = data6;	
	testdataosend[4] = _cnt - 5;
	for ( i = 0; i < _cnt; i++)
	{
			sum += testdataosend[i];	
	}
	testdataosend[_cnt++] = sum;
	for (uint8_t k = 0; k < _cnt; k++)
	{
			while ((USART1->SR&0x40) == 0);
			USART1->DR= testdataosend[k];	
	}
}

/**
  * @brief  ���ٶȴ�������
  * @param  setspeed	 --Ŀ���ٶ�
			speed_transit--�ٶȻ����趨�ٶ�
			acc			 --���ٶ�			dec--���ٶ�
  * @retval void
  * @note
**/	
void speed_accelerate(float setspeed,float *speed_transit,int16_t acc,int16_t  dec)
{

	if ((*speed_transit > 0) || (*speed_transit == 0 && setspeed >= 0))
	{
		if ((setspeed - *speed_transit > acc) && (setspeed > *speed_transit))	//Ŀ���ٶȴ��ڵ�ǰ�ٶ� ����
		{
			*speed_transit	+= acc;
		}
		else if (( *speed_transit - setspeed > dec) && (setspeed < *speed_transit))//Ŀ���ٶ�С�ڵ�ǰ�ٶ� ����
		{
			*speed_transit	-= dec;
		}
		else
		{
			*speed_transit = setspeed;
		}	
		
		if (*speed_transit < 0) 
		{
			*speed_transit = 0;
		}
	}
	else
	{
		if (( *speed_transit - setspeed > acc) && (setspeed < *speed_transit))
		{
			*speed_transit	-= acc;
		}
		else if ((setspeed - *speed_transit > dec) && (setspeed > *speed_transit))	
		{
			*speed_transit	+= dec;
		}
		else
		{
			*speed_transit	 = setspeed;
		}	
		
		if (*speed_transit > 0) 
		{
			*speed_transit = 0;
		}
	}
}
/***************DATA*****************
	��������acclerate
	��  �ã� ���ٶ�
	��	���� float setnum    �趨�ٶ�
			float *actnum    ʵ���ٶȣ�ָ�룩
			int16_t acc        ���ٶ�
			int16_t dec        ���ٶ�
******************************************/
void acclerate(float setnum,float *actnum,int16_t acc,int16_t dec)
{
	if(*actnum >= 0)
	{
		if(setnum >= 0)     //�趨 ʵ�ʶ�Ϊ��
		{
			if(setnum - *actnum >= 0)    //�趨ֵ����ʵ��ֵ  ����
			{
				if(setnum - *actnum >= acc)
				{
					*actnum += acc;
				}
				if(setnum - *actnum < acc)
				{
					*actnum = setnum;
				}
			}
			else if(setnum - *actnum < 0)      //�趨ֵС��ʵ��ֵ  ����
			{
				if(*actnum - setnum >= dec)
				{
					*actnum -= dec;
				}
				if(*actnum - setnum < dec)
				{
					*actnum = setnum;
				}
			}
		}                                       
		if(setnum < 0)                //�趨Ϊ�� ʵ��Ϊ�� �����ٶ�
		{
			if(*actnum - setnum >= acc)
			{
				*actnum -= acc;
			}
			if(*actnum - setnum < acc)
			{
				*actnum = setnum;
			}
		}                                    
	}
	if(*actnum < 0)   
	{
		if(setnum < 0)       //�趨ʵ�ʶ�Ϊ��
		{
			if(setnum - *actnum <= 0)       //������
			{
				if(setnum - *actnum <= -acc)
				{
					*actnum -= acc;
				}
				if(setnum - *actnum > -acc)
				{
					*actnum = setnum;
				}
			}
			if(setnum - *actnum > 0)       //������
			{
				if(*actnum - setnum <= -dec)
				{
					*actnum += dec;
				}
				if(*actnum - setnum > -dec)
				{
					*actnum = setnum;
				}
			}
		}                                       
		if(setnum > 0)                    //�趨Ϊ�� ʵ��Ϊ��
		{
			if(setnum - *actnum >= dec)
			{
				*actnum += dec;
			}
			if(setnum - *actnum < dec)
			{
				*actnum = setnum;
			}
		}                                    
	}
}

