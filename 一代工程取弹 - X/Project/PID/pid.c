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

	
PIDxtype Claw_p_pid;  //爪子位置环pid
PIDxtype Claw_s_pid;  //爪子速度环pid
PIDxtype Claw2_p_pid; // 爪子2位置环（无用）
PIDxtype Claw2_s_pid; //爪子2速度环

PIDxtype x_p_pid;  //x横移位置环
PIDxtype x_s_pid;  //x横移速度环
//PIDxtype x2_p_pid; // x2
//PIDxtype x2_s_pid;
	
PIDxtype Send_p_pid;  //送弹pid位置环
PIDxtype Send_s_pid;  //送弹pid速度环
/***************位置式PID*****************
	函数名：PID_MInit 
	作  用： PID初始化
	参	数：函数指针
			Kp   Ki    Kd
******************************************/
void pid_MInit(PIDxtype *pid, float Kp , float Ki , float Kd)    //自写pid初始化
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

/***************位置式PID*****************
	函数名：PID_GInit 
	作  用： PID初始化
	参	数：函数指针
			Kp   Ti   Td
******************************************/
void pid_GInit(PIDxtype *pid, float Kp , float Ti , float Td)    //自写pid初始化
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
	函数名：Increment_Angle
	作  用： 调节位置环
	参	数： 
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
	函数名：PID_ Posit
	作  用： 位置型PID计算
	参	数：函数指针
			Kp Ki Kd 三参数
			P_out ,I_out , D_out 三输出
******************************************/
//PIDxtype motor_pid_p;
uint8_t X_flag = 0;
float Bit = 1;
void pid_Posit(PIDxtype *PId_p,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout)   //位置型PID
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
	if(ele1_flag == 0 || ele2_flag == 0)      //是否上电
	{
		X_flag = 0;
	}
	if(X_flag == 0)       //积分清零  判断是否累加
	{
		PId_p->integral = 0;
		if(fabs(PId_p->error[0]) <= err_limit)
		{
			X_flag = 1;
		}
	}
	if(X_flag !=0)        //积分累加
	{
		Bit = (err_limit - fabs(PId_p->error[0]))/err_limit;
		if(Bit <= 0)
		{
			Bit = 1;
		}
		PId_p->integral +=((PId_p->error[0] + PId_p->error[1]) / 2);
	}
	LimitMax(PId_p->integral, imaxout);   //15000
	PId_p->differential[2] =  PId_p->differential[1];       //微分平移
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
	函数名：PID_ increment
	作  用： 增量型PID计算
	参	数：函数指针
			Kp Ki Kd 三参数
			P_out ,I_out , D_out 三输出
******************************************/
//PIDxtype motor_pid_i;
void pid_Increment(PIDxtype *PId_i,float actualnum ,int16_t setnum,int16_t maxout)   //增量PID
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
	函数名：PID_ incomplete
	作  用： 不完全微分PID计算
	参	数：函数指针
			Kp Ki Kd 三参数
			P_out ,I_out , D_out 三输出
******************************************/
uint8_t Y_flag = 0;
float Pit = 0;
void pid_incomplete(PIDxtype *PId_in,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout)   //不完全PID
{
	PId_in->error[2] = PId_in->error[1];     //误差平移
    PId_in->error[1] = PId_in->error[0];
	PId_in->ActualNum = actualnum;
	PId_in->SetNum = setnum;
	PId_in->error[0] = (PId_in->SetNum) - (PId_in->ActualNum);
	if(ele1_flag == 0 && ele2_flag == 0)      //是否上电
	{
		Y_flag = 0;
	}
	if(Y_flag == 0)       //积分清零  判断是否累加
	{
		PId_in->integral = 0;
		if(fabs(PId_in->error[0]) <= err_limit)
		{
			Y_flag = 1;
		}
	}
	if(Y_flag !=0)        //积分累加
	{
//		Pit = (err_limit - fabs(PId_in->error[0]))/err_limit;
//		if(Pit <= 0)
//		{
//			Pit = 1;
//		}
		PId_in->integral +=((PId_in->error[0] + PId_in->error[1]) / 2);
	}
	LimitMax(PId_in->integral, imaxout);
	PId_in->differential[2] =  PId_in->differential[1];       //微分平移
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
	函数名：Data_sends
	作  用： PID数据发送波形
	参	数： data1 2 3 4 5  数据
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
  * @brief  加速度处理运算
  * @param  setspeed	 --目标速度
			speed_transit--速度环的设定速度
			acc			 --加速度			dec--减速度
  * @retval void
  * @note
**/	
void speed_accelerate(float setspeed,float *speed_transit,int16_t acc,int16_t  dec)
{

	if ((*speed_transit > 0) || (*speed_transit == 0 && setspeed >= 0))
	{
		if ((setspeed - *speed_transit > acc) && (setspeed > *speed_transit))	//目标速度大于当前速度 加速
		{
			*speed_transit	+= acc;
		}
		else if (( *speed_transit - setspeed > dec) && (setspeed < *speed_transit))//目标速度小于当前速度 减速
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
	函数名：acclerate
	作  用： 加速度
	参	数： float setnum    设定速度
			float *actnum    实际速度（指针）
			int16_t acc        加速度
			int16_t dec        减速度
******************************************/
void acclerate(float setnum,float *actnum,int16_t acc,int16_t dec)
{
	if(*actnum >= 0)
	{
		if(setnum >= 0)     //设定 实际都为正
		{
			if(setnum - *actnum >= 0)    //设定值大于实际值  加速
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
			else if(setnum - *actnum < 0)      //设定值小于实际值  减速
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
		if(setnum < 0)                //设定为负 实际为正 减加速度
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
		if(setnum < 0)       //设定实际都为负
		{
			if(setnum - *actnum <= 0)       //负加速
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
			if(setnum - *actnum > 0)       //负减速
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
		if(setnum > 0)                    //设定为正 实际为负
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

