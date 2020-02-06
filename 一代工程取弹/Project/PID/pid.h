#ifndef _PID_H_
#define _PID_H_
#include "main.h"
#include "stm32f4xx.h"
typedef struct
{
	float SetNum;       //设定值
	float ActualNum;    //实际值
	float Act_num;     //实际值限制值
	float Set_num;      //设定值限制值
	float error[3];          //实际与设定误差值（3次）
	float output;      //输出值（控制执行器的变量） 
	float output_last;   //上一次的输出值
	float integral;     //积分值
	float Kp;      //比例参数
	float Ki;      //积分和参数
	float Kd;      //微分参数
	float Ti;      //积分时间
	float Td;      //微分时间
	float differential[3];//误差的差值
	float P_out;   //比例输出
	float I_out;   //积分输出
	float D_out;   //微分输出
	int16_t angle_now;   //当前机械角度
	int16_t angle_last;  // 上一次机械角度
	int16_t first_angle; //上电后第一次机械角度
	float err;       //两次机械角度误差
//	struct
//	{
//		float err[7];
//		float ec_err[7];
//		float NB,NM,NS,ZO,PS,PM,PB;
//		
//	} Fuzzy;
 
} PIDxtype;
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

extern int8_t Add_flag;     //位置环绝对变增量初始值检测标志位
extern PIDxtype Claw_p_pid;   //抓弹爪子位置pid（一代）
extern PIDxtype Claw_s_pid;   //抓弹爪子速度pid（一代）
extern PIDxtype Claw2_p_pid;   //抓弹爪子位置pid（一代）
extern PIDxtype Claw2_s_pid;   //抓弹爪子速度pid（一代）
extern PIDxtype Send_p_pid;    //送弹位置pid
extern PIDxtype Send_s_pid;    //送弹速度pid

extern PIDxtype x_p_pid;      //抓弹x平移位置pid
extern PIDxtype x_s_pid;      //抓弹x平移速度pid
//extern PIDxtype x2_p_pid;
//extern PIDxtype x2_s_pid;

extern float Pit;
void pid_MInit(PIDxtype *pid, float Kp , float Ki , float Kd);  //电机pid初始化
void pid_GInit(PIDxtype *pid, float Kp , float Ti , float Td);  //云台电机初始化
void pid_Posit(PIDxtype *pid,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout);//位置式   速度环  积分分离
void Data_sends(int16_t data1,int16_t data2,int16_t data3,int16_t data4,int16_t data5,int16_t data6);   //发送数据
void Increment_Angle(PIDxtype *PId_p,int16_t an_now);//绝对变增量（位置环）
void pid_Increment(PIDxtype *PId_i,float actualnum ,int16_t setnum,int16_t maxout);  //增量pid  
void speed_accelerate(float setspeed,float *speed_transit,int16_t acc,int16_t  dec);// 加速度
void acclerate(float setnum,float *actnum,int16_t acc,int16_t dec);  //自写加速度
void pid_incomplete(PIDxtype *PId_i,float actualnum ,float setnum,float err_limit,int16_t imaxout,int16_t maxout);  // 不完全微分
#endif

