#include "main.h"
	/*FreeRTOS系统*/
#if SYSTEM_SUPPORT_OS					//控制FreeRTOS系统的打开和关闭
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "led.h"
#include  "pid.h"
#include "uart4.h"
#include "usart6.h"
#include "chassis.h"
#include "motor.h"
#include "gimbal.h"

#endif

int main(void)
{
	
/***************************************	初始化		***********************************************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置系统中断优先级分组4
	
#if SYSTEM_SUPPORT_OS
	delay_init(168);                               //初始化延时函数（系统时钟频率）
#endif		
	USART1_Configuration();			//USART1	串口打印数据·蓝牙115200		测试成功	
	Led_Configuration();
	CAN1_Configuration();			//CAN1		
	CAN2_Configuration();			//CAN2
	USART3_Configuration();			//USART3	妙算			未测试
	UART4_Configuration();
	USART6_Configuration();			//串口6初始化  激光测距2
	Chassis_Motor_Init();  //底盘电机pid初始化
	Gimbal_Motor_Init();   //云台电机pid初始化
	gyro_Init(&gyro_yaw);  //小黑块初始化
	Chassis_Init(&motor_chassis);  //底盘运动初始化
	Nvic();							//存放所有中断配置，便于配置 优先级	
	printf("/*-------------------- TITR-RM2019-HERO-GIMBAL-ORDER ---------------------*/\r\n");
/**************************************		任务开始函数		********************************************/
#if SYSTEM_SUPPORT_OS
	startTast();					//进入系统任务
	vTaskStartScheduler();          //开启任务调度
#else
	while(1){;}	
#endif						

}


