#include "main.h"
	/*FreeRTOSϵͳ*/
#if SYSTEM_SUPPORT_OS					//����FreeRTOSϵͳ�Ĵ򿪺͹ر�
#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  
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
	
/***************************************	��ʼ��		***********************************************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	
#if SYSTEM_SUPPORT_OS
	delay_init(168);                               //��ʼ����ʱ������ϵͳʱ��Ƶ�ʣ�
#endif		
	USART1_Configuration();			//USART1	���ڴ�ӡ���ݡ�����115200		���Գɹ�	
	Led_Configuration();
	CAN1_Configuration();			//CAN1		
	CAN2_Configuration();			//CAN2
	USART3_Configuration();			//USART3	����			δ����
	UART4_Configuration();
	USART6_Configuration();			//����6��ʼ��  ������2
	Chassis_Motor_Init();  //���̵��pid��ʼ��
	Gimbal_Motor_Init();   //��̨���pid��ʼ��
	gyro_Init(&gyro_yaw);  //С�ڿ��ʼ��
	Chassis_Init(&motor_chassis);  //�����˶���ʼ��
	Nvic();							//��������ж����ã��������� ���ȼ�	
	printf("/*-------------------- TITR-RM2019-HERO-GIMBAL-ORDER ---------------------*/\r\n");
/**************************************		����ʼ����		********************************************/
#if SYSTEM_SUPPORT_OS
	startTast();					//����ϵͳ����
	vTaskStartScheduler();          //�����������
#else
	while(1){;}	
#endif						

}


