#include "Nvic.h"



/***********************		中断优先级设定		*****************************/
void Nvic()
{
	NVIC_InitTypeDef 		nvic;
	
	// 串口2	空闲中断，用于   接收 裁判系统数据
	nvic.NVIC_IRQChannel 						= 	USART2_IRQn;		//串口2中断通道
    nvic.NVIC_IRQChannelPreemptionPriority 		= 	5;					//抢占优先级3
    nvic.NVIC_IRQChannelSubPriority 			= 	0;					//子优先级2
    nvic.NVIC_IRQChannelCmd						= 	ENABLE;				//IRQ通道使能
    NVIC_Init(&nvic);													//根据指定的参数初始化VIC寄存器
	
//	// DMA1	数据流6	中断，用于给  裁判系统 发送 数据
//	nvic.NVIC_IRQChannel           				= 	DMA1_Stream6_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority		= 	3;
//	nvic.NVIC_IRQChannelSubPriority     		= 	0;
//	nvic.NVIC_IRQChannelCmd         			= 	ENABLE;
//	NVIC_Init(&nvic);

	// 串口3	空闲中断，用于 接收 激光测距 1 数据
	nvic.NVIC_IRQChannel 						= 	USART3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority 		= 	5;
	nvic.NVIC_IRQChannelSubPriority 			= 	0;
	nvic.NVIC_IRQChannelCmd						= 	ENABLE;
	NVIC_Init(&nvic);
	
	// 串口4	空闲中断，用于 接收 遥控器数据
	nvic.NVIC_IRQChannel 						= 	UART4_IRQn;							
    nvic.NVIC_IRQChannelPreemptionPriority 		= 	4;
    nvic.NVIC_IRQChannelSubPriority 			=   0;
    nvic.NVIC_IRQChannelCmd 					=	 ENABLE;
    NVIC_Init(&nvic);
	
	// 串口5	空闲中断，用于 接收 绝对值陀螺仪数据
//	nvic.NVIC_IRQChannel 						= 	UART5_IRQn;
//	nvic.NVIC_IRQChannelPreemptionPriority 		= 	5;
//	nvic.NVIC_IRQChannelSubPriority 			= 	0;
//	nvic.NVIC_IRQChannelCmd						= 	ENABLE;
//	NVIC_Init(&nvic);
	
	// 串口6	空闲中断，用于 接收 激光测距 2 数据
	nvic.NVIC_IRQChannel 						= 	USART6_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority 		= 	5;
	nvic.NVIC_IRQChannelSubPriority 			= 	0;
	nvic.NVIC_IRQChannelCmd						= 	ENABLE;
	NVIC_Init(&nvic);
	
	//	CAN1	中断
	nvic.NVIC_IRQChannel 						= 	CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority 		= 	1;
    nvic.NVIC_IRQChannelSubPriority 			= 	0;
    nvic.NVIC_IRQChannelCmd 					= 	ENABLE;
    NVIC_Init(&nvic);
	
	//	CAN2	中断
	nvic.NVIC_IRQChannel 						= 	CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority 		= 	1;
    nvic.NVIC_IRQChannelSubPriority 			= 	0;
    nvic.NVIC_IRQChannelCmd 					= 	ENABLE;
    NVIC_Init(&nvic);
	
//	nvic.NVIC_IRQChannel						=	TIM2_IRQn; 		
//	nvic.NVIC_IRQChannelPreemptionPriority		=	5; 				
//	nvic.NVIC_IRQChannelSubPriority				=	2; 				
//	nvic.NVIC_IRQChannelCmd						=	ENABLE;	
//	NVIC_Init(&nvic);
//	
//	
//	//	TIM6_DAC	中断
//	nvic.NVIC_IRQChannel 						= 	TIM6_DAC_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority		= 	0;
//    nvic.NVIC_IRQChannelSubPriority				= 	0;
//    nvic.NVIC_IRQChannelCmd 					= 	DISABLE;
//    NVIC_Init(&nvic);
//	
//		
//	nvic.NVIC_IRQChannel						=	TIM7_IRQn; 		
//	nvic.NVIC_IRQChannelPreemptionPriority		=	1; 				
//	nvic.NVIC_IRQChannelSubPriority				=	3; 				
//	nvic.NVIC_IRQChannelCmd						=	ENABLE;	
//	NVIC_Init(&nvic);
//	
//	
//	
//	//	TIM8_定时器8中断
//	nvic.NVIC_IRQChannel						=	 TIM8_CC_IRQn ; 		
//	nvic.NVIC_IRQChannelPreemptionPriority		=	0; 				
//	nvic.NVIC_IRQChannelSubPriority				=	0; 				
//	nvic.NVIC_IRQChannelCmd						=	ENABLE;	
//	NVIC_Init(&nvic);
//	
//	
//	//按键	外部中断
//	nvic.NVIC_IRQChannel 						= 	EXTI0_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority 		= 	2;
//    nvic.NVIC_IRQChannelSubPriority 			= 	1;
//    nvic.NVIC_IRQChannelCmd 					= 	ENABLE;
//    NVIC_Init(&nvic);


}



