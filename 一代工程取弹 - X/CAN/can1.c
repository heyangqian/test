#include "can1.h"
#include "gimbal.h"
#include "motor.h"
#include "can_task.h"
CanRxMsg 	Rx_message;
CanTxMsg 	Tx_message;
/**
  * @brief  CAN1相关配置
  * @param  void
  * @retval void
  * @note	CAN1_TX-----PA12		CAN1_RX-----PA11
  */

void CAN1_Configuration(void)
{
		CAN_InitTypeDef        			can;
		CAN_FilterInitTypeDef  			can_f;
		GPIO_InitTypeDef       			gpio;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,  ENABLE);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

		gpio.GPIO_Pin  = GPIO_Pin_12 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOA, &gpio);						

		CAN_DeInit(CAN1);
		CAN_StructInit(&can);

		can.CAN_TTCM 		= 	DISABLE;	
		can.CAN_ABOM 		= 	ENABLE;	
		can.CAN_AWUM 		= 	DISABLE;	
		can.CAN_NART 		= 	DISABLE;	
		can.CAN_RFLM 		= 	DISABLE;	
		can.CAN_TXFP 		= 	ENABLE;	
		can.CAN_Mode 		= 	CAN_Mode_Normal;	

		/*设置波特率*/
		can.CAN_SJW  		= 	CAN_SJW_1tq;
		can.CAN_BS1 		= 	CAN_BS1_9tq;	
		can.CAN_BS2 		= 	CAN_BS2_4tq;	
		can.CAN_Prescaler 	= 	3;   // CAN BaudRate 42/(1+9+4)/3=1Mbps	分频系数（Fdiv）
		CAN_Init(CAN1, &can);	//初始化CAN1

		can_f.CAN_FilterNumber			=	0;	
		can_f.CAN_FilterMode			=	CAN_FilterMode_IdMask;	
		can_f.CAN_FilterScale			=	CAN_FilterScale_32bit;	
		can_f.CAN_FilterIdHigh			=	0x0000;
		can_f.CAN_FilterIdLow			=	0x0000;	
		can_f.CAN_FilterMaskIdHigh		=	0x0000;	
		can_f.CAN_FilterMaskIdLow		=	0x0000;	
		can_f.CAN_FilterFIFOAssignment	=	0;	
		can_f.CAN_FilterActivation		=	ENABLE;	
		CAN_FilterInit(&can_f);

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);	
		CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_RX0_IRQHandler(void)
{
	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &Rx_message);		
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		Can1_motor_handle(&Rx_message);
//		Can1_Mes_handle(&Rx_message);
		ele1_flag = 1;
	}
}








