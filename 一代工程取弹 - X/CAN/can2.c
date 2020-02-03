#include "can2.h"
#include "can_task.h"
#include "motor.h"
/**
  * @brief  CAN2œ‡πÿ≈‰÷√
  * @param  void
  * @retval void
  * @note	CAN2_TX-----PB12		CAN1_RX-----PB13
  */
CanRxMsg 	Rx2_message;
CanTxMsg 	Tx2_message;
void CAN2_Configuration(void)
{
		CAN_InitTypeDef        			can;
		CAN_FilterInitTypeDef  			can_f;
		GPIO_InitTypeDef       			gpio;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);


		GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

		gpio.GPIO_Pin  = GPIO_Pin_13 | GPIO_Pin_12 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		GPIO_Init(GPIOB, &gpio);

		CAN_DeInit(CAN2);
		CAN_StructInit(&can);

		can.CAN_TTCM 		= 	DISABLE;
		can.CAN_ABOM 		= 	ENABLE;    
		can.CAN_AWUM 		= 	DISABLE;    
		can.CAN_NART 		= 	DISABLE;    
		can.CAN_RFLM 		= 	DISABLE;    
		can.CAN_TXFP 		= 	ENABLE;     
		can.CAN_Mode 		= 	CAN_Mode_Normal; 
		can.CAN_SJW  		= 	CAN_SJW_1tq;
		can.CAN_BS1 		= 	CAN_BS1_9tq;
		can.CAN_BS2 		= 	CAN_BS2_4tq;
		can.CAN_Prescaler = 	3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
		CAN_Init(CAN2, &can);

		can_f.CAN_FilterNumber			=	14;
		can_f.CAN_FilterMode			=	CAN_FilterMode_IdMask;
		can_f.CAN_FilterScale			=	CAN_FilterScale_32bit;
		can_f.CAN_FilterIdHigh			=	0x0000;
		can_f.CAN_FilterIdLow			=	0x0000;
		can_f.CAN_FilterMaskIdHigh		=	0x0000;
		can_f.CAN_FilterMaskIdLow		=	0x0000;
		can_f.CAN_FilterFIFOAssignment=	0;//the message which pass the filter save in fifo0
		can_f.CAN_FilterActivation		=	ENABLE;
		CAN_FilterInit(&can_f);

		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
		CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE); 

}
int yyy =0;
void CAN2_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_Receive(CAN2, CAN_FIFO0, &Rx2_message);		
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		Can2_motor_handle(&Rx2_message);
		ele2_flag = 1;
//		Can2_Mes_handle(&Rx2_message);
	}
}



