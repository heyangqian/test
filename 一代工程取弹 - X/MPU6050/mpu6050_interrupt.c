#include "main.h"

void MPU6050_Interrupt_Configuration(void)
{
    GPIO_InitTypeDef    gpio;
//    NVIC_InitTypeDef    nvic;
    EXTI_InitTypeDef    exti;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,  ENABLE);   

	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &gpio);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,GPIO_PinSource5); 
    
    exti.EXTI_Line = EXTI_Line5;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Falling;//�½����ж�
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);
    
//    nvic.NVIC_IRQChannel = EXTI4_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 0;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
}

//MPU6050 �ⲿ�жϴ�����
//void EXTI4_IRQHandler(void)
//{
//    if(EXTI_GetITStatus(EXTI_Line4) == SET)
//    {
//        
//        //��ȡMPU6050����,Ϊ��ʹ��̨�Ŀ��Ƹ�ƽ����
//        //ʹ��MPU6050�������������Ϊ�ٶȻ�����
//        //����ʹ�õ���巵�ػ�е�Ƕ�ֵ���ٶȻ���������������������
//        MPU6050_ReadData();                                              
//        
//        EXTI_ClearFlag(EXTI_Line4);          
//        EXTI_ClearITPendingBit(EXTI_Line4);
//    }
//}
