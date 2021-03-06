#include "pwm2.h"


/*-PWM1---(PA6---TIM3_CH1)--*/
/*-PWM2---(PA7---TIM3_CH2)--*/
/*-PWM3---(PC8---TIM3_CH3)--*/
/*-PWM4---(PC9---TIM3_CH4)--*/

void PWM_2_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC,&gpio);
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource6, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource7, GPIO_AF_TIM3);  
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource9, GPIO_AF_TIM3);  	
    
    tim.TIM_Prescaler = 840-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 2000;   //2.5ms
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Enable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM3,&oc);
    TIM_OC2Init(TIM3,&oc);
	TIM_OC3Init(TIM3,&oc);
    TIM_OC4Init(TIM3,&oc);
    
    TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM3,ENABLE);
    
    TIM_CtrlPWMOutputs(TIM3,ENABLE);
    
    TIM_Cmd(TIM3,ENABLE);
}
