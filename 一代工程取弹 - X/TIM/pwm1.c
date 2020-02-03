#include "pwm1.h"			//TIM1

/*-PWM1---(PE9 ---TIM1_CH1)--*/
/*-PWM2---(PE11---TIM1_CH2)--*/
/*-PWM3---(PE13---TIM1_CH3)--*/
/*-PWM4---(PE14---TIM1_CH4)--*/

void PWM_1_Configuration(void)
{
    GPIO_InitTypeDef          gpio;
    TIM_TimeBaseInitTypeDef   tim;
    TIM_OCInitTypeDef         oc;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11| GPIO_Pin_13| GPIO_Pin_14;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOE,&gpio);
    
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource11, GPIO_AF_TIM1);  
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE,GPIO_PinSource14, GPIO_AF_TIM1);  	
    
    tim.TIM_Prescaler = 168-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 20000-1;   						
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1,&tim);
    
    oc.TIM_OCMode = TIM_OCMode_PWM2;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputState_Disable;
    oc.TIM_Pulse = 10;
    oc.TIM_OCPolarity = TIM_OCPolarity_Low;
    oc.TIM_OCNPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM1,&oc);//根据T指定的参数初始化外设TIM1 OC1
    TIM_OC2Init(TIM1,&oc);
	TIM_OC3Init(TIM1,&oc);
    TIM_OC4Init(TIM1,&oc);
	
    
    TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable); //使能TIM1在CCR1上的预装载寄存器
    TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
           
    TIM_ARRPreloadConfig(TIM1,ENABLE);
    
    TIM_CtrlPWMOutputs(TIM1,ENABLE);
    
    TIM_Cmd(TIM1,ENABLE);
}
