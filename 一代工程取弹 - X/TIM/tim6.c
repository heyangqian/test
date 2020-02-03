#include "tim6.h"

void TIM6_Configuration(void)
{
    TIM_TimeBaseInitTypeDef  tim;
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	
    tim.TIM_Period = 2000-1;
    tim.TIM_Prescaler = 42-1;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM6,&tim);
	
	  TIM_ITConfig(TIM6, TIM_IT_Update,ENABLE);
		TIM_Cmd(TIM6, ENABLE);	 

}

