#include "tim2.h"

void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef 	tim2;

	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  					//使能TIM2时钟
	
	
	tim2.TIM_Period 		= 	50 - 1; 									//自动重装载值	1ms
	tim2.TIM_Prescaler		= 	8400 - 1;  									//定时器分频
	tim2.TIM_CounterMode	=	TIM_CounterMode_Up; 						//向上计数模式
	tim2.TIM_ClockDivision	=	TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM2,&tim2);											//初始化TIM2
	
		//	TIM2_定时器2中断

//	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); 								//允许定时器2更新中断		运用中断时打开
	TIM_Cmd(TIM2,ENABLE); 													//使能定时器2
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) 
	{


	}
	TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	TIM_ClearFlag(TIM2, TIM_FLAG_Update);
}

