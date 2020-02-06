#include "tim7.h"

void TIM7_Configuration(void)
{
	TIM_TimeBaseInitTypeDef 	tim7;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  					//使能TIM7时钟
	
	tim7.TIM_Period 		= 	10 - 1; 									//自动重装载值	1ms
	tim7.TIM_Prescaler		= 	8400 - 1;  									//定时器分频
	tim7.TIM_CounterMode	=	TIM_CounterMode_Up; 						//向上计数模式
	tim7.TIM_ClockDivision	=	TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM7,&tim7);											//初始化TIM7

	
//	TIM7_定时器2中断
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); 								//允许定时器2更新中断		运用中断时打开
	TIM_Cmd(TIM7,ENABLE); 													//使能定时器2
}

void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET) 
	{

	}
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
}

