#include "tim8.h"

void TIM8_Configuration(void)
{
	TIM_TimeBaseInitTypeDef 	tim;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);

	tim.TIM_ClockDivision		=	TIM_CKD_DIV1;
	tim.TIM_CounterMode			=	TIM_CounterMode_Up;
	tim.TIM_Period				=	100;
	tim.TIM_Prescaler			=	1680-1;
	tim.TIM_RepetitionCounter	=	0;
	TIM_TimeBaseInit(TIM8,&tim);
	
	TIM_ClearFlag(TIM8,TIM_FLAG_Update);
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE); 								//允许定时器2更新中断
	TIM_Cmd(TIM8,ENABLE);

	
}

int tim_8=0;
	
void TIM8_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM8,TIM_IT_Update)==SET)
	{
		tim_8++;
		TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
		TIM_ClearFlag(TIM8, TIM_FLAG_Update);
	}
	
}






