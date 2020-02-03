#include "encoder.h"

//TIM4_CH1 ----- PD12
//TIM4_CH2 ----- PD13

//TIM5_CH1 ----- PA0
//TIM5_CH2 ----- PA1

//编码器
void Encoder_1_Configuration(void)
{
	GPIO_InitTypeDef         gpio; 
	TIM_TimeBaseInitTypeDef  tim;
	TIM_ICInitTypeDef        tim_ic;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,	ENABLE);//开启TIM4时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//开启GPIOB时钟

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);//PD12引脚复用
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);//PD13引脚服用

	gpio.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; 
	gpio.GPIO_Mode  = GPIO_Mode_AF;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_OType = GPIO_OType_OD;
	gpio.GPIO_PuPd  = GPIO_PuPd_UP ;
	GPIO_Init(GPIOD,&gpio); 

	tim.TIM_Period = 60000; //设置下一个更新事件装入活动的自动重装载寄存器周期的值
	tim.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	tim.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	tim.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM4, &tim); 

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);
	TIM_ICStructInit(&tim_ic);
	tim_ic.TIM_ICFilter = 0;  //输入滤波器
	TIM_ICInit(TIM4, &tim_ic);
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);  //清楚所有标志位

	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //允许中断更新
	TIM4->CNT = 30000;
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
}

void Encoder_2_Configuration(void)
{
	GPIO_InitTypeDef         gpio; 
    TIM_TimeBaseInitTypeDef  tim;
    TIM_ICInitTypeDef        tim_ic;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//开启TIM5时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//开启GPIOB时钟
  
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5);//PA0引脚复用
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);//PA1引脚服用
 
    gpio.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1; 
    gpio.GPIO_Mode  = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_OType = GPIO_OType_OD;
    gpio.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA,&gpio); 
 
    tim.TIM_Period = 60000; //设置下一个更新事件装入活动的自动重装载寄存器周期的值
    tim.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    tim.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    tim.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM5, &tim); 
 
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);
    TIM_ICStructInit(&tim_ic);
    tim_ic.TIM_ICFilter = 0;  //输入滤波器
    TIM_ICInit(TIM5, &tim_ic);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);  //清楚所有标志位
	
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //允许中断更新
    TIM5->CNT = 30000;
    TIM_Cmd(TIM5, ENABLE);  //使能TIM5
}























//void Encoder1_Configuration(void)		//	有问题
//{
//	GPIO_InitTypeDef        gpio;
//	TIM_TimeBaseInitTypeDef tim;
//	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOA,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5 ,ENABLE);

//	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
//	gpio.GPIO_Mode = GPIO_Mode_AF;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOA,&gpio);

//	gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//	gpio.GPIO_Mode = GPIO_Mode_AF;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOB,&gpio);

//	tim.TIM_Period = 60000;
//	tim.TIM_Prescaler = 0;//当Prescaler+1>1时，倍频器起作用，因此计数频率为36M*2/(Prescaler+1)
//	tim.TIM_ClockDivision = TIM_CKD_DIV1;
//	tim.TIM_CounterMode = TIM_CounterMode_Up;				//上升沿计数
//	TIM_TimeBaseInit(TIM4, &tim);
//	TIM_TimeBaseInit(TIM5, &tim);

//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6,  GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7,  GPIO_AF_TIM4);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0,  GPIO_AF_TIM5);
//	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1,  GPIO_AF_TIM5);
//	
//	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_SetCounter(TIM4,30000); 
//	TIM_SetCounter(TIM5,30000); 
//	TIM_Cmd(TIM4, ENABLE);
//	TIM_Cmd(TIM5, ENABLE);
//}

