#include "led.h"

/*----LED_GREEN----PA6-----'0' is on,'1' is off */
/*----LED_RED------PA7-----'0' is on,'1' is off */

void Led_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOC| RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOD,ENABLE);
	
	//     A
	gpio.GPIO_Pin = GPIO_Pin_5 ;            //气缸
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_2;   //触碰开关
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_0;       //红外
	gpio.GPIO_Mode = GPIO_Mode_IN;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd =  GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&gpio);
	
	gpio.GPIO_Pin = GPIO_Pin_10;            
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&gpio);
	
	// B
	gpio.GPIO_Pin = GPIO_Pin_0;   //气缸
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&gpio);
	
	
	// C
	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_1;        //气缸
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&gpio);
	
//	gpio.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_4;      //触碰
//	gpio.GPIO_Mode = GPIO_Mode_IN;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOC,&gpio);
	
//	gpio.GPIO_Pin = GPIO_Pin_12;      //红外
//	gpio.GPIO_Mode = GPIO_Mode_IN;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_PuPd =  GPIO_PuPd_NOPULL;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOC,&gpio);

	// D
//	gpio.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;    //触碰
//	gpio.GPIO_Mode = GPIO_Mode_IN;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_PuPd = GPIO_PuPd_UP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOD,&gpio);
//	
//	gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_5 ;    //红外
//	gpio.GPIO_Mode = GPIO_Mode_IN;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_PuPd =  GPIO_PuPd_UP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOD,&gpio);
//	
//	// E
//	gpio.GPIO_Pin = GPIO_Pin_10;   //气缸
//	gpio.GPIO_Mode = GPIO_Mode_OUT;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOE,&gpio);
//	
//	gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_14 | GPIO_Pin_13;     //气缸
//	gpio.GPIO_Mode = GPIO_Mode_OUT;
//	gpio.GPIO_OType = GPIO_OType_PP;
//	gpio.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOE,&gpio);
	
    LED_GREEN_ON();
    LED_RED_ON();
}
