#include "laser.h"

void Laser_int()
{
	GPIO_InitTypeDef  gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	gpio.GPIO_Pin = GPIO_Pin_15;
	gpio.GPIO_Mode = GPIO_Mode_OUT;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOE,&gpio);
	GPIO_SetBits(GPIOE,GPIO_Pin_15);			//À­¸ß
}	









