#ifndef __LED_H__
#define __LED_H__
#include "main.h"

void Led_Configuration(void);

#define  LED_GREEN_ON()      GPIO_SetBits(GPIOE, GPIO_Pin_8)
#define  LED_GREEN_OFF()     GPIO_ResetBits(GPIOE, GPIO_Pin_8)
#define  LED_GREEN_TOGGLE()  GPIO_ToggleBits(GPIOE, GPIO_Pin_8)

#define  LED_RED_ON()        GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define  LED_RED_OFF()       GPIO_ResetBits(GPIOE, GPIO_Pin_7)
#define  LED_RED_TOGGLE()    GPIO_ToggleBits(GPIOE, GPIO_Pin_7)

#endif
