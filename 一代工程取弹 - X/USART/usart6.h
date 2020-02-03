#ifndef _USART6_H_
#define _USART6_H_
#include "stdio.h"
#include "sys.h"
#include "stm32f4xx.h"

void USART6_Configuration(void);

#define laster_buffer_length	19

typedef union
{
	char laster_rang_buff[4];
	float laster_range_date;
}Laster_Range;

extern Laster_Range laster_range;

int Date(char ppp);


#endif

