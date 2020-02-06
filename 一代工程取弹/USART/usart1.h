#ifndef __USART1_H__
#define __USART1_H__
#include "main.h"



#if  STSTEM_SUPPORT_OS
#include "FreeRTOS.h"
#endif

#define USART_REC_LEN  			200  	//定义最大接收字节数 200


void USART1_Configuration(void);
void USART1_SendChar(unsigned char a);

#endif
