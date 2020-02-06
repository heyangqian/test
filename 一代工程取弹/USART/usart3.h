#ifndef __USART6_H__
#define __USART6_H__


void CopeSerialData(unsigned char ucData);
void USART3_Configuration(void);
extern void USART3_SendChar(unsigned char b);


#define Laster_Buffer_Length_2      60



extern  int  RX_BUF_1[12];
extern   unsigned   char    RX_BUF[22]; 
#endif
