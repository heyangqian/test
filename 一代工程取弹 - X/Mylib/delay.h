#ifndef __DELAY_H
#define __DELAY_H 			   
#include <sys.h>	  

#if SYSTEM_SUPPORT_OS
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
#else
void delay_ms(unsigned int t);
void delay_us(unsigned int t);
#endif

void delay_xms(u16 nms);


#endif





























