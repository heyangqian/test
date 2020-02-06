#ifndef __CAN2_H__
#define __CAN2_H__
#include "main.h"

void CAN2_Configuration(void);
void CAN2_RX0_IRQHandler(void);
extern CanRxMsg 	Rx2_message;
extern CanTxMsg 	Tx2_message;
#endif 




