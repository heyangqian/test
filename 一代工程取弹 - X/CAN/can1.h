#ifndef __CAN1_H__
#define __CAN1_H__
#include "main.h"
void CAN1_Configuration(void);
void CAN1_RX0_IRQHandler(void);
extern CanRxMsg 	Rx_message;
extern CanTxMsg 	Tx_message;

#endif 
