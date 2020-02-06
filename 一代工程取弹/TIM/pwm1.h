#ifndef __PWM1_H__
#define __PWM1_H__
#include "main.h"

void PWM_1_Configuration(void);

#define PWM1  TIM1->CCR
#define PWM2  TIM1->CCR1

#endif


