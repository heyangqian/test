#ifndef _START_H_
#define _START_H_
#include "main.h"



void vl53l0x_task(void *pvParameters);
void test_task(void *pvParameters);
void query_task(void *pvParameters);
void interrupt_task(void *pvParameters);
void startTast(void);
void PRINTF_task(void *pvParameters);
void Chassis_task(void *pvParameter);
void Gas_task(void *pvParameter);
void Printf_task(void *pvParameter);
void Motor_task(void *pvParameter);
void Motor_x_task(void *pvParameter);
extern void VL53L0X_task(void *pvParameters);
extern void vl53l0x_2_task(void *pvParameters);

#endif






