#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "delay.h"

#include "laser.h"
//#include "led.h"

#include "Nvic.h"			//存放 所有 中断 配置，便于配置 优先级

#include "usart1.h"			//串口 1 用于 打印数据·蓝牙
#include "usart3.h"			//串口 6 用于 激光测距2
//#include "usart6.h"

#include "dbus.h"			/*  遥控器  */


//#include "judge.h"			/* 裁判系统 */

/*定时器控制*/
#include "pwm1.h"			//TIM1			电机1-4

#include "can1.h"			//CAN1
#include "can2.h"			//CAN2


/*	MPU6050	板载 */
#include "mpu6050_driver.h"
#include "mpu6050_i2c.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_process.h"


#include "start.h"


#include "stdint.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "stdlib.h"	 
#include "sys.h" 


#endif 


