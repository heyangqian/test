#ifndef __VL53L0X_GEN_2_H
#define __VL53L0X_GEN_2_H

#include "vl53l0x_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X-普通测量模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

extern VL53L0X_RangingMeasurementData_t_2 vl53l0x_data_2;

VL53L0X_Error_2 vl53l0x_set_mode_2(VL53L0X_Dev_t_2 *dev,u8 mode);
void vl53l0x_general_test_2(VL53L0X_Dev_t_2 *dev);
void vl53l0x_general_start_2(VL53L0X_Dev_t_2 *dev,u8 mode);
VL53L0X_Error_2 vl53l0x_start_single_test_2(VL53L0X_Dev_t_2 *dev,VL53L0X_RangingMeasurementData_t_2 *pdata,char *buf);



typedef struct
{
	float distance;
}VL5310X_2;


extern VL5310X_2 vl5310x_2;

#endif


