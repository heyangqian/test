#ifndef __VL53L0X_CAIL_2_H_
#define __VL53L0X_CAIL_2_H_

#include "vl53l0x_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X-校准模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//vl53l0x传感器校准信息结构体定义
typedef __packed struct
{
	uint8_t  adjustok;                    //校准成功标志，0XAA，已校准;其他，未校准
	uint8_t  isApertureSpads;             //ApertureSpads值
	uint8_t  VhvSettings;                 //VhvSettings值
	uint8_t  PhaseCal;                    //PhaseCal值
	uint32_t XTalkCalDistance;            //XTalkCalDistance值
	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCps值
	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeter值
	int32_t  OffsetMicroMeter;            //OffsetMicroMeter值
	uint32_t refSpadCount;                //refSpadCount值
	
}_vl53l0x_adjust_2;

extern _vl53l0x_adjust_2 Vl53l0x_data_2;

void vl53l0x_calibration_test_2(VL53L0X_Dev_t_2 *dev);

#endif
