#ifndef __VL53L0X_2_H_
#define __VL53L0X_2_H_

#include "vl53l0x_api_2.h"
#include "vl53l0x_platform_2.h"
#include "vl53l0x_gen_2.h"
#include "vl53l0x_cali_2.h"
#include "vl53l0x_it_2.h"
#include "stmflash.h"
#include "delay.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X-功能测试 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  

//VL53L0X传感器上电默认IIC地址为0X52(不包含最低位)
#define VL53L0X_Addr_2 0x52

//控制Xshut电平,从而使能VL53L0X工作 1:使能 0:关闭
#define VL53L0X_Xshut_2 PCout(0)		

//使能2.8V IO电平模式
#define USE_I2C_2V8_2  1

//测量模式
#define Default_Mode_2   0// 默认
#define HIGH_ACCURACY_2  1//高精度
#define LONG_RANGE_2     2//长距离
#define HIGH_SPEED_2     3//高速

//vl53l0x模式配置参数集
typedef __packed struct
{
	FixPoint1616_t_2 signalLimit;    //Signal极限数值 
	FixPoint1616_t_2 sigmaLimit;     //Sigmal极限数值
	uint32_t timingBudget;         //采样时间周期
	uint8_t preRangeVcselPeriod ;  //VCSEL脉冲周期
	uint8_t finalRangeVcselPeriod ;//VCSEL脉冲周期范围
	
}mode_data_2;


extern mode_data_2 Mode_data_2[];
extern uint8_t AjustOK_2;

VL53L0X_Error_2 vl53l0x_init_2(VL53L0X_Dev_t_2 *dev);//初始化vl53l0x
void print_pal_error_2(VL53L0X_Error_2 Status);//错误信息打印
void mode_string_2(u8 mode,char *buf);//模式字符串显示
void vl53l0x_test_2(void);//vl53l0x测试
void vl53l0x_reset_2(VL53L0X_Dev_t_2 *dev);//vl53l0x复位

void vl53l0x_info_2(void);//获取vl53l0x设备ID信息
void One_measurement_2(u8 mode);//获取一次测量距离数据
#endif


