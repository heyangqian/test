#ifndef __VL53L0X_GEN_2_H
#define __VL53L0X_GEN_2_H

#include "vl53l0x_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ̽����STM32F407������
//VL53L0X-��ͨ����ģʽ ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
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


