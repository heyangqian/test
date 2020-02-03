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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ̽����STM32F407������
//VL53L0X-���ܲ��� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////  

//VL53L0X�������ϵ�Ĭ��IIC��ַΪ0X52(���������λ)
#define VL53L0X_Addr_2 0x52

//����Xshut��ƽ,�Ӷ�ʹ��VL53L0X���� 1:ʹ�� 0:�ر�
#define VL53L0X_Xshut_2 PCout(0)		

//ʹ��2.8V IO��ƽģʽ
#define USE_I2C_2V8_2  1

//����ģʽ
#define Default_Mode_2   0// Ĭ��
#define HIGH_ACCURACY_2  1//�߾���
#define LONG_RANGE_2     2//������
#define HIGH_SPEED_2     3//����

//vl53l0xģʽ���ò�����
typedef __packed struct
{
	FixPoint1616_t_2 signalLimit;    //Signal������ֵ 
	FixPoint1616_t_2 sigmaLimit;     //Sigmal������ֵ
	uint32_t timingBudget;         //����ʱ������
	uint8_t preRangeVcselPeriod ;  //VCSEL��������
	uint8_t finalRangeVcselPeriod ;//VCSEL�������ڷ�Χ
	
}mode_data_2;


extern mode_data_2 Mode_data_2[];
extern uint8_t AjustOK_2;

VL53L0X_Error_2 vl53l0x_init_2(VL53L0X_Dev_t_2 *dev);//��ʼ��vl53l0x
void print_pal_error_2(VL53L0X_Error_2 Status);//������Ϣ��ӡ
void mode_string_2(u8 mode,char *buf);//ģʽ�ַ�����ʾ
void vl53l0x_test_2(void);//vl53l0x����
void vl53l0x_reset_2(VL53L0X_Dev_t_2 *dev);//vl53l0x��λ

void vl53l0x_info_2(void);//��ȡvl53l0x�豸ID��Ϣ
void One_measurement_2(u8 mode);//��ȡһ�β�����������
#endif


