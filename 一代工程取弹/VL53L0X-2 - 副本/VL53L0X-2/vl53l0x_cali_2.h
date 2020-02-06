#ifndef __VL53L0X_CAIL_2_H_
#define __VL53L0X_CAIL_2_H_

#include "vl53l0x_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ̽����STM32F407������
//VL53L0X-У׼ģʽ ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//vl53l0x������У׼��Ϣ�ṹ�嶨��
typedef __packed struct
{
	uint8_t  adjustok;                    //У׼�ɹ���־��0XAA����У׼;������δУ׼
	uint8_t  isApertureSpads;             //ApertureSpadsֵ
	uint8_t  VhvSettings;                 //VhvSettingsֵ
	uint8_t  PhaseCal;                    //PhaseCalֵ
	uint32_t XTalkCalDistance;            //XTalkCalDistanceֵ
	uint32_t XTalkCompensationRateMegaCps;//XTalkCompensationRateMegaCpsֵ
	uint32_t CalDistanceMilliMeter;       //CalDistanceMilliMeterֵ
	int32_t  OffsetMicroMeter;            //OffsetMicroMeterֵ
	uint32_t refSpadCount;                //refSpadCountֵ
	
}_vl53l0x_adjust_2;

extern _vl53l0x_adjust_2 Vl53l0x_data_2;

void vl53l0x_calibration_test_2(VL53L0X_Dev_t_2 *dev);

#endif
