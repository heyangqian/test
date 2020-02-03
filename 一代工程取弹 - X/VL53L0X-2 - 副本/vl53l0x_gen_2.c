#include "vl53l0x_gen_2.h"


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

VL53L0X_RangingMeasurementData_t_2 vl53l0x_data_2;//�������ṹ��
 VL5310X_2 vl5310x_2;

//VL53L0X ����ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������
VL53L0X_Error_2 vl53l0x_set_mode_2(VL53L0X_Dev_t_2 *dev,u8 mode)
{
	
	 VL53L0X_Error_2 status = VL53L0X_ERROR_NONE_2;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	// vl53l0x_reset_2(dev);//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	 status = VL53L0X_StaticInit_2(dev);

     if(AjustOK_2!=0)//��У׼����,д��У׼ֵ
     {
	    status= VL53L0X_SetReferenceSpads_2(dev,Vl53l0x_data_2.refSpadCount,Vl53l0x_data_2.isApertureSpads);//�趨SpadsУ׼ֵ
        if(status!=VL53L0X_ERROR_NONE_2) goto error;	
        delay_ms(2);		 
	    status= VL53L0X_SetRefCalibration_2(dev,Vl53l0x_data_2.VhvSettings,Vl53l0x_data_2.PhaseCal);//�趨RefУ׼ֵ
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
		 delay_ms(2);
	    status=VL53L0X_SetOffsetCalibrationDataMicroMeter_2(dev,Vl53l0x_data_2.OffsetMicroMeter);//�趨ƫ��У׼ֵ
		if(status!=VL53L0X_ERROR_NONE_2) goto error; 
		 delay_ms(2);
		status=VL53L0X_SetXTalkCompensationRateMegaCps_2(dev,Vl53l0x_data_2.XTalkCompensationRateMegaCps);//�趨����У׼ֵ
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
         delay_ms(2);		 
		 
     }
	 else
	 {
		status = VL53L0X_PerformRefCalibration_2(dev, &VhvSettings, &PhaseCal);//Ref�ο�У׼
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement_2(dev, &refSpadCount, &isApertureSpads);//ִ�вο�SPAD����
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
        delay_ms(2);		 	 
	 }
	 status = VL53L0X_SetDeviceMode_2(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING_2);//ʹ�ܵ��β���ģʽ
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable_2(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,1);//ʹ��SIGMA��Χ���
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable_2(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,1);//ʹ���ź����ʷ�Χ���
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue_2(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,Mode_data_2[mode].sigmaLimit);//�趨SIGMA��Χ
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue_2(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,Mode_data_2[mode].signalLimit);//�趨�ź����ʷ�Χ��Χ
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(dev,Mode_data_2[mode].timingBudget);//�趨��������ʱ��
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod_2(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE_2, Mode_data_2[mode].preRangeVcselPeriod);//�趨VCSEL��������
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod_2(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2, Mode_data_2[mode].finalRangeVcselPeriod);//�趨VCSEL�������ڷ�Χ
	 
	 error://������Ϣ
	 if(status!=VL53L0X_ERROR_NONE_2)
	 {
		print_pal_error_2(status);
		return status;
	 }
	 return status;
	
}	

//VL53L0X ���ξ����������
//dev:�豸I2C�����ṹ��
//pdata:����������ݽṹ��
VL53L0X_Error_2 vl53l0x_start_single_test_2(VL53L0X_Dev_t_2 *dev,VL53L0X_RangingMeasurementData_t_2 *pdata,char *buf)
{
	VL53L0X_Error_2 status = VL53L0X_ERROR_NONE_2;
	uint8_t RangeStatus_2;
	
	status = VL53L0X_PerformSingleRangingMeasurement_2(dev, pdata);//ִ�е��β�ಢ��ȡ����������
	if(status !=VL53L0X_ERROR_NONE_2) return status;
   
	RangeStatus_2 = pdata->RangeStatus_2;//��ȡ��ǰ����״̬
    memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH_2);
	VL53L0X_GetRangeStatusString_2(RangeStatus_2,buf);//���ݲ���״̬��ȡ״̬�ַ���
	
	vl5310x_2.distance = pdata->RangeMilliMeter_2;//�������һ�β���������
	
    return status;
}


//������ͨ����
//dev���豸I2C�����ṹ��
//modeģʽ���� 0:Ĭ��;1:�߾���;2:������
void vl53l0x_general_start_2(VL53L0X_Dev_t_2 *dev,u8 mode)
{
	int a=0;
	static char buf[VL53L0X_MAX_STRING_LENGTH_2];	//����ģʽ�ַ����ַ�������
	VL53L0X_Error_2 Status=VL53L0X_ERROR_NONE_2;	//����״̬
	
	mode_string_2(mode,buf);						//��ʾ��ǰ���õ�ģʽ

	while(vl53l0x_set_mode_2(dev,mode));		//���þ���ģʽ
	vl53l0x_reset_2(dev);						//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	while(1)
	{
		 if(Status==VL53L0X_ERROR_NONE_2)
		 {
			a++;
			Status = vl53l0x_start_single_test_2(dev,&vl53l0x_data_2,buf);//ִ��һ�β���
//			printf("State;%i , %s\r\n",vl53l0x_data_2.RangeStatus_2,buf);//��ӡ����״̬	
			printf("d2: %36.1fcm\r\n",(vl5310x_2.distance)/10);//��ӡ��������
			delay_ms(1);
//			printf("%d\r\n",a);//��ӡ��������
		 }
	}
}


//vl53l0x��ͨ����ģʽ����
//dev:�豸I2C�����ṹ��
//modeģʽ���� 0:Ĭ��;1:�߾���;2:������;3:����
void vl53l0x_general_test_2(VL53L0X_Dev_t_2 *dev)
{
	while(1)
	{
		vl53l0x_general_start_2(dev,0);
	}
}
