#include "vl53l0x_it_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ̽����STM32F407������
//VL53L0X-�жϲ���ģʽ ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//�����޾���ֵ ��λ:mm
#define Thresh_Low_2  60
#define Thresh_High_2 150

//�ж�ģʽ�����ṹ��
typedef struct 
{
     const int VL53L0X_Mode;//ģʽ
	 uint32_t ThreshLow;    //����ֵ
	 uint32_t ThreshHigh;   //����ֵ
}AlrmMode_t_2; 

AlrmMode_t_2 AlarmModes_2 ={
	
     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT_2,// value < thresh_low OR value > thresh_high
	 Thresh_Low_2<<16,
	 Thresh_High_2<<16
};

//�ж����ó�ʼ��
static void exti_init_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOFʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);//PF6 ���ӵ��ж���6
	
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE6
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش��� 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE6
    EXTI_Init(&EXTI_InitStructure);//����
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�6
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
    NVIC_Init(&NVIC_InitStructure);
	
}

//������־λ alarm_flag_2 
//1:�о���
//0����
u8 alarm_flag_2=0;

//�ⲿ�жϷ�����
//void EXTI9_5_IRQHandler(void)
//{
//	alarm_flag_2=1;//��־
//	EXTI_ClearITPendingBit(EXTI_Line7);  //���LINE6�ϵ��жϱ�־λ 
//}

extern uint8_t AjustOK_2;
extern mode_data_2 Mode_data_2[];

//vl53l0x�жϲ���ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������;3:����
void vl53l0x_interrupt_start_2(VL53L0X_Dev_t_2 *dev,uint8_t mode)
{
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	 VL53L0X_RangingMeasurementData_t_2 RangingMeasurementData;
//	 static char buf[VL53L0X_MAX_STRING_LENGTH];//����ģʽ�ַ����ַ�������
	 VL53L0X_Error_2 status=VL53L0X_ERROR_NONE_2;//����״̬


	 exti_init_2();//�жϳ�ʼ��
	printf("\r\n888888\r\n");
     vl53l0x_reset_2(dev);//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	 status = VL53L0X_StaticInit_2(dev);
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;

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
	 }else
	 {
	 	status = VL53L0X_PerformRefCalibration_2(dev, &VhvSettings, &PhaseCal);//Ref�ο�У׼
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement_2(dev, &refSpadCount, &isApertureSpads);//ִ�вο�SPAD����
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
        delay_ms(2);
		 
	 }

	 status = VL53L0X_SetDeviceMode_2(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2);//ʹ����������ģʽ
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetInterMeasurementPeriodMilliSeconds_2(dev,Mode_data_2[mode].timingBudget);//�����ڲ����ڲ���ʱ��
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
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_StopMeasurement_2(dev);//ֹͣ����
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetInterruptThresholds_2(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2,AlarmModes_2.ThreshLow, AlarmModes_2.ThreshHigh);//�趨�����ж��ϡ�����ֵ
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetGpioConfig_2(dev,0,VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,AlarmModes_2.VL53L0X_Mode,VL53L0X_INTERRUPTPOLARITY_LOW_2);//�趨�����ж�ģʽ �½���
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 	 	 	

	 status = VL53L0X_ClearInterruptMask_2(dev,0);//���VL53L0X�жϱ�־λ

	 error://������Ϣ
	 if(status!=VL53L0X_ERROR_NONE_2)
	 {
		print_pal_error_2(status);
		return ;
	 }

	 alarm_flag_2 = 0;
	 VL53L0X_StartMeasurement_2(dev);//��������
	 while(1)
	 {
//		printf("\r\n999999\r\n");
		if(alarm_flag_2==1)//�����ж�
		{
			alarm_flag_2=0;
			VL53L0X_GetRangingMeasurementData_2(dev,&RangingMeasurementData);//��ȡ��������,������ʾ����
			printf("d: %3d mm\r\n",RangingMeasurementData.RangeMilliMeter_2);
			
			delay_ms(70);
			VL53L0X_ClearInterruptMask_2(dev,0);//���VL53L0X�жϱ�־λ 
			
		}
		delay_ms(30);


	 }
		
}


//vl53l0x�жϲ���ģʽ����
//dev:�豸I2C�����ṹ��
void vl53l0x_interrupt_test_2(VL53L0X_Dev_t_2 *dev)
{
	while(1)
	{
		
		vl53l0x_interrupt_start_2(dev,1);
	       
	}
	
}
