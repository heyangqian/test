#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  
#include "task.h"
#include "vl53l0x_2.h"

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

VL53L0X_Dev_t_2 vl53l0x_dev_2;//�豸I2C���ݲ���
VL53L0X_DeviceInfo_t_2 vl53l0x_dev_2_info;//�豸ID�汾��Ϣ
uint8_t AjustOK_2=0;//У׼��־λ

//VL53L0X������ģʽ����
//0��Ĭ��;1:�߾���;2:������;3:����
mode_data_2 Mode_data_2[]=
{
    {(FixPoint1616_t_2)(0.25*65536), 
	 (FixPoint1616_t_2)(18*65536),
	 33000,
	 14,
	 10},//Ĭ��
		
	{(FixPoint1616_t_2)(0.25*65536) ,
	 (FixPoint1616_t_2)(18*65536),
	 200000, 
	 14,
	 10},//�߾���
		
    {(FixPoint1616_t_2)(0.1*65536) ,
	 (FixPoint1616_t_2)(60*65536),
	 33000,
	 18,
	 14},//������
	
    {(FixPoint1616_t_2)(0.25*65536) ,
	 (FixPoint1616_t_2)(32*65536),
	 20000,
	 14,
	 10},//����
		
};

//API������Ϣ��ӡ
//Status�����鿴VL53L0X_Error_2�����Ķ���
void print_pal_error_2(VL53L0X_Error_2 Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH_2];
	
	VL53L0X_GetPalErrorString_2(Status,buf);//����Status״̬��ȡ������Ϣ�ַ���
	
    printf("API Status: %i : %s\r\n",Status, buf);//��ӡ״̬�ʹ�����Ϣ
	
}

//ģʽ�ַ�����ʾ
//mode:0-Ĭ��;1-�߾���;2-������;3-����
void mode_string_2(u8 mode,char *buf)
{
	switch(mode)
	{
		case Default_Mode_2: strcpy(buf,"Default");        break;
		case HIGH_ACCURACY_2: strcpy(buf,"High Accuracy"); break;
		case LONG_RANGE_2: strcpy(buf,"Long Range");       break;
		case HIGH_SPEED_2: strcpy(buf,"High Speed");       break;
	}

}

//����VL53L0X�豸I2C��ַ
//dev:�豸I2C�����ṹ��
//newaddr:�豸��I2C��ַ
//VL53L0X_Error_2 vl53l0x_Addr_set_2(VL53L0X_Dev_t_2 *dev,uint8_t newaddr)
//{
//	uint16_t Id;
//	uint8_t FinalAddress;
//	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
//	u8 sta=0x00;
//	
//	FinalAddress = newaddr;
//	
//	if(FinalAddress==dev->I2cDevAddr)//���豸I2C��ַ��ɵ�ַһ��,ֱ���˳�
//		return VL53L0X_ERROR_NONE_2;
//	//�ڽ��е�һ���Ĵ�������֮ǰ����I2C��׼ģʽ(400Khz)
//	Status = VL53L0X_WrByte(dev,0x88,0x00);
//	if(Status!=VL53L0X_ERROR_NONE_2) 
//	{
//		sta=0x01;//����I2C��׼ģʽ����
//		goto set_error;
//	}
//	//����ʹ��Ĭ�ϵ�0x52��ַ��ȡһ���Ĵ���
//	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
//	if(Status!=VL53L0X_ERROR_NONE_2) 
//	{
//		sta=0x02;//��ȡ�Ĵ�������
//		goto set_error;
//	}
//	if(Id == 0xEEAA)
//	{
//		//�����豸�µ�I2C��ַ
//		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
//		if(Status!=VL53L0X_ERROR_NONE_2) 
//		{
//			sta=0x03;//����I2C��ַ����
//			goto set_error;
//		}
//		//�޸Ĳ����ṹ���I2C��ַ
//		dev->I2cDevAddr = FinalAddress;
//		//����µ�I2C��ַ��д�Ƿ�����
//		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
//		if(Status!=VL53L0X_ERROR_NONE_2) 
//		{
//			sta=0x04;//��I2C��ַ��д����
//			goto set_error;
//		}	
//	}
//	set_error:
//	if(Status!=VL53L0X_ERROR_NONE_2)
//	{
//		print_pal_error_2(Status);//��ӡ������Ϣ
//	}
//	if(sta!=0)
//	  printf("sta:0x%x\r\n",sta);
//	return Status;
//}

//vl53l0x��λ����
//dev:�豸I2C�����ṹ��
void vl53l0x_reset_2(VL53L0X_Dev_t_2 *dev)
{
//	uint8_t addr;
//	addr = dev->I2cDevAddr;//�����豸ԭI2C��ַ
    VL53L0X_Xshut_2=0;//ʧ��VL53L0X
	delay_ms(30);
	VL53L0X_Xshut_2=1;//ʹ��VL53L0X,�ô��������ڹ���(I2C��ַ��ָ�Ĭ��0X52)
	delay_ms(30);	
	dev->I2cDevAddr=0x52;
//	vl53l0x_Addr_set(dev,addr);//����VL53L0X������ԭ���ϵ�ǰԭI2C��ַ
	VL53L0X_DataInit_2(dev);	
}

 TaskHandle_t Vl53l0X_2_Handler;



//��ʼ��vl53l0x
//dev:�豸I2C�����ṹ��
VL53L0X_Error_2 vl53l0x_init_2(VL53L0X_Dev_t_2 *dev)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_Dev_t_2 *pMyDevice = dev;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//��ʹ������IO PORTCʱ�� 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
		
	pMyDevice->I2cDevAddr = VL53L0X_Addr_2;//I2C��ַ(�ϵ�Ĭ��0x52)
	pMyDevice->comms_type = 1;           //I2Cͨ��ģʽ
	pMyDevice->comms_speed_khz = 400;    //I2Cͨ������
	
	VL53L0X_i2c_init_2();//��ʼ��IIC����
	
	VL53L0X_Xshut_2=0;//ʧ��VL53L0X
	delay_ms(30);
	VL53L0X_Xshut_2=1;//ʹ��VL53L0X,�ô��������ڹ���
	delay_ms(30);
	
//	vl53l0x_Addr_set(pMyDevice,0x54);//����VL53L0X������I2C��ַ
    if(Status!=VL53L0X_ERROR_NONE_2) goto error;
	Status = VL53L0X_DataInit_2(pMyDevice);//�豸��ʼ��
//	printf("111111111111");
	delay_ms(2);
	Status = VL53L0X_GetDeviceInfo_2(pMyDevice,&vl53l0x_dev_2_info);//��ȡ�豸ID��Ϣ
    if(Status!=VL53L0X_ERROR_NONE_2) goto error;
//	printf("333333333333");
//	AT24CXX_Read(0,(u8*)&Vl53l0x_data_2,sizeof(_vl53l0x_adjust));//��ȡ24c02�����У׼����,����У׼ Vl53l0x_data_2.adjustok==0xAA
//	STMFLASH_Read(0,(u32*)&Vl53l0x_data_2,sizeof(_vl53l0x_adjust));
	if(Vl53l0x_data_2.adjustok==0xAA)//��У׼ 
	  AjustOK_2=1;	
	else //ûУ׼	
	  AjustOK_2=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE_2)
	{
		vl5310x_2.distance  = 0;
		vTaskDelete(Vl53l0X_2_Handler);
		print_pal_error_2(Status);//��ӡ������Ϣ
		return Status;
	}
	return Status;
}


//VL53L0X�����Գ���
void vl53l0x_test_2(void)
{
	while(vl53l0x_init_2(&vl53l0x_dev_2))//vl53l0x��ʼ��
	{
		 printf("VL53L0X Error!!!");
	}
	printf("VL53L0X OK\r\n");
	while(1)
	{
		 vl53l0x_info_2();
//		 vl53l0x_calibration_test_2(&vl53l0x_dev_2);          //У׼ģʽ
		 vl53l0x_general_test_2(&vl53l0x_dev_2);              //��ͨ����ģʽ
//		 vl53l0x_interrupt_test_2(&vl53l0x_dev_2);            //�жϲ���ģʽ  
	}
}

//----------���º���ΪUSMART����------------//

//��ȡvl53l0x������ID��Ϣ
void vl53l0x_info_2(void)
{
	printf("\r\n-------vl53l0x�������豸��Ϣ-------\r\n\r\n");
	printf("  Name:%s\r\n",vl53l0x_dev_2_info.Name);
	printf("  Addr:0x%x\r\n",vl53l0x_dev_2.I2cDevAddr);
	printf("  ProductId:%s\r\n",vl53l0x_dev_2_info.ProductId);
	printf("  RevisionMajor:0x%x\r\n",vl53l0x_dev_2_info.ProductRevisionMajor_2);
	printf("  RevisionMinor:0x%x\r\n",vl53l0x_dev_2_info.ProductRevisionMinor_2);
	printf("\r\n-----------------------------------\r\n");
}

//��ȡһ�β�����������
//modeģʽ���� 0:Ĭ��;1:�߾���;2:������;3:����
void One_measurement_2(u8 mode)
{
	vl53l0x_set_mode_2(&vl53l0x_dev_2,mode);
	VL53L0X_PerformSingleRangingMeasurement_2(&vl53l0x_dev_2,&vl53l0x_data_2);
	printf("\r\n d: %4d mm.\r\n",vl53l0x_data_2.RangeMilliMeter_2);
		
}
