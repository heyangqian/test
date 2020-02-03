#include "vl53l0x_platform_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ̽����STM32F407������
//VL53L0X �ײ����� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//VL53L0X����д����
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:����
VL53L0X_Error_2 VL53L0X_WriteMulti_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t *pdata,uint32_t count)
{
	
	 VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	
	 int32_t status_int = 0;
	
	 uint8_t deviceAddress;
	
	 if(count >=VL53L0X_MAX_I2C_XFER_SIZE_2)
	 {
		 Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	 }
	
	 deviceAddress = Dev->I2cDevAddr;
	
	 status_int = VL53L0X_write_multi_2(deviceAddress, index, pdata, count);
	
	 if(status_int !=0)
	   Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;
	
	 return Status;
	
}

//VL53L0X����������
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:����
VL53L0X_Error_2 VL53L0X_ReadMulti_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t *pdata,uint32_t count)
{
	
	 VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	
	 int32_t status_int;
	
	 uint8_t deviceAddress;
	
	 if(count >=VL53L0X_MAX_I2C_XFER_SIZE_2)
	 {
		 Status = VL53L0X_ERROR_INVALID_PARAMS_2;
	 } 
	 
	 deviceAddress = Dev->I2cDevAddr;
	 
	 status_int = VL53L0X_read_multi_2(deviceAddress, index, pdata, count);
	 
	 if(status_int!=0)
	   Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;
	 
	 return Status;
	
}

//VL53L0X д���ֽڼĴ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:ʧ��
VL53L0X_Error_2 VL53L0X_WrByte_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t data)
{
	
	 VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	 int32_t status_int;
	 uint8_t deviceAddress;
	
	 deviceAddress = Dev->I2cDevAddr;
	
	 status_int = VL53L0X_write_byte_2(deviceAddress,index,data);
	
	 if(status_int!=0)
		Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;
	
	 return Status;
	 
}

//VL53L0X д�֣�2�ֽڣ��Ĵ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:ʧ��
VL53L0X_Error_2 VL53L0X_WrWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint16_t data)
{
	
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_word_2(deviceAddress,index,data);

	if(status_int!=0)
	 Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;

	return Status;
	
}

//VL53L0X д˫�֣�4�ֽڣ��Ĵ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:ʧ��
VL53L0X_Error_2 VL53L0X_WrDWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint32_t data)
{
	
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_write_dword_2(deviceAddress, index, data);

	if (status_int != 0)
	 Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;

	return Status;
	
}

//VL53L0X ��в��ȫ����(��/�޸�/д)���ֽڼĴ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//AndData:8λ������
//OrData:8λ������
//����ֵ: 0:�ɹ�  
//       ����:����
VL53L0X_Error_2 VL53L0X_UpdateByte_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
	 
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int32_t status_int;
	uint8_t deviceAddress;
	uint8_t data;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_byte_2(deviceAddress,index,&data);

	if(status_int!=0)
	  Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;

	if(Status == VL53L0X_ERROR_NONE_2)
	{
	  data = (data & AndData) | OrData;
	  status_int = VL53L0X_write_byte_2(deviceAddress, index,data);
	 
	 
	  if(status_int !=0)
		 Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;
	 
	}

	return Status;
	  
}

//VL53L0X �����ֽڼĴ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:����
VL53L0X_Error_2 VL53L0X_RdByte_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t *data)
{
	
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_byte_2(deviceAddress, index, data);

	if(status_int !=0)
	Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;

	return Status;

}

//VL53L0X ���֣�2�ֽڣ��Ĵ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:����
VL53L0X_Error_2 VL53L0X_RdWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint16_t *data)
{
	
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	int32_t status_int;
	uint8_t deviceAddress;

	deviceAddress = Dev->I2cDevAddr;

	status_int = VL53L0X_read_word_2(deviceAddress, index, data);

	if(status_int !=0)
	  Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;

	return Status;
	  
}

//VL53L0X ��˫�֣�4�ֽڣ��Ĵ���
//Dev:�豸I2C�����ṹ��
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:����
//����ֵ: 0:�ɹ�  
//       ����:����
VL53L0X_Error_2  VL53L0X_RdDWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint32_t *data)
{
	
    VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
    int32_t status_int;
    uint8_t deviceAddress;

    deviceAddress = Dev->I2cDevAddr;

    status_int = VL53L0X_read_dword_2(deviceAddress, index, data);

    if (status_int != 0)
        Status = VL53L0X_ERROR_CONTROL_INTERFACE_2;

    return Status;
}


//VL53L0X �ײ���ʱ����
//Dev:�豸I2C�����ṹ��
//����ֵ: 0:�ɹ�  
//       ����:����
#define VL53L0X_POLLINGDELAY_LOOPNB_2  250
VL53L0X_Error_2 VL53L0X_PollingDelay_2(VL53L0X_DEV_2 Dev)
{
	
    VL53L0X_Error_2 status = VL53L0X_ERROR_NONE_2;
    volatile uint32_t i;

    for(i=0;i<VL53L0X_POLLINGDELAY_LOOPNB_2;i++){
        //Do nothing
        
    }

    return status;
}

