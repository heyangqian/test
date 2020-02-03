#ifndef __VL53L0X_PLATFORM_2_H
#define __VL53L0X_PLATFORM_2_H

#include "vl53l0x_def_2.h"
#include "vl53l0x_i2c_2.h"
#include "vl53l0x_platform_log_2.h"

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

//vl53l0x�豸I2C��Ϣ
typedef struct {
    VL53L0X_DevData_t_2 Data;              /*!< embed ST Ewok Dev  data as "Data"*/
							/*!< user specific field */
    uint8_t   I2cDevAddr;                /*!< i2c device address user specific field */
    uint8_t   comms_type;                /*!< Type of comms : VL53L0X_COMMS_I2C or VL53L0X_COMMS_SPI */
    uint16_t  comms_speed_khz;           /*!< Comms speed [kHz] : typically 400kHz for I2C           */

} VL53L0X_Dev_t_2;


typedef VL53L0X_Dev_t_2* VL53L0X_DEV_2;

#define VL53L0X_MAX_I2C_XFER_SIZE_2  64 //����I2Cд������ֽ���
#define PALDevDataGet_2(Dev, field) (Dev->Data.field)
#define PALDevDataSet_2(Dev, field, data) (Dev->Data.field)=(data)


VL53L0X_Error_2 VL53L0X_WriteMulti_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t *pdata,uint32_t count);
VL53L0X_Error_2 VL53L0X_ReadMulti_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t *pdata,uint32_t count);
VL53L0X_Error_2 VL53L0X_WrByte_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t data);
VL53L0X_Error_2 VL53L0X_WrWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint16_t data);
VL53L0X_Error_2 VL53L0X_WrDWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint32_t data);
VL53L0X_Error_2 VL53L0X_UpdateByte_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t AndData, uint8_t OrData);
VL53L0X_Error_2 VL53L0X_RdByte_2(VL53L0X_DEV_2 Dev, uint8_t index, uint8_t *data);
VL53L0X_Error_2 VL53L0X_RdWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint16_t *data);
VL53L0X_Error_2 VL53L0X_RdDWord_2(VL53L0X_DEV_2 Dev, uint8_t index, uint32_t *data);
VL53L0X_Error_2 VL53L0X_PollingDelay_2(VL53L0X_DEV_2 Dev);


#endif 

