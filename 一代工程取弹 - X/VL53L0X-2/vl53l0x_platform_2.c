#include "vl53l0x_platform_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X 底层驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//VL53L0X连续写数据
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
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

//VL53L0X连续读数据
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
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

//VL53L0X 写单字节寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:失败
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

//VL53L0X 写字（2字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:失败
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

//VL53L0X 写双字（4字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:失败
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

//VL53L0X 威胁安全更新(读/修改/写)单字节寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//AndData:8位与数据
//OrData:8位或数据
//返回值: 0:成功  
//       其他:错误
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

//VL53L0X 读单字节寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
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

//VL53L0X 读字（2字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
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

//VL53L0X 读双字（4字节）寄存器
//Dev:设备I2C参数结构体
//index:偏移地址
//pdata:数据指针
//count:长度
//返回值: 0:成功  
//       其他:错误
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


//VL53L0X 底层延时函数
//Dev:设备I2C参数结构体
//返回值: 0:成功  
//       其他:错误
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

