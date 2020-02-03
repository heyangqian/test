#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "vl53l0x_2.h"

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

VL53L0X_Dev_t_2 vl53l0x_dev_2;//设备I2C数据参数
VL53L0X_DeviceInfo_t_2 vl53l0x_dev_2_info;//设备ID版本信息
uint8_t AjustOK_2=0;//校准标志位

//VL53L0X各测量模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data_2 Mode_data_2[]=
{
    {(FixPoint1616_t_2)(0.25*65536), 
	 (FixPoint1616_t_2)(18*65536),
	 33000,
	 14,
	 10},//默认
		
	{(FixPoint1616_t_2)(0.25*65536) ,
	 (FixPoint1616_t_2)(18*65536),
	 200000, 
	 14,
	 10},//高精度
		
    {(FixPoint1616_t_2)(0.1*65536) ,
	 (FixPoint1616_t_2)(60*65536),
	 33000,
	 18,
	 14},//长距离
	
    {(FixPoint1616_t_2)(0.25*65536) ,
	 (FixPoint1616_t_2)(32*65536),
	 20000,
	 14,
	 10},//高速
		
};

//API错误信息打印
//Status：详情看VL53L0X_Error_2参数的定义
void print_pal_error_2(VL53L0X_Error_2 Status)
{
	
	char buf[VL53L0X_MAX_STRING_LENGTH_2];
	
	VL53L0X_GetPalErrorString_2(Status,buf);//根据Status状态获取错误信息字符串
	
    printf("API Status: %i : %s\r\n",Status, buf);//打印状态和错误信息
	
}

//模式字符串显示
//mode:0-默认;1-高精度;2-长距离;3-高速
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

//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
//VL53L0X_Error_2 vl53l0x_Addr_set_2(VL53L0X_Dev_t_2 *dev,uint8_t newaddr)
//{
//	uint16_t Id;
//	uint8_t FinalAddress;
//	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
//	u8 sta=0x00;
//	
//	FinalAddress = newaddr;
//	
//	if(FinalAddress==dev->I2cDevAddr)//新设备I2C地址与旧地址一致,直接退出
//		return VL53L0X_ERROR_NONE_2;
//	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
//	Status = VL53L0X_WrByte(dev,0x88,0x00);
//	if(Status!=VL53L0X_ERROR_NONE_2) 
//	{
//		sta=0x01;//设置I2C标准模式出错
//		goto set_error;
//	}
//	//尝试使用默认的0x52地址读取一个寄存器
//	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
//	if(Status!=VL53L0X_ERROR_NONE_2) 
//	{
//		sta=0x02;//读取寄存器出错
//		goto set_error;
//	}
//	if(Id == 0xEEAA)
//	{
//		//设置设备新的I2C地址
//		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
//		if(Status!=VL53L0X_ERROR_NONE_2) 
//		{
//			sta=0x03;//设置I2C地址出错
//			goto set_error;
//		}
//		//修改参数结构体的I2C地址
//		dev->I2cDevAddr = FinalAddress;
//		//检查新的I2C地址读写是否正常
//		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
//		if(Status!=VL53L0X_ERROR_NONE_2) 
//		{
//			sta=0x04;//新I2C地址读写出错
//			goto set_error;
//		}	
//	}
//	set_error:
//	if(Status!=VL53L0X_ERROR_NONE_2)
//	{
//		print_pal_error_2(Status);//打印错误信息
//	}
//	if(sta!=0)
//	  printf("sta:0x%x\r\n",sta);
//	return Status;
//}

//vl53l0x复位函数
//dev:设备I2C参数结构体
void vl53l0x_reset_2(VL53L0X_Dev_t_2 *dev)
{
//	uint8_t addr;
//	addr = dev->I2cDevAddr;//保存设备原I2C地址
    VL53L0X_Xshut_2=0;//失能VL53L0X
	delay_ms(30);
	VL53L0X_Xshut_2=1;//使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
	delay_ms(30);	
	dev->I2cDevAddr=0x52;
//	vl53l0x_Addr_set(dev,addr);//设置VL53L0X传感器原来上电前原I2C地址
	VL53L0X_DataInit_2(dev);	
}

 TaskHandle_t Vl53l0X_2_Handler;



//初始化vl53l0x
//dev:设备I2C参数结构体
VL53L0X_Error_2 vl53l0x_init_2(VL53L0X_Dev_t_2 *dev)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	VL53L0X_Error_2 Status = VL53L0X_ERROR_NONE_2;
	VL53L0X_Dev_t_2 *pMyDevice = dev;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//先使能外设IO PORTC时钟 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;//端口配置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化
		
	pMyDevice->I2cDevAddr = VL53L0X_Addr_2;//I2C地址(上电默认0x52)
	pMyDevice->comms_type = 1;           //I2C通信模式
	pMyDevice->comms_speed_khz = 400;    //I2C通信速率
	
	VL53L0X_i2c_init_2();//初始化IIC总线
	
	VL53L0X_Xshut_2=0;//失能VL53L0X
	delay_ms(30);
	VL53L0X_Xshut_2=1;//使能VL53L0X,让传感器处于工作
	delay_ms(30);
	
//	vl53l0x_Addr_set(pMyDevice,0x54);//设置VL53L0X传感器I2C地址
    if(Status!=VL53L0X_ERROR_NONE_2) goto error;
	Status = VL53L0X_DataInit_2(pMyDevice);//设备初始化
//	printf("111111111111");
	delay_ms(2);
	Status = VL53L0X_GetDeviceInfo_2(pMyDevice,&vl53l0x_dev_2_info);//获取设备ID信息
    if(Status!=VL53L0X_ERROR_NONE_2) goto error;
//	printf("333333333333");
//	AT24CXX_Read(0,(u8*)&Vl53l0x_data_2,sizeof(_vl53l0x_adjust));//读取24c02保存的校准数据,若已校准 Vl53l0x_data_2.adjustok==0xAA
//	STMFLASH_Read(0,(u32*)&Vl53l0x_data_2,sizeof(_vl53l0x_adjust));
	if(Vl53l0x_data_2.adjustok==0xAA)//已校准 
	  AjustOK_2=1;	
	else //没校准	
	  AjustOK_2=0;
	
	error:
	if(Status!=VL53L0X_ERROR_NONE_2)
	{
		vl5310x_2.distance  = 0;
		vTaskDelete(Vl53l0X_2_Handler);
		print_pal_error_2(Status);//打印错误信息
		return Status;
	}
	return Status;
}


//VL53L0X主测试程序
void vl53l0x_test_2(void)
{
	while(vl53l0x_init_2(&vl53l0x_dev_2))//vl53l0x初始化
	{
		 printf("VL53L0X Error!!!");
	}
	printf("VL53L0X OK\r\n");
	while(1)
	{
		 vl53l0x_info_2();
//		 vl53l0x_calibration_test_2(&vl53l0x_dev_2);          //校准模式
		 vl53l0x_general_test_2(&vl53l0x_dev_2);              //普通测量模式
//		 vl53l0x_interrupt_test_2(&vl53l0x_dev_2);            //中断测量模式  
	}
}

//----------以下函数为USMART调用------------//

//获取vl53l0x传感器ID信息
void vl53l0x_info_2(void)
{
	printf("\r\n-------vl53l0x传感器设备信息-------\r\n\r\n");
	printf("  Name:%s\r\n",vl53l0x_dev_2_info.Name);
	printf("  Addr:0x%x\r\n",vl53l0x_dev_2.I2cDevAddr);
	printf("  ProductId:%s\r\n",vl53l0x_dev_2_info.ProductId);
	printf("  RevisionMajor:0x%x\r\n",vl53l0x_dev_2_info.ProductRevisionMajor_2);
	printf("  RevisionMinor:0x%x\r\n",vl53l0x_dev_2_info.ProductRevisionMinor_2);
	printf("\r\n-----------------------------------\r\n");
}

//获取一次测量距离数据
//mode模式配置 0:默认;1:高精度;2:长距离;3:高速
void One_measurement_2(u8 mode)
{
	vl53l0x_set_mode_2(&vl53l0x_dev_2,mode);
	VL53L0X_PerformSingleRangingMeasurement_2(&vl53l0x_dev_2,&vl53l0x_data_2);
	printf("\r\n d: %4d mm.\r\n",vl53l0x_data_2.RangeMilliMeter_2);
		
}
