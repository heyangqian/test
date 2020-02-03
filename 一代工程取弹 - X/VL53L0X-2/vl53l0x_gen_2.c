#include "vl53l0x_gen_2.h"


//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X-普通测量模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

VL53L0X_RangingMeasurementData_t_2 vl53l0x_data_2;//测距测量结构体
 VL5310X_2 vl5310x_2;

//VL53L0X 测量模式配置
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离
VL53L0X_Error_2 vl53l0x_set_mode_2(VL53L0X_Dev_t_2 *dev,u8 mode)
{
	
	 VL53L0X_Error_2 status = VL53L0X_ERROR_NONE_2;
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	
	// vl53l0x_reset_2(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit_2(dev);

     if(AjustOK_2!=0)//已校准好了,写入校准值
     {
	    status= VL53L0X_SetReferenceSpads_2(dev,Vl53l0x_data_2.refSpadCount,Vl53l0x_data_2.isApertureSpads);//设定Spads校准值
        if(status!=VL53L0X_ERROR_NONE_2) goto error;	
        delay_ms(2);		 
	    status= VL53L0X_SetRefCalibration_2(dev,Vl53l0x_data_2.VhvSettings,Vl53l0x_data_2.PhaseCal);//设定Ref校准值
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
		 delay_ms(2);
	    status=VL53L0X_SetOffsetCalibrationDataMicroMeter_2(dev,Vl53l0x_data_2.OffsetMicroMeter);//设定偏移校准值
		if(status!=VL53L0X_ERROR_NONE_2) goto error; 
		 delay_ms(2);
		status=VL53L0X_SetXTalkCompensationRateMegaCps_2(dev,Vl53l0x_data_2.XTalkCompensationRateMegaCps);//设定串扰校准值
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
         delay_ms(2);		 
		 
     }
	 else
	 {
		status = VL53L0X_PerformRefCalibration_2(dev, &VhvSettings, &PhaseCal);//Ref参考校准
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement_2(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
        delay_ms(2);		 	 
	 }
	 status = VL53L0X_SetDeviceMode_2(dev,VL53L0X_DEVICEMODE_SINGLE_RANGING_2);//使能单次测量模式
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable_2(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,1);//使能SIGMA范围检查
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable_2(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,1);//使能信号速率范围检查
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue_2(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE_2,Mode_data_2[mode].sigmaLimit);//设定SIGMA范围
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue_2(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE_2,Mode_data_2[mode].signalLimit);//设定信号速率范围范围
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds_2(dev,Mode_data_2[mode].timingBudget);//设定完整测距最长时间
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod_2(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE_2, Mode_data_2[mode].preRangeVcselPeriod);//设定VCSEL脉冲周期
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod_2(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE_2, Mode_data_2[mode].finalRangeVcselPeriod);//设定VCSEL脉冲周期范围
	 
	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE_2)
	 {
		print_pal_error_2(status);
		return status;
	 }
	 return status;
	
}	

//VL53L0X 单次距离测量函数
//dev:设备I2C参数结构体
//pdata:保存测量数据结构体
VL53L0X_Error_2 vl53l0x_start_single_test_2(VL53L0X_Dev_t_2 *dev,VL53L0X_RangingMeasurementData_t_2 *pdata,char *buf)
{
	VL53L0X_Error_2 status = VL53L0X_ERROR_NONE_2;
	uint8_t RangeStatus_2;
	
	status = VL53L0X_PerformSingleRangingMeasurement_2(dev, pdata);//执行单次测距并获取测距测量数据
	if(status !=VL53L0X_ERROR_NONE_2) return status;
   
	RangeStatus_2 = pdata->RangeStatus_2;//获取当前测量状态
    memset(buf,0x00,VL53L0X_MAX_STRING_LENGTH_2);
	VL53L0X_GetRangeStatusString_2(RangeStatus_2,buf);//根据测量状态读取状态字符串
	
	vl5310x_2.distance = pdata->RangeMilliMeter_2;//保存最近一次测距测量数据
	
    return status;
}


//启动普通测量
//dev：设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离
void vl53l0x_general_start_2(VL53L0X_Dev_t_2 *dev,u8 mode)
{
	int a=0;
	static char buf[VL53L0X_MAX_STRING_LENGTH_2];	//测试模式字符串字符缓冲区
	VL53L0X_Error_2 Status=VL53L0X_ERROR_NONE_2;	//工作状态
	
	mode_string_2(mode,buf);						//显示当前配置的模式

	while(vl53l0x_set_mode_2(dev,mode));		//配置精度模式
	vl53l0x_reset_2(dev);						//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	while(1)
	{
		 if(Status==VL53L0X_ERROR_NONE_2)
		 {
			a++;
			Status = vl53l0x_start_single_test_2(dev,&vl53l0x_data_2,buf);//执行一次测量
//			printf("State;%i , %s\r\n",vl53l0x_data_2.RangeStatus_2,buf);//打印测量状态	
			printf("d2: %36.1fcm\r\n",(vl5310x_2.distance)/10);//打印测量距离
			delay_ms(1);
//			printf("%d\r\n",a);//打印测量个数
		 }
	}
}


//vl53l0x普通测量模式测试
//dev:设备I2C参数结构体
//mode模式配置 0:默认;1:高精度;2:长距离;3:高速
void vl53l0x_general_test_2(VL53L0X_Dev_t_2 *dev)
{
	while(1)
	{
		vl53l0x_general_start_2(dev,0);
	}
}
