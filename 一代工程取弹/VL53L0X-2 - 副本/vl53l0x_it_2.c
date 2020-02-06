#include "vl53l0x_it_2.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK 探索者STM32F407开发板
//VL53L0X-中断测量模式 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2017/7/1
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

//上下限距离值 单位:mm
#define Thresh_Low_2  60
#define Thresh_High_2 150

//中断模式参数结构体
typedef struct 
{
     const int VL53L0X_Mode;//模式
	 uint32_t ThreshLow;    //下限值
	 uint32_t ThreshHigh;   //上限值
}AlrmMode_t_2; 

AlrmMode_t_2 AlarmModes_2 ={
	
     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT_2,// value < thresh_low OR value > thresh_high
	 Thresh_Low_2<<16,
	 Thresh_High_2<<16
};

//中断配置初始化
static void exti_init_2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOF时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输出模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);//PF6 连接到中断线6
	
    EXTI_InitStructure.EXTI_Line = EXTI_Line6;//LINE6
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE6
    EXTI_Init(&EXTI_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断6
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x03;//抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);
	
}

//警报标志位 alarm_flag_2 
//1:有警报
//0：无
u8 alarm_flag_2=0;

//外部中断服务函数
//void EXTI9_5_IRQHandler(void)
//{
//	alarm_flag_2=1;//标志
//	EXTI_ClearITPendingBit(EXTI_Line7);  //清除LINE6上的中断标志位 
//}

extern uint8_t AjustOK_2;
extern mode_data_2 Mode_data_2[];

//vl53l0x中断测量模式测试
//dev:设备I2C参数结构体
//mode: 0:默认;1:高精度;2:长距离;3:高速
void vl53l0x_interrupt_start_2(VL53L0X_Dev_t_2 *dev,uint8_t mode)
{
	 uint8_t VhvSettings;
	 uint8_t PhaseCal;
	 uint32_t refSpadCount;
	 uint8_t isApertureSpads;
	 VL53L0X_RangingMeasurementData_t_2 RangingMeasurementData;
//	 static char buf[VL53L0X_MAX_STRING_LENGTH];//测试模式字符串字符缓冲区
	 VL53L0X_Error_2 status=VL53L0X_ERROR_NONE_2;//工作状态


	 exti_init_2();//中断初始化
	printf("\r\n888888\r\n");
     vl53l0x_reset_2(dev);//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	 status = VL53L0X_StaticInit_2(dev);
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;

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
	 }else
	 {
	 	status = VL53L0X_PerformRefCalibration_2(dev, &VhvSettings, &PhaseCal);//Ref参考校准
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement_2(dev, &refSpadCount, &isApertureSpads);//执行参考SPAD管理
		if(status!=VL53L0X_ERROR_NONE_2) goto error;
        delay_ms(2);
		 
	 }

	 status = VL53L0X_SetDeviceMode_2(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2);//使能连续测量模式
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetInterMeasurementPeriodMilliSeconds_2(dev,Mode_data_2[mode].timingBudget);//设置内部周期测量时间
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
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_StopMeasurement_2(dev);//停止测量
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetInterruptThresholds_2(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING_2,AlarmModes_2.ThreshLow, AlarmModes_2.ThreshHigh);//设定触发中断上、下限值
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetGpioConfig_2(dev,0,VL53L0X_VCSEL_PERIOD_PRE_RANGE_2,AlarmModes_2.VL53L0X_Mode,VL53L0X_INTERRUPTPOLARITY_LOW_2);//设定触发中断模式 下降沿
	 if(status!=VL53L0X_ERROR_NONE_2) goto error;
	 delay_ms(2);
	 	 	 	

	 status = VL53L0X_ClearInterruptMask_2(dev,0);//清除VL53L0X中断标志位

	 error://错误信息
	 if(status!=VL53L0X_ERROR_NONE_2)
	 {
		print_pal_error_2(status);
		return ;
	 }

	 alarm_flag_2 = 0;
	 VL53L0X_StartMeasurement_2(dev);//启动测量
	 while(1)
	 {
//		printf("\r\n999999\r\n");
		if(alarm_flag_2==1)//触发中断
		{
			alarm_flag_2=0;
			VL53L0X_GetRangingMeasurementData_2(dev,&RangingMeasurementData);//获取测量距离,并且显示距离
			printf("d: %3d mm\r\n",RangingMeasurementData.RangeMilliMeter_2);
			
			delay_ms(70);
			VL53L0X_ClearInterruptMask_2(dev,0);//清除VL53L0X中断标志位 
			
		}
		delay_ms(30);


	 }
		
}


//vl53l0x中断测量模式测试
//dev:设备I2C参数结构体
void vl53l0x_interrupt_test_2(VL53L0X_Dev_t_2 *dev)
{
	while(1)
	{
		
		vl53l0x_interrupt_start_2(dev,1);
	       
	}
	
}
