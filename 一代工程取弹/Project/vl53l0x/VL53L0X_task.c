#include "VL53L0X_task.h"
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "vl53l0x.h"
#include "vl53l0x_2.h"



extern VL53L0X_Error Status;	//工作状态
extern  char buf[VL53L0X_MAX_STRING_LENGTH];	//测试模式字符串字符缓冲区
extern VL53L0X_Dev_t 		vl53l0x_dev;			//设备I2C1数据参数
	
void VL53L0X_task(void *pvParameters)
{
	//函数vTaskGetInfo()的使用
//	printf("1");
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount ();
	
	int VL5310X_MODE=3;							//mode:0-默认;1-高精度;2-长距离;3-高速
	//tof初始化函数
	VL53L0X_Dev_t 		vl53l0x_dev;			//设备I2C1数据参数
	char buf[VL53L0X_MAX_STRING_LENGTH];	//测试模式字符串字符缓冲区
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;	//工作状态
//	printf("1");
	while(vl53l0x_init(&vl53l0x_dev))//vl53l0x初始化
	{
//		 printf("VL53L0X Error!!!");
	}
//	printf("VL53L0X OK\r\n");
//	printf("1");
	mode_string(VL5310X_MODE,buf);							//显示当前配置的模式 0:默认;1:高精度;2:长距离;3:高速
	while(vl53l0x_set_mode(&vl53l0x_dev,VL5310X_MODE));	//配置精度模式
	vl53l0x_reset(&vl53l0x_dev);				//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	
	while(1)
	{
		 if(Status==VL53L0X_ERROR_NONE)		//40ms一次   < 15 没有物体 第一排22.6 -24.6 第二排 26-28
		 {
			Status = vl53l0x_start_single_test(&vl53l0x_dev,&vl53l0x_data,buf);//执行一次测量
//			printf("State;%i , %s\r\n",vl53l0x_data_2.RangeStatus_2,buf);//打印测量状态	
			vl5310x.distance =(int) (vl5310x.distance / 10);
			printf("	d11111111111111111111111 %8.1fcm		\r\n",vl5310x.distance);//打印测量距离	 
//			if (vl5310x.distance <= 15) 
//			{
//				vl5310x.distance = 80;
//			}
//			if (vl5310x.distance >=80)
//			{
//				vl5310x.distance = 80;
//			}
		 }
		 else
		 {
			vl5310x.distance = 0;
		 }
		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
}


void vl53l0x_2_task(void *pvParameters)
{
	//函数vTaskGetInfo()的使用
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 21;
	xLastWakeTime = xTaskGetTickCount ();
	
	int VL5310X_MODE_2=3;						//mode:0-默认;1-高精度;2-长距离;3-高速
	
	VL53L0X_Dev_t_2 	vl53l0x_dev_2;		//设备I2C2数据参数	
	char buf_2[VL53L0X_MAX_STRING_LENGTH_2];	//测试模式字符串字符缓冲区
	VL53L0X_Error_2 Status_2=VL53L0X_ERROR_NONE_2;	//工作状态
	
	while(vl53l0x_init_2(&vl53l0x_dev_2))//vl53l0x初始化
	{
//		 printf("VL53L0X_2 Error!!!");
	}
//	printf("VL53L0X_2 OK\r\n");
	
	mode_string_2(VL5310X_MODE_2,buf_2);						//显示当前配置的模式
	while(vl53l0x_set_mode_2(&vl53l0x_dev_2,VL5310X_MODE_2));		//配置精度模式
	vl53l0x_reset_2(&vl53l0x_dev_2);						//复位vl53l0x(频繁切换工作模式容易导致采集距离数据不准，需加上这一代码)
	
	while(1)
	{
		
		if(Status_2==VL53L0X_ERROR_NONE_2)
		{
			Status_2 = vl53l0x_start_single_test_2(&vl53l0x_dev_2,&vl53l0x_data_2,buf_2);//执行一次测量
//			printf("State;%i , %s\r\n",vl53l0x_data_2.RangeStatus_2,buf);//打印测量状态	
			vl5310x_2.distance = (int)(vl5310x_2.distance / 10);
//			printf("	d2222222222222: %8.1fcm		\r\n",vl5310x_2.distance);//打印测量距离
//			delay_ms(1);
		}
		else 
		{
			vl5310x_2.distance = 0;
		}
		 
		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
	
}
