#include "FreeRTOS_task.h"
#include "main.h"



//	vTaskSuspend(Task1Task_Handler);//挂起任务1			括号中为任务句柄
//	printf("挂起任务1的运行!\r\n");
			
//	vTaskResume(Task1Task_Handler);	//恢复任务1			括号中为任务句柄
//	printf("恢复任务1的运行!\r\n");
			

void FreeRTOS_Task()
{
	
	/*创建开始任务*/
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄              
    vTaskStartScheduler();          //开启任务调度
}
/*开始任务任务函数*/
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	

    //创建KS103任务
    xTaskCreate((TaskFunction_t )ks103_task,             
                (const char*    )"ks103_task",           
                (uint16_t       )KS103_STK_SIZE,                                                                                                               
                (void*          )NULL,                  
                (UBaseType_t    )KS103_TASK_PRIO,        
                (TaskHandle_t*  )&Task1Task_Handler);   
				
    //创建Vl53l0X任务
    xTaskCreate((TaskFunction_t )vl53l0x_task,     
                (const char*    )"vl53l0x_task",   
                (uint16_t       )Vl53l0X_STK_SIZE,
                (void*          )NULL,
                (UBaseType_t    )Vl53l0X_TASK_PRIO,
                (TaskHandle_t*  )&Task2Task_Handler); 
				
	//创建中断测试任务
    xTaskCreate((TaskFunction_t )interrupt_task,  			
                (const char*    )"interrupt_task", 			
                (uint16_t       )INTERRUPT_STK_SIZE,		
                (void*          )NULL,						
                (UBaseType_t    )INTERRUPT_TASK_PRIO,		
                (TaskHandle_t*  )&INTERRUPTTask_Handler); 	
				
	/*		创建QUERY任务						该函数用于读取任务信息					该函数开启式其他函数无法执行		*/													
//    xTaskCreate((TaskFunction_t )query_task,     
//                (const char*    )"query_task",   
//                (uint16_t       )QUERY_STK_SIZE,
//                (void*          )NULL,
//                (UBaseType_t    )QUERY_TASK_PRIO,
//                (TaskHandle_t*  )&QueryTask_Handler); 
	
	
    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}


//ks103任务函数
void ks103_task (void *pvParameters)
{
	int range;
	while(1)
	{
			/*	超声波初始化	*/
		KS103_WriteOneByte(0Xd0,0X02,0Xb0); 
		delay_ms(20);															//80
		range = KS103_ReadOneByte(0xd0, 0x02);
		range <<= 8;
		range += KS103_ReadOneByte(0xd0, 0x03);
		printf("距离：%d \r\n",range);
		delay_ms(20);						
	}
}

//vl53l0x任务函数
void vl53l0x_task(void *pvParameters)
{
	while(1)
	{
			vl53l0x_test();					//vl53l0x测试
	}
}

//中断测试任务函数
/*
	该函数的目的是：用于打开和关闭 所选 中断，打开"FreertosConfig.h"这个文件，
	其中“configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY”
	这个宏定义代表系统所管理的最高中断优先级。
*/
void interrupt_task(void *pvParameters)
{
	static u32 total_num=0;
    while(1)
    {
		total_num+=1;
		if(total_num==5) 
		{
			printf("关闭中断.............\r\n");
			portDISABLE_INTERRUPTS();				//关闭中断函数
			
			delay_xms(5000);						//延时5s
			
			printf("打开中断.............\r\n");		
			portENABLE_INTERRUPTS();				//打开中断函数
		}
        vTaskDelay(1000);
    }
}

/*读取任务信息函数，主要用于获取剩余推栈大小，以此决定最合适的任务推栈大小*/
void query_task(void *pvParameters)
{
	
	//函数vTaskGetInfo()的使用
	TaskHandle_t TaskHandle;	
	TaskStatus_t TaskStatus;
	
	printf("/************  函数vTaskGetInfo()的使用  **************/\r\n");
	
	TaskHandle=xTaskGetHandle("ks103_task");										//根据任务名获取任务句柄。
	
	//获取ks103_task的任务信息
	vTaskGetInfo((TaskHandle_t	)TaskHandle, 										//任务句柄
				 (TaskStatus_t*	)&TaskStatus, 										//任务信息结构体
				 (BaseType_t	)pdTRUE,											//允许统计任务堆栈历史最小剩余大小
			     (eTaskState	)eInvalid);											//函数自己获取任务运行壮态
	//通过串口打印出指定任务的有关信息。
	printf("任务名:                %s\r\n",TaskStatus.pcTaskName);
	printf("任务编号:              %d\r\n",(int)TaskStatus.xTaskNumber);
	printf("任务壮态:              %d\r\n",TaskStatus.eCurrentState);
	printf("任务当前优先级:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
	printf("任务基优先级:          %d\r\n",(int)TaskStatus.uxBasePriority);
	printf("任务堆栈基地址:        %#x\r\n",(int)TaskStatus.pxStackBase);
	printf("任务堆栈历史剩余最小值:%d\r\n",TaskStatus.usStackHighWaterMark);
	printf("/**************************结束***************************/\r\n");
	
}





