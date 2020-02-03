#include "start.h"
#include "main.h"
#include "chassis.h"
#include "gas.h"
#include "motor.h"
#include "VL53L0X_task.h"

	/*FreeRTOS系统*/

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//支持OS时，使用	  
#include "task.h"
#include "semphr.h"  
float INA226_voltageVal[4];
float INA226_currentVal[4];
float INA226_powerVal[4];


/*******************************************  任务开始函数  ****************************************/

#define START_TASK_PRIO			1			//任务优先级
#define START_STK_SIZE 			128  		//任务堆栈大小	
TaskHandle_t StartTask_Handler;				//任务句柄

/*******************************************  任务函数  ********************************************/
#define CLOUD_TASK_PRIO		12
#define CLOUD_STK_SIZE 		256  
TaskHandle_t CLOUDTask_Handler;

//#define PRINTF_TASK_PRIO		8
//#define PRINTF_STK_SIZE 		100 
//TaskHandle_t PRINTFTask_Handler;

#define Auto_aim_TASK_PRIO		10
#define Auto_aim_STK_SIZE 		256 
TaskHandle_t Auto_aim_Task_Handler;

#define MinPc_TASK_PRIO		8
#define MinPc_STK_SIZE 		100 
TaskHandle_t MinPcTask_Handler;

#define CHASSIS_TASK_PRIO			9
#define CHASSIS_STK_SIZE 			256  
TaskHandle_t Chassis_Task_Handler;

#define MOTOR_TASK_PRIO			10
#define MOTOR_STK_SIZE 			256  
TaskHandle_t Motor_Task_Handler;

#define GAS_TASK_PRIO			11
#define GAS_STK_SIZE 			256  
TaskHandle_t Gas_Task_Handler;

#define PRINTF_TASK_PRIO		7
#define PRINTF_STK_SIZE 		256
TaskHandle_t Printf_Task_Handler;     

#define VL53L0X_TASK_PRIO			5		//任务优先级
#define VL53L0X_STK_SIZE 			400  		//任务堆栈大小	
extern TaskHandle_t VL53L0X_Handler;		

#define Vl53l0X_2_TASK_PRIO			5
#define Vl53l0X_2_STK_SIZE 			400
extern TaskHandle_t Vl53l0X_2_Handler;
/****************************************** 读取任务信息函数   ***************************************/
#define QUERY_TASK_PRIO			4
#define QUERY_STK_SIZE 			256  
TaskHandle_t QueryTask_Handler;


int temp;


char RunTimeInfo[400];		          		//保存任务运行时间信息
SemaphoreHandle_t BinarySemaphore,Pan_control_task; 		//二值信号量句柄
//SemaphoreHandle_t BinarySemaphore;  		//二值信号量句柄
//QueueHandle_t   Key_Queue;				//按键队列
//QueueHandle_t   Message_Queue;			//消息队列
//#define			KEYMSG_Q_NUM	1		//队列存储区大小
//#define			MESSAGE_Q_NUM	4		//队列存储区大小


/*开始任务任务函数*/
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //进入临界区
	
	//////////////////////////////	创建任务函数的代码	//////////////////////////////
	
	xTaskCreate((TaskFunction_t )Gas_task,     	//任务函数
        (const char*    )"Gas_task",          //任务名称
        (uint16_t       )GAS_STK_SIZE,        //任务堆栈大小
        (void*          )NULL,                  //传递给任务函数的参数
        (UBaseType_t    )GAS_TASK_PRIO,       //任务优先级
        (TaskHandle_t*  )&Gas_Task_Handler);   //任务句柄  
	xTaskCreate((TaskFunction_t )Motor_task,     	//任务函数
        (const char*    )"Motor_task",          //任务名称
        (uint16_t       )MOTOR_STK_SIZE,        //任务堆栈大小
        (void*          )NULL,                  //传递给任务函数的参数
        (UBaseType_t    )MOTOR_TASK_PRIO,       //任务优先级
        (TaskHandle_t*  )&Motor_Task_Handler);   //任务句柄  
	xTaskCreate((TaskFunction_t )Chassis_task,     	//任务函数
        (const char*    )"Chassis_task",          //任务名称
        (uint16_t       )CHASSIS_STK_SIZE,        //任务堆栈大小
        (void*          )NULL,                  //传递给任务函数的参数
        (UBaseType_t    )CHASSIS_TASK_PRIO,       //任务优先级
        (TaskHandle_t*  )&Chassis_Task_Handler);   //任务句柄  
//	xTaskCreate((TaskFunction_t )Printf_task,     	//任务函数
//        (const char*    )"Printf_task",          //任务名称
//        (uint16_t       )PRINTF_STK_SIZE,        //任务堆栈大小
//        (void*          )NULL,                  //传递给任务函数的参数
//        (UBaseType_t    )PRINTF_TASK_PRIO,       //任务优先级
//        (TaskHandle_t*  )&Printf_Task_Handler);   //任务句柄  
//	xTaskCreate((TaskFunction_t )VL53L0X_task,     
//		(const char*    )"VL53L0X_task",   				//TOF任务//
//		(uint16_t       )VL53L0X_STK_SIZE,
//		(void*          )NULL,
//		(UBaseType_t    )VL53L0X_TASK_PRIO,
//		(TaskHandle_t*  )&VL53L0X_Handler); 
//		
//	xTaskCreate((TaskFunction_t )vl53l0x_2_task,     
//		(const char*    )"vl53l0x_2_task",   			//tof测距函数				//测试成功
//		(uint16_t       )Vl53l0X_2_STK_SIZE,
//		(void*          )NULL,
//		(UBaseType_t    )Vl53l0X_2_TASK_PRIO,
//		(TaskHandle_t*  )&Vl53l0X_2_Handler); 
		
//		xTaskCreate((TaskFunction_t )Prin/

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}


void startTast(void)
{  

    BinarySemaphore = xSemaphoreCreateBinary();	//创建二值信号量
    Pan_control_task = xSemaphoreCreateBinary();	//创建二值信号量
  
    xTaskCreate((TaskFunction_t )start_task,     	//任务函数
        (const char*    )"start_task",          //任务名称
        (uint16_t       )START_STK_SIZE,        //任务堆栈大小
        (void*          )NULL,                  //传递给任务函数的参数
        (UBaseType_t    )START_TASK_PRIO,       //任务优先级
        (TaskHandle_t*  )&StartTask_Handler);   //任务句柄    
	
}






























/*******************************************************	以下 所有 函数为 任务函数，需要时可以调用		******************************************************/

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



/*读取任务信息函数，  主要  用于获取剩余推栈大小，以此决定最合适的任务推栈大小*/
void query_task(void *pvParameters)
{
	//函数vTaskGetInfo()的使用
	TaskHandle_t TaskHandle;	
	TaskStatus_t TaskStatus;
	
	printf("/************  函数vTaskGetInfo()的使用  **************/\r\n");
	
	TaskHandle=xTaskGetHandle("vl53l0x_task");										//根据任务名获取任务句柄。
	
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




/****************************************	函数挂起		********************************************************/
	/*		vTaskSuspend(Task1Task_Handler);//挂起任务1		*/
	/*		vTaskResume(Task1Task_Handler);	//恢复任务1		*/
	/*		xTaskResumeFromISR(Vl53l0X_Task_Handler);//恢复任务2  (中断中的恢复)		*/



		
/************************************		信号量函数	******************************************************************/			
//发送一个二值信号量
 /*   if(BinarySemaphore != NULL)
			{
				xSemaphoreGive(BinarySemaphore );        //在任务中发送
				xSemaphoreGiveFromISR(BinarySemaphore ); //在中断中发送
				
			}*/
  
//获取一个信号量			
	/*	xSemaphoreTake(BinarySemaphore,portMAX_DELAY); //获取信号量，死等  ，在任务中获取
			xSemaphoreTakeFromISR(BinarySemaphore,portMAX_DELAY);//在中断中获取   */
/***********************************************************************************************************************/




/************************************		队列函数		*******************************************************************/  
// 发送
/*一般任务中
if((Key_Queue != NULL)&& (key)) //队列有效，按键有效
			{
					err=xQueueOverwrite(Key_Queue,&key);      //向队列中发送消息
                             函数句柄   发送的消息
			}
		 
	中断中	 
	BaseType_t err;
	BaseType_t xHigherPriorityTaskWoken;
	
	//向队列发送数据
	if((Message_Queue != NULL) &&(USART_RX_STA&0x8000))
	{
		err=xQueueSendFromISR(Message_Queue,USART_RX_BUF,&xHigherPriorityTaskWoken);
		if(err != NULL)
		{
			printf("Send Failed!\r\n");
		}
		USART_RX_STA=0;                                        //清空标志00、
		memset(USART_RX_BUF,0,USART_REC_LEN);                 //清空缓存区
		
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    //进行上下文切换
	}
		 
*/

//接收
/*  

	一般任务中
		 if(Key_Queue!=NULL)
		{                                     等待时间
			err=xQueueReceive(Key_Queue,&key,portMAX_DELAY);  //从队列中接收消息
			if(err==pdTRUE) 																	//获取消息成功
			{
				printf("key value=%d\r\n",key);
			}
		}
		
		
	中断中	
	BaseType_t err;
	BaseType_t xTaskWoken=pdFALSE;   
		if(Message_Queue != NULL)
		{
			err=xQueueReceiveFromISR(Message_Queue,ReceiveBuf,&xTaskWoken);
			if(err == pdTRUE)                                    是否进行任务切换，只需提供一个变量来保存，当此值为PdTRUE时，切换
			{
				printf("Receve:%s\r\n",ReceiveBuf);
				memset(ReceiveBuf,0,USART_REC_LEN);
			}
			else
			{
				printf("Receve Failed!\r\n");
			}
		}
		portYIELD_FROM_ISR(xTaskWoken);   //进行上下文切换
*/



