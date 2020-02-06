#include "start.h"
#include "main.h"
#include "chassis.h"
#include "gas.h"
#include "motor.h"
#include "VL53L0X_task.h"

	/*FreeRTOSϵͳ*/

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  
#include "task.h"
#include "semphr.h"  
float INA226_voltageVal[4];
float INA226_currentVal[4];
float INA226_powerVal[4];


/*******************************************  ����ʼ����  ****************************************/

#define START_TASK_PRIO			1			//�������ȼ�
#define START_STK_SIZE 			128  		//�����ջ��С	
TaskHandle_t StartTask_Handler;				//������

/*******************************************  ������  ********************************************/
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

#define VL53L0X_TASK_PRIO			5		//�������ȼ�
#define VL53L0X_STK_SIZE 			400  		//�����ջ��С	
extern TaskHandle_t VL53L0X_Handler;		

#define Vl53l0X_2_TASK_PRIO			5
#define Vl53l0X_2_STK_SIZE 			400
extern TaskHandle_t Vl53l0X_2_Handler;
/****************************************** ��ȡ������Ϣ����   ***************************************/
#define QUERY_TASK_PRIO			4
#define QUERY_STK_SIZE 			256  
TaskHandle_t QueryTask_Handler;


int temp;


char RunTimeInfo[400];		          		//������������ʱ����Ϣ
SemaphoreHandle_t BinarySemaphore,Pan_control_task; 		//��ֵ�ź������
//SemaphoreHandle_t BinarySemaphore;  		//��ֵ�ź������
//QueueHandle_t   Key_Queue;				//��������
//QueueHandle_t   Message_Queue;			//��Ϣ����
//#define			KEYMSG_Q_NUM	1		//���д洢����С
//#define			MESSAGE_Q_NUM	4		//���д洢����С


/*��ʼ����������*/
void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();           //�����ٽ���
	
	//////////////////////////////	�����������Ĵ���	//////////////////////////////
	
	xTaskCreate((TaskFunction_t )Gas_task,     	//������
        (const char*    )"Gas_task",          //��������
        (uint16_t       )GAS_STK_SIZE,        //�����ջ��С
        (void*          )NULL,                  //���ݸ��������Ĳ���
        (UBaseType_t    )GAS_TASK_PRIO,       //�������ȼ�
        (TaskHandle_t*  )&Gas_Task_Handler);   //������  
	xTaskCreate((TaskFunction_t )Motor_task,     	//������
        (const char*    )"Motor_task",          //��������
        (uint16_t       )MOTOR_STK_SIZE,        //�����ջ��С
        (void*          )NULL,                  //���ݸ��������Ĳ���
        (UBaseType_t    )MOTOR_TASK_PRIO,       //�������ȼ�
        (TaskHandle_t*  )&Motor_Task_Handler);   //������  
	xTaskCreate((TaskFunction_t )Chassis_task,     	//������
        (const char*    )"Chassis_task",          //��������
        (uint16_t       )CHASSIS_STK_SIZE,        //�����ջ��С
        (void*          )NULL,                  //���ݸ��������Ĳ���
        (UBaseType_t    )CHASSIS_TASK_PRIO,       //�������ȼ�
        (TaskHandle_t*  )&Chassis_Task_Handler);   //������  
//	xTaskCreate((TaskFunction_t )Printf_task,     	//������
//        (const char*    )"Printf_task",          //��������
//        (uint16_t       )PRINTF_STK_SIZE,        //�����ջ��С
//        (void*          )NULL,                  //���ݸ��������Ĳ���
//        (UBaseType_t    )PRINTF_TASK_PRIO,       //�������ȼ�
//        (TaskHandle_t*  )&Printf_Task_Handler);   //������  
//	xTaskCreate((TaskFunction_t )VL53L0X_task,     
//		(const char*    )"VL53L0X_task",   				//TOF����//
//		(uint16_t       )VL53L0X_STK_SIZE,
//		(void*          )NULL,
//		(UBaseType_t    )VL53L0X_TASK_PRIO,
//		(TaskHandle_t*  )&VL53L0X_Handler); 
//		
//	xTaskCreate((TaskFunction_t )vl53l0x_2_task,     
//		(const char*    )"vl53l0x_2_task",   			//tof��ຯ��				//���Գɹ�
//		(uint16_t       )Vl53l0X_2_STK_SIZE,
//		(void*          )NULL,
//		(UBaseType_t    )Vl53l0X_2_TASK_PRIO,
//		(TaskHandle_t*  )&Vl53l0X_2_Handler); 
		
//		xTaskCreate((TaskFunction_t )Prin/

    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}


void startTast(void)
{  

    BinarySemaphore = xSemaphoreCreateBinary();	//������ֵ�ź���
    Pan_control_task = xSemaphoreCreateBinary();	//������ֵ�ź���
  
    xTaskCreate((TaskFunction_t )start_task,     	//������
        (const char*    )"start_task",          //��������
        (uint16_t       )START_STK_SIZE,        //�����ջ��С
        (void*          )NULL,                  //���ݸ��������Ĳ���
        (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
        (TaskHandle_t*  )&StartTask_Handler);   //������    
	
}






























/*******************************************************	���� ���� ����Ϊ ����������Ҫʱ���Ե���		******************************************************/

//�жϲ���������
/*
	�ú�����Ŀ���ǣ����ڴ򿪺͹ر� ��ѡ �жϣ���"FreertosConfig.h"����ļ���
	���С�configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY��
	����궨�����ϵͳ�����������ж����ȼ���
*/
void interrupt_task(void *pvParameters)
{
	static u32 total_num=0;
    while(1)
    {
		total_num+=1;
		if(total_num==5) 
		{
			printf("�ر��ж�.............\r\n");
			portDISABLE_INTERRUPTS();				//�ر��жϺ���
			
			delay_xms(5000);						//��ʱ5s
			
			printf("���ж�.............\r\n");		
			portENABLE_INTERRUPTS();				//���жϺ���
		}
        vTaskDelay(1000);
    }
}



/*��ȡ������Ϣ������  ��Ҫ  ���ڻ�ȡʣ����ջ��С���Դ˾�������ʵ�������ջ��С*/
void query_task(void *pvParameters)
{
	//����vTaskGetInfo()��ʹ��
	TaskHandle_t TaskHandle;	
	TaskStatus_t TaskStatus;
	
	printf("/************  ����vTaskGetInfo()��ʹ��  **************/\r\n");
	
	TaskHandle=xTaskGetHandle("vl53l0x_task");										//������������ȡ��������
	
	//��ȡks103_task��������Ϣ
	vTaskGetInfo((TaskHandle_t	)TaskHandle, 										//������
				 (TaskStatus_t*	)&TaskStatus, 										//������Ϣ�ṹ��
				 (BaseType_t	)pdTRUE,											//����ͳ�������ջ��ʷ��Сʣ���С
			     (eTaskState	)eInvalid);											//�����Լ���ȡ��������׳̬
	//ͨ�����ڴ�ӡ��ָ��������й���Ϣ��
	printf("������:                %s\r\n",TaskStatus.pcTaskName);
	printf("������:              %d\r\n",(int)TaskStatus.xTaskNumber);
	printf("����׳̬:              %d\r\n",TaskStatus.eCurrentState);
	printf("����ǰ���ȼ�:        %d\r\n",(int)TaskStatus.uxCurrentPriority);
	printf("��������ȼ�:          %d\r\n",(int)TaskStatus.uxBasePriority);
	printf("�����ջ����ַ:        %#x\r\n",(int)TaskStatus.pxStackBase);
	printf("�����ջ��ʷʣ����Сֵ:%d\r\n",TaskStatus.usStackHighWaterMark);
	printf("/**************************����***************************/\r\n");
	
}




/****************************************	��������		********************************************************/
	/*		vTaskSuspend(Task1Task_Handler);//��������1		*/
	/*		vTaskResume(Task1Task_Handler);	//�ָ�����1		*/
	/*		xTaskResumeFromISR(Vl53l0X_Task_Handler);//�ָ�����2  (�ж��еĻָ�)		*/



		
/************************************		�ź�������	******************************************************************/			
//����һ����ֵ�ź���
 /*   if(BinarySemaphore != NULL)
			{
				xSemaphoreGive(BinarySemaphore );        //�������з���
				xSemaphoreGiveFromISR(BinarySemaphore ); //���ж��з���
				
			}*/
  
//��ȡһ���ź���			
	/*	xSemaphoreTake(BinarySemaphore,portMAX_DELAY); //��ȡ�ź���������  ���������л�ȡ
			xSemaphoreTakeFromISR(BinarySemaphore,portMAX_DELAY);//���ж��л�ȡ   */
/***********************************************************************************************************************/




/************************************		���к���		*******************************************************************/  
// ����
/*һ��������
if((Key_Queue != NULL)&& (key)) //������Ч��������Ч
			{
					err=xQueueOverwrite(Key_Queue,&key);      //������з�����Ϣ
                             �������   ���͵���Ϣ
			}
		 
	�ж���	 
	BaseType_t err;
	BaseType_t xHigherPriorityTaskWoken;
	
	//����з�������
	if((Message_Queue != NULL) &&(USART_RX_STA&0x8000))
	{
		err=xQueueSendFromISR(Message_Queue,USART_RX_BUF,&xHigherPriorityTaskWoken);
		if(err != NULL)
		{
			printf("Send Failed!\r\n");
		}
		USART_RX_STA=0;                                        //��ձ�־00��
		memset(USART_RX_BUF,0,USART_REC_LEN);                 //��ջ�����
		
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);    //�����������л�
	}
		 
*/

//����
/*  

	һ��������
		 if(Key_Queue!=NULL)
		{                                     �ȴ�ʱ��
			err=xQueueReceive(Key_Queue,&key,portMAX_DELAY);  //�Ӷ����н�����Ϣ
			if(err==pdTRUE) 																	//��ȡ��Ϣ�ɹ�
			{
				printf("key value=%d\r\n",key);
			}
		}
		
		
	�ж���	
	BaseType_t err;
	BaseType_t xTaskWoken=pdFALSE;   
		if(Message_Queue != NULL)
		{
			err=xQueueReceiveFromISR(Message_Queue,ReceiveBuf,&xTaskWoken);
			if(err == pdTRUE)                                    �Ƿ���������л���ֻ���ṩһ�����������棬����ֵΪPdTRUEʱ���л�
			{
				printf("Receve:%s\r\n",ReceiveBuf);
				memset(ReceiveBuf,0,USART_REC_LEN);
			}
			else
			{
				printf("Receve Failed!\r\n");
			}
		}
		portYIELD_FROM_ISR(xTaskWoken);   //�����������л�
*/



