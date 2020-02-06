#include "VL53L0X_task.h"
#include "FreeRTOS.h"					//֧��OSʱ��ʹ��	  
#include "task.h"
#include "vl53l0x.h"
#include "vl53l0x_2.h"



extern VL53L0X_Error Status;	//����״̬
extern  char buf[VL53L0X_MAX_STRING_LENGTH];	//����ģʽ�ַ����ַ�������
extern VL53L0X_Dev_t 		vl53l0x_dev;			//�豸I2C1���ݲ���
	
void VL53L0X_task(void *pvParameters)
{
	//����vTaskGetInfo()��ʹ��
//	printf("1");
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 20;
	xLastWakeTime = xTaskGetTickCount ();
	
	int VL5310X_MODE=3;							//mode:0-Ĭ��;1-�߾���;2-������;3-����
	//tof��ʼ������
	VL53L0X_Dev_t 		vl53l0x_dev;			//�豸I2C1���ݲ���
	char buf[VL53L0X_MAX_STRING_LENGTH];	//����ģʽ�ַ����ַ�������
	VL53L0X_Error Status=VL53L0X_ERROR_NONE;	//����״̬
//	printf("1");
	while(vl53l0x_init(&vl53l0x_dev))//vl53l0x��ʼ��
	{
//		 printf("VL53L0X Error!!!");
	}
//	printf("VL53L0X OK\r\n");
//	printf("1");
	mode_string(VL5310X_MODE,buf);							//��ʾ��ǰ���õ�ģʽ 0:Ĭ��;1:�߾���;2:������;3:����
	while(vl53l0x_set_mode(&vl53l0x_dev,VL5310X_MODE));	//���þ���ģʽ
	vl53l0x_reset(&vl53l0x_dev);				//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	
	while(1)
	{
		 if(Status==VL53L0X_ERROR_NONE)		//40msһ��   < 15 û������ ��һ��22.6 -24.6 �ڶ��� 26-28
		 {
			Status = vl53l0x_start_single_test(&vl53l0x_dev,&vl53l0x_data,buf);//ִ��һ�β���
//			printf("State;%i , %s\r\n",vl53l0x_data_2.RangeStatus_2,buf);//��ӡ����״̬	
			vl5310x.distance =(int) (vl5310x.distance / 10);
			printf("	d11111111111111111111111 %8.1fcm		\r\n",vl5310x.distance);//��ӡ��������	 
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
	//����vTaskGetInfo()��ʹ��
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 21;
	xLastWakeTime = xTaskGetTickCount ();
	
	int VL5310X_MODE_2=3;						//mode:0-Ĭ��;1-�߾���;2-������;3-����
	
	VL53L0X_Dev_t_2 	vl53l0x_dev_2;		//�豸I2C2���ݲ���	
	char buf_2[VL53L0X_MAX_STRING_LENGTH_2];	//����ģʽ�ַ����ַ�������
	VL53L0X_Error_2 Status_2=VL53L0X_ERROR_NONE_2;	//����״̬
	
	while(vl53l0x_init_2(&vl53l0x_dev_2))//vl53l0x��ʼ��
	{
//		 printf("VL53L0X_2 Error!!!");
	}
//	printf("VL53L0X_2 OK\r\n");
	
	mode_string_2(VL5310X_MODE_2,buf_2);						//��ʾ��ǰ���õ�ģʽ
	while(vl53l0x_set_mode_2(&vl53l0x_dev_2,VL5310X_MODE_2));		//���þ���ģʽ
	vl53l0x_reset_2(&vl53l0x_dev_2);						//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	
	while(1)
	{
		
		if(Status_2==VL53L0X_ERROR_NONE_2)
		{
			Status_2 = vl53l0x_start_single_test_2(&vl53l0x_dev_2,&vl53l0x_data_2,buf_2);//ִ��һ�β���
//			printf("State;%i , %s\r\n",vl53l0x_data_2.RangeStatus_2,buf);//��ӡ����״̬	
			vl5310x_2.distance = (int)(vl5310x_2.distance / 10);
//			printf("	d2222222222222: %8.1fcm		\r\n",vl5310x_2.distance);//��ӡ��������
//			delay_ms(1);
		}
		else 
		{
			vl5310x_2.distance = 0;
		}
		 
		vTaskDelayUntil( &xLastWakeTime, xFrequency);
	}
	
}
