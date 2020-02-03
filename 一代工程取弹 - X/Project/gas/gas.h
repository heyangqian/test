#ifndef __GAS_H__
#define __GAS_H__
#include "main.h"
#include "pid.h"


#define G_MODE 3           // 1 ��ͨץ����������  2 ��ͨץ����һ����  3 ʮ��ץ��  4 x����ץ����������  5 x����ץ����һ����

//#define  MINGLE_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_5)    //������
//#define  MINGLE_OFF()      GPIO_ResetBits(GPIOA, GPIO_Pin_5)    //������
//#define  BULLET_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_2)    //������
//#define  BULLET_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_2)    //������
//#define  UP_DOWN2_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_1)    //��������
//#define  UP_DOWN2_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_1)    //��������
//#define  X_ON()      GPIO_SetBits(GPIOE, GPIO_Pin_11)    //Xƽ��
//#define  X_OFF()      GPIO_ResetBits(GPIOE, GPIO_Pin_11)    //Xƽ��
//#define  Y_ON()     GPIO_SetBits(GPIOE, GPIO_Pin_13)    //Yƽ��
//#define  Y_OFF()     GPIO_ResetBits(GPIOE, GPIO_Pin_13)    //Yƽ��
//#define  TURN_ON()     GPIO_SetBits(GPIOE, GPIO_Pin_14)    //ת
//#define  TURN_OFF()     GPIO_ResetBits(GPIOE, GPIO_Pin_14)    //ת
//#define  SEND_ON()     GPIO_SetBits(GPIOB, GPIO_Pin_0)    //�͵�
//#define  SEND_OFF()     GPIO_ResetBits(GPIOB, GPIO_Pin_0)    //�͵�

//#define  Y_STRETCH_KEY     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)    //Y��
//#define  Y_SHRINK_KEY     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)    //Y��
//#define  X_KEY_ON     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)    //X��
//#define  X_KEY_OFF     GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)    //X��
//#define  CLOSE_LIMIT_KEY     GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8)    //�պϵ��ռ���
//#define  OPEN_LIMIT_OFF     GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)    //�������ؼ���
//#define  CLAW_ON     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)    //���Կ���  צ�ӷ�ת
//#define  CLAW_OFF     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)    //���Կ��� צ�ӷ�ת
//#define  LED_GREEN_TOGGLE()  GPIO_ToggleBits(GPIOE, GPIO_Pin_8)

//#define  LED_RED_ON()        GPIO_SetBits(GPIOE, GPIO_Pin_7)
//#define  LED_RED_OFF()       GPIO_ResetBits(GPIOE, GPIO_Pin_7)
//#define  LED_RED_TOGGLE()    GPIO_ToggleBits(GPIOE, GPIO_Pin_7)

#define  Y_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_5)    //Y����չ
#define  Y_OFF()      GPIO_ResetBits(GPIOA, GPIO_Pin_5)    
#define  MINGLE_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_2)    //צ�Ӽ�ȡ
#define  MINGLE_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_2)   
#define  UP_DOWN_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_1)    //����
#define  UP_DOWN_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_1)    
#define  HELP_ON()     GPIO_SetBits(GPIOB, GPIO_Pin_0)    // ��Ԯ
#define  HELP_OFF()     GPIO_ResetBits(GPIOB, GPIO_Pin_0)   


void Gas_task(void *pvParameter);
void Printf_task(void *pvParameter);
extern int8_t gas_flag;
extern int8_t mov_flag;
extern int8_t end_flag;
extern int8_t mos_flag;
#endif 
