#ifndef __GAS_H__
#define __GAS_H__
#include "main.h"
#include "pid.h"


#define G_MODE 3           // 1 普通抓弹（按键）  2 普通抓弹（一键）  3 十字抓球  4 x横移抓弹（按键）  5 x横移抓弹（一键）

//#define  MINGLE_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_5)    //夹箱子
//#define  MINGLE_OFF()      GPIO_ResetBits(GPIOA, GPIO_Pin_5)    //夹箱子
//#define  BULLET_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_2)    //弹箱子
//#define  BULLET_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_2)    //弹箱子
//#define  UP_DOWN2_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_1)    //二级升降
//#define  UP_DOWN2_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_1)    //二级升降
//#define  X_ON()      GPIO_SetBits(GPIOE, GPIO_Pin_11)    //X平移
//#define  X_OFF()      GPIO_ResetBits(GPIOE, GPIO_Pin_11)    //X平移
//#define  Y_ON()     GPIO_SetBits(GPIOE, GPIO_Pin_13)    //Y平移
//#define  Y_OFF()     GPIO_ResetBits(GPIOE, GPIO_Pin_13)    //Y平移
//#define  TURN_ON()     GPIO_SetBits(GPIOE, GPIO_Pin_14)    //转
//#define  TURN_OFF()     GPIO_ResetBits(GPIOE, GPIO_Pin_14)    //转
//#define  SEND_ON()     GPIO_SetBits(GPIOB, GPIO_Pin_0)    //送弹
//#define  SEND_OFF()     GPIO_ResetBits(GPIOB, GPIO_Pin_0)    //送弹

//#define  Y_STRETCH_KEY     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)    //Y伸
//#define  Y_SHRINK_KEY     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7)    //Y缩
//#define  X_KEY_ON     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)    //X开
//#define  X_KEY_OFF     GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4)    //X关
//#define  CLOSE_LIMIT_KEY     GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_8)    //闭合弹舱极限
//#define  OPEN_LIMIT_OFF     GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_9)    //弹舱缩回极限
//#define  CLAW_ON     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)    //磁性开关  爪子翻转
//#define  CLAW_OFF     GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)    //磁性开关 爪子翻转
//#define  LED_GREEN_TOGGLE()  GPIO_ToggleBits(GPIOE, GPIO_Pin_8)

//#define  LED_RED_ON()        GPIO_SetBits(GPIOE, GPIO_Pin_7)
//#define  LED_RED_OFF()       GPIO_ResetBits(GPIOE, GPIO_Pin_7)
//#define  LED_RED_TOGGLE()    GPIO_ToggleBits(GPIOE, GPIO_Pin_7)

#define  Y_ON()      GPIO_SetBits(GPIOA, GPIO_Pin_5)    //Y轴伸展
#define  Y_OFF()      GPIO_ResetBits(GPIOA, GPIO_Pin_5)    
#define  MINGLE_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_2)    //爪子夹取
#define  MINGLE_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_2)   
#define  UP_DOWN_ON()     GPIO_SetBits(GPIOC, GPIO_Pin_1)    //升降
#define  UP_DOWN_OFF()     GPIO_ResetBits(GPIOC, GPIO_Pin_1)    
#define  HELP_ON()     GPIO_SetBits(GPIOB, GPIO_Pin_0)    // 救援
#define  HELP_OFF()     GPIO_ResetBits(GPIOB, GPIO_Pin_0)   


void Gas_task(void *pvParameter);
void Printf_task(void *pvParameter);
extern int8_t gas_flag;
extern int8_t mov_flag;
extern int8_t end_flag;
extern int8_t mos_flag;
#endif 
