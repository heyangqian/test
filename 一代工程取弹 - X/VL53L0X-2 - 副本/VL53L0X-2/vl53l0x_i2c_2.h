#ifndef __VL53L0_I2C_2_H
#define __VL53L0_I2C_2_H

#include "sys.h"
extern char Flag_2;


//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK ̽����STM32F407������
//VL53L0X IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2017/7/1
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

//IO��������
#define VL_SDA_IN_2()  {GPIOD->MODER&=~(3<<(9*2));GPIOD->MODER|=0<<9*2;}	//PB5����ģʽ
#define VL_SDA_OUT_2() {GPIOD->MODER&=~(3<<(9*2));GPIOD->MODER|=1<<9*2;}    //PB5���ģʽ



//IO��������	 
#define VL_IIC_SCL_2    PDout(8) 		//SCL
#define VL_IIC_SDA_2    PDout(9) 		//SDA	 
#define VL_READ_SDA_2   PDin(9) 		    //����SDA 





//״̬
#define STATUS_OK_2       0x00
#define STATUS_FAIL_2     0x01

//IIC��������
void VL53L0X_i2c_init_2(void);//��ʼ��IIC��IO��

u8 VL53L0X_write_byte_2(u8 address,u8 index,u8 data);              //IICдһ��8λ����
u8 VL53L0X_write_word_2(u8 address,u8 index,u16 data);             //IICдһ��16λ����
u8 VL53L0X_write_dword_2(u8 address,u8 index,u32 data);            //IICдһ��32λ����
u8 VL53L0X_write_multi_2(u8 address, u8 index,u8 *pdata,u16 count);//IIC����д
u8 VL53L0X_read_byte_2(u8 address,u8 index,u8 *pdata);             //IIC��һ��8λ����
u8 VL53L0X_read_word_2(u8 address,u8 index,u16 *pdata);            //IIC��һ��16λ����
u8 VL53L0X_read_dword_2(u8 address,u8 index,u32 *pdata);           //IIC��һ��32λ����
u8 VL53L0X_read_multi_2(u8 address,u8 index,u8 *pdata,u16 count);  //IIC������


#endif 


