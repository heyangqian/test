#include "upper.h"
#include "usart1.h"
/*上位机测试*/
#define BYTE0(dwTemp) ( *( (char *) (&dwTemp)   ))
#define BYTE1(dwTemp) ( *( (char *) (&dwTemp) +1))
#define BYTE2(dwTemp) ( *( (char *) (&dwTemp) +2))
#define BYTE3(dwTemp) ( *( (char *) (&dwTemp) +3))
typedef unsigned short u16;
typedef unsigned char u8;

u8 testdataosend[50];	//发送数据缓存

void Data_sends(int16_t data1,int16_t data2,int16_t data3,int16_t data4,int16_t data5,int16_t data6)
{
		uint8_t _cnt = 0;
		int8_t i = 0;
		uint8_t sum = 0;
		testdataosend[_cnt++] = 0xAA;
		testdataosend[_cnt++] = 0x30;
		testdataosend[_cnt++] = 0xAF;
		testdataosend[_cnt++] = 0xF1;
	
		testdataosend[_cnt++] = 0;
		testdataosend[_cnt++] = BYTE1(data1);
		testdataosend[_cnt++] = BYTE0(data1);
	
		testdataosend[_cnt++] = BYTE1(data2);
		testdataosend[_cnt++] = BYTE0(data2);
	
		testdataosend[_cnt++] = BYTE1(data3);
		testdataosend[_cnt++] = BYTE0(data3);
	
		testdataosend[_cnt++] = BYTE1(data4);
		testdataosend[_cnt++] = BYTE0(data4);
	
		testdataosend[_cnt++] = BYTE1(data5);
		testdataosend[_cnt++] = BYTE0(data5);
	
		testdataosend[_cnt++] = BYTE1(data6);
		testdataosend[_cnt++] = BYTE0(data6);	
		testdataosend[4] = _cnt - 5;
		for ( i = 0; i < _cnt; i++)
		{
				sum += testdataosend[i];	
		}
		testdataosend[_cnt++] = sum;
		for (uint8_t k = 0; k < _cnt; k++)
		{
				while ((USART1->SR&0x40) == 0);
				USART1->DR= testdataosend[k];	
		}
}


