#include "uart4.h"
#include "dbus.h"
#include "stm32f4xx.h"
#include "stdio.h"


extern unsigned char dbus_buf0[DBUS_BUF_SIZE];

void UART4_Configuration(void)
{
		USART_InitTypeDef 	uart4;		//DBUS的串口
		GPIO_InitTypeDef  	gpio;		//DBUS的引脚
		DMA_InitTypeDef  	dma;		//定义DMA

		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11, GPIO_AF_UART4);

		gpio.GPIO_Pin = GPIO_Pin_11 ;
   gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOC,&gpio);

		USART_DeInit(UART4);
		uart4.USART_BaudRate = 100000;   //SBUS 100K baudrate		//DBUS波特率，请勿改动
		uart4.USART_WordLength = USART_WordLength_8b;				
		uart4.USART_StopBits = USART_StopBits_1;
		uart4.USART_Parity = USART_Parity_Even;
		uart4.USART_Mode = USART_Mode_Rx;
		uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(UART4,&uart4);

		USART_Cmd(UART4,ENABLE);
		USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
		USART_ITConfig(UART4,USART_IT_IDLE ,ENABLE);			//开启串口4空闲中断


		DMA_DeInit(DMA1_Stream2);								//dma与数据流选择
		dma.DMA_Channel= DMA_Channel_4;							//通道选择
		dma.DMA_PeripheralBaseAddr = (u32)&(UART4->DR);			//DMA外设地址
		dma.DMA_Memory0BaseAddr = (u32)dbus_buf0;				//DMA 存储器0地址
		dma.DMA_DIR = DMA_DIR_PeripheralToMemory;				//存储器到外设模式
		dma.DMA_BufferSize = DBUS_BUF_SIZE;						//数据传输量 
		dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设非增量模式
		dma.DMA_MemoryInc = DMA_MemoryInc_Enable;				//存储器增量模式
		dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
		dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//存储器数据长度:8位
		dma.DMA_Mode = DMA_Mode_Circular;						//使用循环模式 
		dma.DMA_Priority = DMA_Priority_VeryHigh;				//最高等优先级
		dma.DMA_FIFOMode = DMA_FIFOMode_Disable;				//直接模式
		dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;	//	1/4, 1/2, 3/4, 1
		dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;			//存储器突发单次传输		
		dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	//接收器突发单次传输
		DMA_Init(DMA1_Stream2,&dma);							//初始化DMA Stream数据流

//	DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);			//DMA中断
    DMA_Cmd(DMA1_Stream2,ENABLE);

////其余和单缓冲模式一样
////内存0 即Buffer0先被传输
//DMA_DoubleBufferModeConfig(DMA1_Stream2,(uint32_t)&dbus_buf1,DMA_Memory_0);		//双缓存
//DMA_DoubleBufferModeCmd(DMA1_Stream2,ENABLE);

}


