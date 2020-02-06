#include "usart6.h"


uint8_t USART6temp;
u8 laster_date[laster_buffer_length];
Laster_Range laster_range;

void USART6_Configuration()
{
	USART_InitTypeDef usart6; 
	GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	DMA_InitTypeDef  dma;
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);    //使能GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);	//使能USART1时钟
	
	//串口1对应引脚复用映射
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6 ,GPIO_AF_USART6);	//GPIOB6复用为USART1
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 	//GPIOB7复用为USART1

	//串口1引脚设置
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;							//复用模式
    gpio.GPIO_OType = GPIO_OType_PP;						//推挽输出
    gpio.GPIO_Speed = GPIO_Speed_100MHz;					//速度100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_UP;						//无上下拉
    GPIO_Init(GPIOC,&gpio);									//初始化
	
	//串口初始化设置
	USART_DeInit(USART6);
	usart6.USART_BaudRate = 115200;							//串口波特率设置
	usart6.USART_WordLength = USART_WordLength_8b;			//字长设置
	usart6.USART_StopBits = USART_StopBits_1;				//1个停止位
	usart6.USART_Parity = USART_Parity_No;					//无奇偶校验位
	usart6.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;		//收发模式
    usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_Init(USART6,&usart6);								//串口初始化

	USART_Cmd(USART6,ENABLE);								//串口使能
//    USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);			//串口接收中断使能
	USART_ClearFlag(USART6, USART_FLAG_TC);					//标志位清除
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);			//串口空闲中断使能	
	
	//中断优先级配置
    nvic.NVIC_IRQChannel = USART6_IRQn;						//串口中断通道
    nvic.NVIC_IRQChannelPreemptionPriority = 0;				//抢占优先级
    nvic.NVIC_IRQChannelSubPriority = 1;					//相应优先级
    nvic.NVIC_IRQChannelCmd = ENABLE;						//中断使能
    NVIC_Init(&nvic);										//根据参数初始化优先级寄存器	
		
	//串口dma配置
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);			//串口dma使能		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能 
	DMA_DeInit(DMA2_Stream1);
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}//等待DMA可配置 
	/* 配置 DMA Stream */
	dma.DMA_Channel = DMA_Channel_5;  //通道选择
	dma.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA外设地址
	dma.DMA_Memory0BaseAddr = (u32)laster_date;//DMA 存储器0地址
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//外设到存储器
	dma.DMA_BufferSize = laster_buffer_length;//数据传输量 

	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
		
	dma.DMA_Mode = DMA_Mode_Circular;// 使用普通模式 
	dma.DMA_Priority = DMA_Priority_VeryHigh;//最高优先级
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable; //直接模式        
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA2_Stream1, &dma);//初始化DMA Stream
}


void USART6_IRQHandler(void)
{
	long int date1 , date2 , date3 , date4;
	
	USART6temp = USART6->DR;
	USART6temp = USART6->SR;
	
	DMA_Cmd(DMA2_Stream1, DISABLE);				//关闭DMA传输 	
    if( laster_date[0]=='~' && laster_date[1] == '0' && laster_date[2] == '1'  )
    {
		laster_range.laster_rang_buff[0] = laster_date[9];
		laster_range.laster_rang_buff[1] = laster_date[10];
		laster_range.laster_rang_buff[2] = laster_date[11];
		laster_range.laster_rang_buff[3] = laster_date[12];
		
		date1 = Date(laster_range.laster_rang_buff[0])*16*16*16;
		date2 = Date(laster_range.laster_rang_buff[1])*16*16;
		date3 = Date(laster_range.laster_rang_buff[2])*16;
		date4 = Date(laster_range.laster_rang_buff[3]);
		
		laster_range.laster_range_date = (date1+date2+date3+date4)/10.0;
//  		printf(" the range is %.2f  \r\n",laster_range.laster_range_date);
    }
		//重启DMA
   	DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);		
	while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);					//检查数据流是否传输完成
	DMA_SetCurrDataCounter(DMA2_Stream1, laster_buffer_length);	//写入要在DMA数据流上传输的数据量
	DMA_Cmd(DMA2_Stream1, ENABLE);										//重新开启DMA
}


int Date(char ppp)			//16进制转换为10进制
{
	if(ppp=='0')
	{
		return 0;
	}
	else if(ppp=='1')
	{
		return 1;
	}
	else if(ppp=='2')
	{
		return 2;
	}
	else if(ppp=='3')
	{
		return 3;
	}
	else if(ppp=='4')
	{
		return 4;
	}
	else if(ppp=='5')
	{
		return 5;
	}
	else if(ppp=='6')
	{
		return 6;
	}
	else if(ppp=='7')
	{
		return 7;
	}
	else if(ppp=='8')
	{
		return 8;
	}
	else if(ppp=='9')
	{
		return 9;
	}
	else if(ppp=='A')
	{
		return 10;
	}
	else if(ppp=='B')
	{
		return 11;
	}
	else if(ppp=='C')
	{
		return 12;
	}
	else if(ppp=='D')
	{
		return 13;
	}
	else if(ppp=='E')
	{
		return 14;
	}
	else if(ppp=='F')
	{
		return 15;
	}
	else
		return 0;
}




