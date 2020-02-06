#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "usart3.h"
#include "semphr.h"  
#include "task.h"  
#include "main.h"

uint8_t USART3temp;


/*妙算接收数据*/
unsigned   char   RX_BUF[22];  
int  RX_BUF_1[12];

void USART3_Configuration(void)
{
	USART_InitTypeDef		usart3;
	GPIO_InitTypeDef 		gpio;
	DMA_InitTypeDef  	 	dma;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11, GPIO_AF_USART3);

	
	gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;	
	GPIO_Init(GPIOB,&gpio);
    
    
    gpio.GPIO_Pin = GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd=GPIO_PuPd_UP;	
	GPIO_Init(GPIOB,&gpio);
         

	USART_DeInit(USART3);
	usart3.USART_BaudRate = 100000;
	usart3.USART_WordLength = USART_WordLength_8b;
	usart3.USART_StopBits = USART_StopBits_1;
	usart3.USART_Parity = USART_Parity_No ;
	usart3.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;   
 
	USART_Init(USART3,&usart3);
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART3,ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_TC);
	
	USART_ITConfig(USART3,USART_IT_IDLE ,ENABLE);		//开启串口6空闲中断
	
		
	/* 配置 DMA Stream */    
	//USART2_RX
	DMA_DeInit(DMA1_Stream1);			//dma与数据流选择
    dma.DMA_Channel= DMA_Channel_4;		//通道选择
    dma.DMA_PeripheralBaseAddr = (u32)&(USART3->DR);	//DMA外设地址
    dma.DMA_Memory0BaseAddr =(u32)RX_BUF;			//DMA 存储器0地址
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;			//存储器到外设模式
    dma.DMA_BufferSize = 22;					//数据传输量 120
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//外设非增量模式
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;			//存储器增量模式
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//外设数据长度:8位
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//存储器数据长度:8位
    dma.DMA_Mode = DMA_Mode_Circular;					// 使用循环模式 
    dma.DMA_Priority = DMA_Priority_VeryHigh;			//最高等优先级
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;			 //直接模式
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;		//	1/4, 1/2, 3/4, 1
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;		//存储器突发单次传输		
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;	//接收器突发单次传输
    DMA_Init(DMA1_Stream1,&dma);	 //初始化DMA Stream数据流
		
}

/**
  * @brief  裁判系统数据读取中断
  * @param  void
  * @retval void
  * @note	为了保证数据包不丢失，注意中断的优先级
  */
extern SemaphoreHandle_t BinarySemaphore ;//二值信号量句柄


void USART3_IRQHandler(void)
{     

     	BaseType_t xHigherPriorityTaskWoken;//二值信号量句柄BaseType_t xHigherPriorityTaskWoken;
        USART3temp = USART3->DR;					//清除空闲中断的标志位
        USART3temp = USART3->SR;
        DMA_Cmd(DMA1_Stream1, DISABLE);				//关闭DMA传输    
		if(DMA1_Stream1->NDTR ==22 )
		{		 
			if((RX_BUF[0]=='$')&&(RX_BUF[21]=='#'))			  
			{
				RX_BUF_1[0]=RX_BUF[1]<<8|RX_BUF[2];		
				RX_BUF_1[1]=RX_BUF[3]<<8|RX_BUF[4];

				RX_BUF_1[2]=RX_BUF[5]<<8|RX_BUF[6];		
				RX_BUF_1[3]=RX_BUF[7]<<8|RX_BUF[8];
			
				RX_BUF_1[4]=RX_BUF[9]<<8|RX_BUF[10];		
				RX_BUF_1[5]=RX_BUF[11]<<8|RX_BUF[12];
			
				RX_BUF_1[6]=RX_BUF[13]<<8|RX_BUF[14];		
				RX_BUF_1[7]=RX_BUF[15]<<8|RX_BUF[16];
						
				RX_BUF_1[8]=RX_BUF[17];	
				RX_BUF_1[9]=RX_BUF[18];	
				RX_BUF_1[10]=RX_BUF[19];	
				RX_BUF_1[11]=RX_BUF[20];	          
		   }
	   }
        //释放二值信号量     
		//重启DMA
		DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);		
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);					//检查数据流是否传输完成
		DMA_SetCurrDataCounter(DMA1_Stream1, 22);	//写入要在DMA数据流上传输的数据量
		DMA_Cmd(DMA1_Stream1, ENABLE);
		if((DMA1_Stream1->NDTR ==22)&&(BinarySemaphore!=NULL))//接收到数据，并且二值信号量有效
        {
                xSemaphoreGiveFromISR(BinarySemaphore,&xHigherPriorityTaskWoken);	//释放二值信号量
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//如果需要的话进行一次任务切换
        }         //重新开启DMA
          
}

void USART3_SendChar(unsigned char b)
{
    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
    USART_SendData(USART3,b);
}



