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
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);    //ʹ��GPIOBʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);	//ʹ��USART1ʱ��
	
	//����1��Ӧ���Ÿ���ӳ��
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6 ,GPIO_AF_USART6);	//GPIOB6����ΪUSART1
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 	//GPIOB7����ΪUSART1

	//����1��������
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;							//����ģʽ
    gpio.GPIO_OType = GPIO_OType_PP;						//�������
    gpio.GPIO_Speed = GPIO_Speed_100MHz;					//�ٶ�100MHz
    gpio.GPIO_PuPd = GPIO_PuPd_UP;						//��������
    GPIO_Init(GPIOC,&gpio);									//��ʼ��
	
	//���ڳ�ʼ������
	USART_DeInit(USART6);
	usart6.USART_BaudRate = 115200;							//���ڲ���������
	usart6.USART_WordLength = USART_WordLength_8b;			//�ֳ�����
	usart6.USART_StopBits = USART_StopBits_1;				//1��ֹͣλ
	usart6.USART_Parity = USART_Parity_No;					//����żУ��λ
	usart6.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;		//�շ�ģʽ
    usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_Init(USART6,&usart6);								//���ڳ�ʼ��

	USART_Cmd(USART6,ENABLE);								//����ʹ��
//    USART_ITConfig(USART6,USART_IT_RXNE,DISABLE);			//���ڽ����ж�ʹ��
	USART_ClearFlag(USART6, USART_FLAG_TC);					//��־λ���
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);			//���ڿ����ж�ʹ��	
	
	//�ж����ȼ�����
    nvic.NVIC_IRQChannel = USART6_IRQn;						//�����ж�ͨ��
    nvic.NVIC_IRQChannelPreemptionPriority = 0;				//��ռ���ȼ�
    nvic.NVIC_IRQChannelSubPriority = 1;					//��Ӧ���ȼ�
    nvic.NVIC_IRQChannelCmd = ENABLE;						//�ж�ʹ��
    NVIC_Init(&nvic);										//���ݲ�����ʼ�����ȼ��Ĵ���	
		
	//����dma����
	USART_DMACmd(USART6,USART_DMAReq_Rx,ENABLE);			//����dmaʹ��		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ�� 
	DMA_DeInit(DMA2_Stream1);
	while (DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}//�ȴ�DMA������ 
	/* ���� DMA Stream */
	dma.DMA_Channel = DMA_Channel_5;  //ͨ��ѡ��
	dma.DMA_PeripheralBaseAddr = (u32)&USART6->DR;//DMA�����ַ
	dma.DMA_Memory0BaseAddr = (u32)laster_date;//DMA �洢��0��ַ
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;//���赽�洢��
	dma.DMA_BufferSize = laster_buffer_length;//���ݴ����� 

	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
		
	dma.DMA_Mode = DMA_Mode_Circular;// ʹ����ͨģʽ 
	dma.DMA_Priority = DMA_Priority_VeryHigh;//������ȼ�
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable; //ֱ��ģʽ        
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA2_Stream1, &dma);//��ʼ��DMA Stream
}


void USART6_IRQHandler(void)
{
	long int date1 , date2 , date3 , date4;
	
	USART6temp = USART6->DR;
	USART6temp = USART6->SR;
	
	DMA_Cmd(DMA2_Stream1, DISABLE);				//�ر�DMA���� 	
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
		//����DMA
   	DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);		
	while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);					//����������Ƿ������
	DMA_SetCurrDataCounter(DMA2_Stream1, laster_buffer_length);	//д��Ҫ��DMA�������ϴ����������
	DMA_Cmd(DMA2_Stream1, ENABLE);										//���¿���DMA
}


int Date(char ppp)			//16����ת��Ϊ10����
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




