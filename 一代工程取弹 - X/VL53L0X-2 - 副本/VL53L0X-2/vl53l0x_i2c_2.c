#include "vl53l0x_i2c_2.h"
#include "delay.h"



//VL53L0X I2C��ʼ��
void VL53L0X_i2c_init_2(void)
{
	GPIO_InitTypeDef 		GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//ʹ��GPIOBʱ��
	 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;//�˿�����
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��

	GPIO_SetBits(GPIOD,GPIO_Pin_8|GPIO_Pin_9);//PB10,PB11 �����
}

//����IIC��ʼ�ź�
void VL_IIC_Start_2(void)
{
	VL_SDA_OUT_2();//sda�����
	VL_IIC_SDA_2=1;	  	  
	VL_IIC_SCL_2=1;
	delay_us(4);
 	VL_IIC_SDA_2=0;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	VL_IIC_SCL_2=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	
}

//����IICֹͣ�ź�
void VL_IIC_Stop_2(void)
{
	VL_SDA_OUT_2();//sda�����
	VL_IIC_SCL_2=0;
	VL_IIC_SDA_2=0;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	VL_IIC_SCL_2=1; 
	VL_IIC_SDA_2=1;//����I2C���߽����ź�
	delay_us(4);			
	
	
}

//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 VL_IIC_Wait_Ack_2(void)
{
	u8 ucErrTime=0;
	VL_SDA_IN_2();  //SDA����Ϊ����  
	VL_IIC_SDA_2=1;delay_us(1);	   
	VL_IIC_SCL_2=1;delay_us(1);	 
	while(VL_READ_SDA_2)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			VL_IIC_Stop_2();
			return 1;
		}
	}
	VL_IIC_SCL_2=0;//ʱ�����0 	 
	
	return 0;  
}

//����ACKӦ��
void VL_IIC_Ack_2(void)
{
	VL_IIC_SCL_2=0;
	VL_SDA_OUT_2();
	VL_IIC_SDA_2=0;
	delay_us(2);
	VL_IIC_SCL_2=1;
	delay_us(2);
	VL_IIC_SCL_2=0;
		

}

//������ACKӦ��		    
void VL_IIC_NAck_2(void)
{
	VL_IIC_SCL_2=0;
	VL_SDA_OUT_2();
	VL_IIC_SDA_2=1;
	delay_us(2);
	VL_IIC_SCL_2=1;
	delay_us(2);
	VL_IIC_SCL_2=0;
		

}

//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void VL_IIC_Send_Byte_2(u8 txd)
{           
	
	
    u8 t;   
	VL_SDA_OUT_2(); 	    
    VL_IIC_SCL_2=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7)
			VL_IIC_SDA_2=1;
		else
			VL_IIC_SDA_2=0;
		txd<<=1; 	  
		delay_us(2);  
		VL_IIC_SCL_2=1;
		delay_us(2); 
		VL_IIC_SCL_2=0;	
		delay_us(2);
    }
}



//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 VL_IIC_Read_Byte_2(unsigned char ack)
{
	
	unsigned char i,receive=0;
	

		VL_SDA_IN_2();//SDA����Ϊ����
		for(i=0;i<8;i++ )
		{
			VL_IIC_SCL_2=0; 
			delay_us(4);
		  VL_IIC_SCL_2=1;
			receive<<=1;
			if(VL_READ_SDA_2)receive++;   
		  delay_us(4); //1
		}					 
		if (!ack)
			VL_IIC_NAck_2();//����nACK
		else


		VL_IIC_Ack_2(); //����ACK   
		return receive;

}

//IICдһ���ֽ�����
u8 VL_IIC_Write_1Byte_2(u8 SlaveAddress, u8 REG_Address,u8 REG_data)
{
		

	VL_IIC_Start_2();
	VL_IIC_Send_Byte_2(SlaveAddress);
	if(VL_IIC_Wait_Ack_2())
	{
		VL_IIC_Stop_2();//�ͷ�����
		return 1;//ûӦ�����˳�

	}
	VL_IIC_Send_Byte_2(REG_Address);
	VL_IIC_Wait_Ack_2();	
	VL_IIC_Send_Byte_2(REG_data);
	VL_IIC_Wait_Ack_2();	
	VL_IIC_Stop_2();

	return 0;
}

//IIC��һ���ֽ�����
u8 VL_IIC_Read_1Byte_2(u8 SlaveAddress, u8 REG_Address,u8 *REG_data)
{
		

	VL_IIC_Start_2();
	VL_IIC_Send_Byte_2(SlaveAddress);//��д����
	if(VL_IIC_Wait_Ack_2())
	{
		 VL_IIC_Stop_2();//�ͷ�����
		 return 1;//ûӦ�����˳�
	}		
	VL_IIC_Send_Byte_2(REG_Address);
	VL_IIC_Wait_Ack_2();
	VL_IIC_Start_2(); 
	VL_IIC_Send_Byte_2(SlaveAddress|0x01);//��������
	VL_IIC_Wait_Ack_2();
	*REG_data = VL_IIC_Read_Byte_2(0);
	VL_IIC_Stop_2();

	return 0;
}

//IICдn�ֽ�����
u8 VL_IIC_Write_nByte_2(u8 SlaveAddress, u8 REG_Address,u16 len, u8 *buf)
{
		

	VL_IIC_Start_2();
	VL_IIC_Send_Byte_2(SlaveAddress);//��д����
	if(VL_IIC_Wait_Ack_2()) 
	{
		VL_IIC_Stop_2();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	VL_IIC_Send_Byte_2(REG_Address);
	VL_IIC_Wait_Ack_2();
	while(len--)
	{
		VL_IIC_Send_Byte_2(*buf++);//����buff������
		VL_IIC_Wait_Ack_2();	
	}
	VL_IIC_Stop_2();//�ͷ�����

	return 0;
	
}

//IIC��n�ֽ�����
u8 VL_IIC_Read_nByte_2(u8 SlaveAddress, u8 REG_Address,u16 len,u8 *buf)
{
	

	VL_IIC_Start_2();
	VL_IIC_Send_Byte_2(SlaveAddress);//��д����
	if(VL_IIC_Wait_Ack_2()) 
	{
		VL_IIC_Stop_2();//�ͷ�����
		return 1;//ûӦ�����˳�
	}
	VL_IIC_Send_Byte_2(REG_Address);
	VL_IIC_Wait_Ack_2();

	VL_IIC_Start_2();
	VL_IIC_Send_Byte_2(SlaveAddress|0x01);//��������
	VL_IIC_Wait_Ack_2();
	while(len)
	{
		if(len==1)
		{
			*buf = VL_IIC_Read_Byte_2(0);
		}
		else
		{
			*buf = VL_IIC_Read_Byte_2(1);
		}
		buf++;
		len--;
	}
	VL_IIC_Stop_2();//�ͷ�����

	return 0;
	
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 
//VL53L0X д�������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_write_multi_2(u8 address, u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK_2;
	
	if(VL_IIC_Write_nByte_2(address,index,count,pdata))
	{
	   status  = STATUS_FAIL_2;
	}

	return status;
	
}


//VL53L0X ���������
//address:��ַ
//index:ƫ�Ƶ�ַ
//pdata:����ָ��
//count:���� ���65535
u8 VL53L0X_read_multi_2(u8 address,u8 index,u8 *pdata,u16 count)
{
	u8 status = STATUS_OK_2;

	if(VL_IIC_Read_nByte_2(address,index,count,pdata))
	{
		status  = STATUS_FAIL_2;
	}

	return status;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_write_byte_2(u8 address,u8 index,u8 data)
{
	u8 status = STATUS_OK_2;

		status = VL53L0X_write_multi_2(address,index,&data,1);
		return status;

}

//VL53L0X д1������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_write_word_2(u8 address,u8 index,u16 data)
{
	u8 status = STATUS_OK_2;
	u8 buffer[2];

		//��16λ���ݲ�ֳ�8λ
		buffer[0] = (u8)(data>>8);//�߰�λ
		buffer[1] = (u8)(data&0xff);//�Ͱ�λ
		
		if(index%2==1)
		{  
			//����ͨ�Ų��ܴ���Է�2�ֽڶ���Ĵ������ֽ�
			status = VL53L0X_write_multi_2(address,index,&buffer[0],1);
			status = VL53L0X_write_multi_2(address,index,&buffer[0],1);
		}else
		{
			status = VL53L0X_write_multi_2(address,index,buffer,2);
		}
		return status;

}

//VL53L0X д1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_write_dword_2(u8 address,u8 index,u32 data)
{
	
    u8 status = STATUS_OK_2;
    u8 buffer[4];	

		//��32λ���ݲ�ֳ�8λ
		buffer[0] = (u8)(data>>24);
		buffer[1] = (u8)((data&0xff0000)>>16);
		buffer[2] = (u8)((data&0xff00)>>8);
		buffer[3] = (u8)(data&0xff);
		status = VL53L0X_write_multi_2(address,index,buffer,4);

		return status;

}


//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(8λ)
u8 VL53L0X_read_byte_2(u8 address,u8 index,u8 *pdata)
{
	u8 status = STATUS_OK_2;

		status = VL53L0X_read_multi_2(address,index,pdata,1);
		
		return status;

	 
}

//VL53L0X ��������(˫�ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(16λ)
u8 VL53L0X_read_word_2(u8 address,u8 index,u16 *pdata)
{
	u8 status = STATUS_OK_2;
	u8 buffer[2];

	status = VL53L0X_read_multi_2(address,index,buffer,2);
	
	*pdata = ((u16)buffer[0]<<8)+(u16)buffer[1];
	
	return status;

}

//VL53L0X ��1������(���ֽ�)
//address:��ַ
//index:ƫ�Ƶ�ַ
//data:����(32λ)
u8 VL53L0X_read_dword_2(u8 address,u8 index,u32 *pdata)
{
	u8 status = STATUS_OK_2;
	u8 buffer[4];

	status = VL53L0X_read_multi_2(address,index,buffer,4);
	
	*pdata = ((u32)buffer[0]<<24)+((u32)buffer[1]<<16)+((u32)buffer[2]<<8)+((u32)buffer[3]);
	
	return status;

	
}





