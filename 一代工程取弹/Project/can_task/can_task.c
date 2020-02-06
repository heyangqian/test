#include "can_task.h"
#include "motor.h"
#include "gimbal.h"
#include "can1.h"
#include "can2.h"
/****************CAN1*****************/
int8_t ele1_flag = 0;    //can1上电检测
int8_t ele2_flag = 0;    //can2上电检测

u8 Can_Send_Msg(u8* msg,u8 len)     //测试  
{	
        u8 mbox;
        u16 i=0;
        CanTxMsg Tx_Message;
        Tx_Message.StdId=0x200;		// 标准标识符 
        Tx_Message.ExtId=0x200;		// 设置扩展标示符 
        Tx_Message.IDE=CAN_Id_Standard; 	// 标准帧
        Tx_Message.RTR=CAN_RTR_Data;		// 数据帧
        Tx_Message.DLC=len;			// 要发送的数据长度
        for(i=0;i<len;i++)Tx_Message.Data[i]=msg[i]; 
        mbox= CAN_Transmit(CAN1, &Tx_Message);   
        i=0;
        while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
        if(i>=0XFFF)return 1;
        return 0; 
}
/***************CAN——1*****************
	函数名：Motor_201_204_Can1TX
	作  用： can1 201 -- 204发送数据
	参	数： 
******************************************/
void Motor_201_204_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x200;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN1,&tx_message);
}
/***************CAN——1*****************
	函数名：Motor_205_208_Can1TX
	作  用： can1 205 -- 208发送数据
	参	数：
******************************************/
void Motor_205_208_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can1 205 -- 208 发送数据
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FF;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN1,&tx_message);
}

/***************CAN——1*****************
	函数名：Gimbal_20x_Can1TX
	作  用： can1 云台发送数据
	参	数： 
******************************************/
void Gimbal_20x_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can1 云台发送数据
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FF;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN1,&tx_message);
}

/***************CAN——1*****************
	函数名：Can1_motor_handle
	作  用： can1处理数据
	参	数： 
******************************************/
void Can1_motor_handle(CanRxMsg *rx_message)   //can1处理数据
{
	switch (rx_message->StdId)
	{
		case CAN_3508_M1_ID : Motor_x[0].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_x[0].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_x[0].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_x[0].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M2_ID : Motor_x[1].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_x[1].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_x[1].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_x[1].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M3_ID : Motor_x[2].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_x[2].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_x[2].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_x[2].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M4_ID : Motor_x[3].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_x[3].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_x[3].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_x[3].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M5_ID : Motor_x[4].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_x[4].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_x[4].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_x[4].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break; 
		case CAN_3508_M6_ID : Motor_x[5].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_x[5].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_x[5].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_x[5].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case 0x401 : get_gyro_measuer(&gyro_yaw,&Rx_message);break;
		default : break;
	}
}


/****************************************************************************************************CAN2********************************************/

/***************CAN——2*****************
	函数名：Motor_Can2TX
	作  用： CAN2 通信发送数据
	参	数： 
******************************************/
void Motor_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//CAN2 通信发送数据
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x125;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN——2*****************
	函数名：Motor_201_204_Can2TX
	作  用： CAN2 201——204发送数据
	参	数： 
******************************************/
void Motor_201_204_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//CAN2 201——204发送数据
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x200;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN——2*****************
	函数名：Gimbal_Can2TX
	作  用： can2 云台发送数据
	参	数： 
******************************************/
void Gimbal_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can2 云台发送数据
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FF;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN——2*****************
	函数名：Can2_motor_handle
	作  用： can2处理数据
	参	数： 
******************************************/
int16_t Mes[8];
//int16_t rst_flag = 0;
void Can2_motor_handle(CanRxMsg *rx_message)   //can2处理数据
{
	switch (rx_message->StdId)
	{
		case CAN_3508_M1_ID : Motor_y[0].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_y[0].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_y[0].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_y[0].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M2_ID : Motor_y[1].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_y[1].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_y[1].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_y[1].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M3_ID : Motor_y[2].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_y[2].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_y[2].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_y[2].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M4_ID : Motor_y[3].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_y[3].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_y[3].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_y[3].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case CAN_3508_M5_ID : Motor_y[4].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_y[4].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_y[4].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_y[4].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break; 
		case CAN_3508_M6_ID : Motor_y[5].motor_angle = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);
							  Motor_y[5].motor_speed = (int16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);
							  Motor_y[5].motor_torque = (int16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]);
							  Motor_y[5].motor_temperate = (int16_t)((rx_message)->Data[6] << 8 | (rx_message)->Data[7]);break;
		case 0x401 : get_gyro_measuer(&gyro_yaw,&Rx2_message);break;
//		case 0x125 :  Mes[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		default : break;
	}
}

/*******************************************************************************CAN1 CAN2副本*******************************************************************/
/***************CAN——1，2*****************
	函数名：Can2_Mes_handle
	作  用： can2副本处理数据
	参	数： 
******************************************/
int yy = 0;
int16_t Message[8];
void Can2_Mes_handle(CanRxMsg *rx_message)         //can2副本处理数据
{
	switch (rx_message->StdId)
	{
		case 0x521 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		case 0x125 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		default : break;
	}
}

/***************CAN——1，2*****************
	函数名：Mes_Can2TX
	作  用： can2 副本发送数据
	参	数： 
******************************************/
void Mes_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can2 副本发送数据
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x125;		  			//标准标识符
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//拓展标识符
	tx_message.IDE		= CAN_ID_STD;			//用来设定消息标识符的类型    
	tx_message.RTR		= CAN_RTR_DATA;			//用来设定待传输消息的帧类型	数据帧或远程帧
	tx_message.DLC		= 8;					//用来设定待传输消息的帧长度 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN——1，2*****************
	函数名：Can1_Mes_handle
	作  用： can1副本处理数据
	参	数： 
******************************************/
void Can1_Mes_handle(CanRxMsg *rx_message)    //can1副本处理数据
{
	switch (rx_message->StdId)
	{
		case 0x521 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		case 0x125 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		default : break;
	}
}


