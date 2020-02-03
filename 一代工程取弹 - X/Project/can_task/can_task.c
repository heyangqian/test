#include "can_task.h"
#include "motor.h"
#include "gimbal.h"
#include "can1.h"
#include "can2.h"
/****************CAN1*****************/
int8_t ele1_flag = 0;    //can1�ϵ���
int8_t ele2_flag = 0;    //can2�ϵ���

u8 Can_Send_Msg(u8* msg,u8 len)     //����  
{	
        u8 mbox;
        u16 i=0;
        CanTxMsg Tx_Message;
        Tx_Message.StdId=0x200;		// ��׼��ʶ�� 
        Tx_Message.ExtId=0x200;		// ������չ��ʾ�� 
        Tx_Message.IDE=CAN_Id_Standard; 	// ��׼֡
        Tx_Message.RTR=CAN_RTR_Data;		// ����֡
        Tx_Message.DLC=len;			// Ҫ���͵����ݳ���
        for(i=0;i<len;i++)Tx_Message.Data[i]=msg[i]; 
        mbox= CAN_Transmit(CAN1, &Tx_Message);   
        i=0;
        while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//�ȴ����ͽ���
        if(i>=0XFFF)return 1;
        return 0; 
}
/***************CAN����1*****************
	��������Motor_201_204_Can1TX
	��  �ã� can1 201 -- 204��������
	��	���� 
******************************************/
void Motor_201_204_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x200;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN1,&tx_message);
}
/***************CAN����1*****************
	��������Motor_205_208_Can1TX
	��  �ã� can1 205 -- 208��������
	��	����
******************************************/
void Motor_205_208_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can1 205 -- 208 ��������
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FF;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN1,&tx_message);
}

/***************CAN����1*****************
	��������Gimbal_20x_Can1TX
	��  �ã� can1 ��̨��������
	��	���� 
******************************************/
void Gimbal_20x_Can1TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can1 ��̨��������
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FF;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN1,&tx_message);
}

/***************CAN����1*****************
	��������Can1_motor_handle
	��  �ã� can1��������
	��	���� 
******************************************/
void Can1_motor_handle(CanRxMsg *rx_message)   //can1��������
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

/***************CAN����2*****************
	��������Motor_Can2TX
	��  �ã� CAN2 ͨ�ŷ�������
	��	���� 
******************************************/
void Motor_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//CAN2 ͨ�ŷ�������
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x125;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN����2*****************
	��������Motor_201_204_Can2TX
	��  �ã� CAN2 201����204��������
	��	���� 
******************************************/
void Motor_201_204_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//CAN2 201����204��������
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x200;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN����2*****************
	��������Gimbal_Can2TX
	��  �ã� can2 ��̨��������
	��	���� 
******************************************/
void Gimbal_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can2 ��̨��������
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x1FF;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN����2*****************
	��������Can2_motor_handle
	��  �ã� can2��������
	��	���� 
******************************************/
int16_t Mes[8];
//int16_t rst_flag = 0;
void Can2_motor_handle(CanRxMsg *rx_message)   //can2��������
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

/*******************************************************************************CAN1 CAN2����*******************************************************************/
/***************CAN����1��2*****************
	��������Can2_Mes_handle
	��  �ã� can2������������
	��	���� 
******************************************/
int yy = 0;
int16_t Message[8];
void Can2_Mes_handle(CanRxMsg *rx_message)         //can2������������
{
	switch (rx_message->StdId)
	{
		case 0x521 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		case 0x125 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		default : break;
	}
}

/***************CAN����1��2*****************
	��������Mes_Can2TX
	��  �ã� can2 ������������
	��	���� 
******************************************/
void Mes_Can2TX(int16_t Vel_1,int16_t Vel_2,int16_t Vel_3,int16_t Vel_4 )	//can2 ������������
{
	CanTxMsg tx_message;
	
	tx_message.StdId = 0x125;		  			//��׼��ʶ��
	tx_message.Data[0] = (int8_t)(Vel_1 >> 8) ;
	tx_message.Data[1] = (int8_t) Vel_1;
	tx_message.Data[2] = (int8_t)(Vel_2 >> 8);
	tx_message.Data[3] = (int8_t) Vel_2;
	tx_message.Data[4] = (int8_t)(Vel_3>> 8);
	tx_message.Data[5] = (int8_t) Vel_3;
	tx_message.Data[6] = (int8_t)(Vel_4 >> 8);
	tx_message.Data[7] = (int8_t) Vel_4;
	
	
	tx_message.ExtId	= CAN_Id_Standard ;		//��չ��ʶ��
	tx_message.IDE		= CAN_ID_STD;			//�����趨��Ϣ��ʶ��������    
	tx_message.RTR		= CAN_RTR_DATA;			//�����趨��������Ϣ��֡����	����֡��Զ��֡
	tx_message.DLC		= 8;					//�����趨��������Ϣ��֡���� 0-8

	CAN_Transmit(CAN2,&tx_message);
}

/***************CAN����1��2*****************
	��������Can1_Mes_handle
	��  �ã� can1������������
	��	���� 
******************************************/
void Can1_Mes_handle(CanRxMsg *rx_message)    //can1������������
{
	switch (rx_message->StdId)
	{
		case 0x521 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		case 0x125 : Message[0] = (int16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);break;
		default : break;
	}
}


