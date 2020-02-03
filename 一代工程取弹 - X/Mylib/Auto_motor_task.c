//#include "Auto_motor_task.h"
//#include "Auto_aim_task.h"
//#include "FreeRTOSConfig.h"
//#include "FreeRTOS.h"
//#include "Variable.h"
//#include "semphr.h"  
//#include "task.h"

//static void Pid_init(void);
//static void Pid_Calc(void);
//static void CAN_CMD_AUTO(rm16 Left, rm16 Right);
//static void M2006_postions_add(void);

//void Auto_Motor_task(void *pvParameters)
//{
//	TickType_t xLastWakeTime;
//	const TickType_t xFrequency = 2; //周期为2ms
//	xLastWakeTime = xTaskGetTickCount ();
//	while(1)
//	{
//		O_O.Left_ = get_Auto_Left_Point();
//		O_O.Right_ = get_Auto_Rigth_Point();
//		O_O.pan_pluck_Point = get_pluck_Point();
//		Pid_init();
//		M2006_postions_add();
//		Pid_Calc();
//		
////		CAN_CMD_AUTO(O_O.Auto_left_v.out,O_O.Auto_Right_v.out);
////		CAN_CMD_GIMBAL(O_O.pitch_v.out,O_O.pluck_v.out);
//		
//		vTaskDelayUntil(&xLastWakeTime,xFrequency);
//	}
//}
//float Right_v[3] = {4,0.25f,0},Left_v[3] = {4,0.25f,0},Right_set_v = -6000,Left_set_v = 6000,Raight_a[2] = {10,20},Left_a[2] = {10,20};

//float M2006_v[3] = {12,0.5},M2006_p[3] = {0.08f},Max_v = 10000;

//static void Pid_init(void)
//{
//	Pid_Init(&O_O.Auto_left_v,Left_v[0],Left_v[1],Left_v[2],20000,16000);     //位置环结构体变量地址 P,I,D,积分上限，输出上限
//    Pid_Init(&O_O.Auto_Right_v,Right_v[0],Right_v[1],Right_v[2],20000,16000);//速度环结构体变量地址 P,I,D,积分上限，输出上限
//	
//	Pid_Init(&O_O.pluck_v,M2006_v[0],M2006_v[1],M2006_v[2],20000,5000);//速度环结构体变量地址 P,I,D,积分上限，输出上限
//	Pid_Init(&O_O.pluck_p,M2006_p[0],M2006_p[1],M2006_p[2],0,Max_v);//速度环结构体变量地址 P,I,D,积分上限，输出上限
//}

//float m2006_v = 5;  //1 32768 2 65536 3 98304
//int times_[3] = {0,0,899};
//static void M2006_postions_add(void)
//{
//	times_[0] = xTaskGetTickCount ();
//	if (times_[0] - times_[1] >= times_[2])
//	{
//		times_[1] = xTaskGetTickCount (); 
//		O_O.pluck_p.set += 32768;
//	}
//}

//static void Pid_Calc(void)
//{
//	O_O.Auto_Right_v_a.set = Right_set_v;
//	add_speed(&O_O.Auto_Right_v_a,Raight_a[0],Raight_a[1]);  //指针 加速度 减速度
//	O_O.Auto_Right_v.set = O_O.Auto_Right_v_a.A.go;
//	O_O.Auto_Right_v.get = O_O.Right_->speed_rpm;	
//	PID_Calc(&O_O.Auto_Right_v);
//	
//	O_O.Auto_left_v_a.set = Left_set_v;
//	add_speed(&O_O.Auto_left_v_a,Left_a[0],Left_a[1]);  //指针 加速度 减速度
//	O_O.Auto_left_v.set = O_O.Auto_left_v_a.A.go;
//	O_O.Auto_left_v.get = O_O.Left_->speed_rpm;
//	PID_Calc(&O_O.Auto_left_v);	
//	
////	O_O.pluck_p.set = m2006_v;
//	O_O.pluck_p.get = O_O.pan_pluck_Point->angle_num;
//	PID_Calc(&O_O.pluck_p);
//	O_O.pluck_v.set = O_O.pluck_p.out;
////	O_O.pluck_v.set = 5000;
//	O_O.pluck_v.get = O_O.pan_pluck_Point->speed_rpm;
//	PID_Calc(&O_O.pluck_v);
//}

//static void CAN_CMD_AUTO(rm16 Right, rm16 Left)		//摩擦轮
//{
//    Tx_message.StdId = 0x200;
//    Tx_message.IDE = CAN_ID_STD;
//    Tx_message.RTR = CAN_RTR_DATA;
//    Tx_message.DLC = 0x08;
////    Tx_message.Data[0] = 0X00
////    Tx_message.Data[1] = 0X00;
//    Tx_message.Data[2] = (Right >> 8);
//    Tx_message.Data[3] = Right;
//	Tx_message.Data[4] = (Left >> 8);
//    Tx_message.Data[5] = Left;
////	Tx_message.Data[6] = 0X00;
////	Tx_message.Data[7] = 0X00;
//    CAN_Transmit( CAN1,  &Tx_message );
//}


