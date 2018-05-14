#include "can.h" 
#include <stdlib.h>
#include "conmunication.h"
#include "uart.h"
#include "string.h"


struct DriverDataStruct *MotorPara;
struct    WQSDataStruct *WQSPara;
struct setHostPara stcHostsPara;
unsigned char WQS_flag = 0x00;
extern struct WQSDataStruct wqs_data;

union int2char
{
	int num;
	unsigned char byte[2];
};
struct MCU2Raspi UploadData;
//float Tempph = 0, Temporp, Tempec, Temptur;
float TempWQS[4] = {1.11, 1.21, 1.31, 1.41}, TempIRS[2] = {0};
unsigned char SensorBoard_Flag = 0x00;

void WQS2RaspberryPi(void);

void DriverWriteData(struct DriverDataStruct *DriverWriteData)
{

}

void DriverReadData(struct DriverDataStruct *DriverRemoteData)
{

}

/*
函数名称: DriverReadWriteData
输入参数: *DriverSendData
输出参数: 0 参数错误, 1 运动参数遥测, 2 功率参数遥测, 3 运动参数写入
函数功能: 主机使用该函数与从机(驱动板)交互
*/

unsigned char DriverReadWriteData(struct DriverDataStruct *DriverSendData)
{
	static CanTxMsg TxMessage;
	if (DriverSendData->Dfi == DriverFrameMotion)
	{
		TxMessage.ExtId=DriverSendData->Dfi;					 //使用的扩展ID
		TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
		TxMessage.RTR=CAN_RTR_REMOTE;				 //发送的是数据
		TxMessage.DLC = 0;
		CAN_Transmit(CAN1, &TxMessage);
		return 1;
	}
	else if (DriverSendData->Dfi == DriverFrameCapacity)
	{
		TxMessage.ExtId=DriverSendData->Dfi;					 //使用的扩展ID
		TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
		TxMessage.RTR=CAN_RTR_REMOTE;				 //发送的是数据
		CAN_Transmit(CAN1, &TxMessage);
		return 2;
	}
	else if (DriverSendData->Dfi == DriverFrameWrite)
	{
		TxMessage.ExtId=DriverSendData->Dfi;					 //使用的扩展ID
		TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
		TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
		TxMessage.DLC=8;							 //数据长度为2字节
		TxMessage.Data[0]=(DriverSendData->Volocity >> 8) & 0xff;
		TxMessage.Data[1]=DriverSendData->Volocity & 0xff;
		TxMessage.Data[2]=(int)DriverSendData->Kp;
		TxMessage.Data[3]=(int)((DriverSendData->Kp-(int)DriverSendData->Kp)*100);
		TxMessage.Data[4]=(int)DriverSendData->Ki;
		TxMessage.Data[5]=(int)((DriverSendData->Ki-(int)DriverSendData->Ki)*100);
		TxMessage.Data[6]=DriverSendData->Volt;
		TxMessage.Data[7]=DriverSendData->Tc;
		CAN_Transmit(CAN1, &TxMessage);
		return 3;
	}
	else
	{
		return 0;
	}
}

unsigned char WQSReadWriteData(struct WQSDataStruct *WQSSendData)
{
	CanTxMsg TxMessage;
	if (WQSSendData->WQSFi == WQSFrameWrite)
	{
		TxMessage.ExtId=WQSFrameWrite;					 //使用的扩展ID
		TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
		TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
		TxMessage.DLC = 0;
		CAN_Transmit(CAN1, &TxMessage);
		return 1;
	}
	else if (WQSSendData->WQSFi == WQSFrameRead)
	{
		/* 发送的是水质信息采集帧 */
		TxMessage.ExtId=WQSFrameRead;					 
		TxMessage.IDE=CAN_ID_EXT;		
		/* 该帧的基本类型为数据帧 */
		TxMessage.RTR=CAN_RTR_DATA;				
		TxMessage.DLC = 1;
		/* 传感器数据通道 */
		TxMessage.Data[0] = WQSSendData->WQSchannel;
		CAN_Transmit(CAN1, &TxMessage);
		return 2;
	}
	else
	{
		return 0;
	}
}

unsigned char GliderReadWriteData(struct GliderDataStruct *GliderSendData)
{
	CanTxMsg TxMessage;
	union int2char i2c;
	if (GliderSendData->GLIDERFi == GliderFrameWrite)
	{
		TxMessage.ExtId=GliderFrameWrite;					 //使用的扩展ID
		TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
		TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
		TxMessage.DLC = 5;
		switch(GliderSendData->Gliderchannel)
		{
				case GliderAng:
					TxMessage.Data[0] = GliderSendData->Gliderchannel;
					i2c.num = GliderSendData->angSink;
					TxMessage.Data[1] = i2c.byte[0];
					TxMessage.Data[2] = i2c.byte[1];
					i2c.num = GliderSendData->angFloat;
					TxMessage.Data[1] = i2c.byte[0];
					TxMessage.Data[2] = i2c.byte[1];
					break;
				case GliderTim:
					TxMessage.Data[0] = GliderSendData->Gliderchannel;
					i2c.num = GliderSendData->timSink;
					TxMessage.Data[1] = i2c.byte[0];
					TxMessage.Data[2] = i2c.byte[1];
					i2c.num = GliderSendData->timFloat;
					TxMessage.Data[1] = i2c.byte[0];
					TxMessage.Data[2] = i2c.byte[1];
					break;
				case GliderDeep:
					TxMessage.Data[0] = GliderSendData->Gliderchannel;
					i2c.num = GliderSendData->deepInit;
					TxMessage.Data[1] = i2c.byte[0];
					TxMessage.Data[2] = i2c.byte[1];
					i2c.num = GliderSendData->deepEnd;
					TxMessage.Data[1] = i2c.byte[0];
					TxMessage.Data[2] = i2c.byte[1];
					break;	
		}
		CAN_Transmit(CAN1, &TxMessage);
		return 1;
	}
	else if (GliderSendData->GLIDERFi == GliderFrameRead)
	{
		/* 发送的是滑翔机信息采集帧 */
		TxMessage.ExtId=FrameID_R_GliderRead;					 
		TxMessage.IDE=CAN_ID_EXT;		
		/* 该帧的基本类型为数据帧 */
		TxMessage.RTR=CAN_RTR_DATA;				
		TxMessage.DLC = 1;
		/* 传感器数据通道 */
		TxMessage.Data[0] = GliderSendData->Gliderchannel;
		CAN_Transmit(CAN1, &TxMessage);
		return 2;
	}
	else
	{
		return 0;
	}
}
/********************************************************************/
void DriverSalveHander(void)
{
	unsigned char i;
	CanRxMsg RxMessage;
	unsigned char TEMP[8];
	struct DriverDataStruct *DriverReciveData;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
  if((RxMessage.ExtId==FrameID_W_DriverWrite) && (RxMessage.RTR==CAN_RTR_DATA) && (RxMessage.DLC==8))
  {
		//receive
		DriverReciveData->Volocity = (RxMessage.Data[0] << 8) | RxMessage.Data[1];
		DriverReciveData->Kp = (float)(RxMessage.Data[2] + ((float)RxMessage.Data[3]/100));
		DriverReciveData->Ki = (float)(RxMessage.Data[4] + ((float)RxMessage.Data[5]/100));
		DriverReciveData->Volt = RxMessage.Data[6];
		DriverReciveData->Tc = RxMessage.Data[7];
		for (i = 0; i < 8; i++)
		{
			USART1_SendByte(RxMessage.Data[i]);
		}
  }
	  /* ExtId扩展标识 数据帧 数据长度匹配 */
  else if ((RxMessage.ExtId==FrameID_R_DriverMotion) && (RxMessage.RTR==CAN_RTR_REMOTE))
  {

	}
	else if ((RxMessage.ExtId==FrameID_R_DriverCapcity) && (RxMessage.RTR==CAN_RTR_REMOTE))
	{

	}
}

extern int GliderBox[8];
void GliderSalveHander(void)
{
	CanRxMsg RxMessage;
	CanRxMsg TxMessage;
	union int2char i2c;
	int temp;
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	switch(RxMessage.ExtId)
	{
		case FrameID_R_GliderRead:
			switch(RxMessage.Data[0])
			{
				case GliderAng:
					i2c.byte[0] = RxMessage.Data[1];
					i2c.byte[1] = RxMessage.Data[2];
					GliderBox[0] = i2c.num;
					i2c.byte[0] = RxMessage.Data[3];
					i2c.byte[1] = RxMessage.Data[4];
					GliderBox[1] = i2c.num;
					break;
				case GliderTim:
					i2c.byte[0] = RxMessage.Data[1];
					i2c.byte[1] = RxMessage.Data[2];
					GliderBox[2] = i2c.num;
					i2c.byte[0] = RxMessage.Data[3];
					i2c.byte[1] = RxMessage.Data[4];
					GliderBox[3] = i2c.num;
					break;
				case GliderDeep:
					i2c.byte[0] = RxMessage.Data[1];
					i2c.byte[1] = RxMessage.Data[2];
					GliderBox[4] = i2c.num;
					i2c.byte[0] = RxMessage.Data[3];
					i2c.byte[1] = RxMessage.Data[4];
					GliderBox[5] = i2c.num;
					break;
				case ReadDeep:
					
//				TxMessage.ExtId=0x1331;					 //使用的扩展ID
//				TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
//				TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
//				TxMessage.DLC=4;							 //数据长度为2字节
//				TxMessage.Data[0]=0xff;		
//						
//				TxMessage.Data[1]=0xc2;
//				TxMessage.Data[2]=0xc3;
//				TxMessage.Data[3]=0xc4;
//				TxMessage.Data[4]=0xc1;
//				CAN_Transmit(CAN1, &TxMessage);
					break;
			}
			break;
	}
}

void MatserHandler(void)
{
	union float2char f2c;
	CanRxMsg RxMessage;
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	
	switch(RxMessage.ExtId)
	{
		case FrameID_R_DriverMotion:
			if (RxMessage.RTR == CAN_RTR_DATA && RxMessage.DLC == 8)
			{

			}
			break;
		case FrameID_R_DriverCapcity:
			if (RxMessage.RTR == CAN_RTR_DATA && RxMessage.DLC == 4)
			{

			}
			break;
		case FrameID_R_WQSRead:
			if (RxMessage.RTR == CAN_RTR_DATA && RxMessage.DLC == 5)
			{
				switch(RxMessage.Data[0])
				{
					case WQSPH://0x01 0000 0001
						f2c.byte[0] = RxMessage.Data[1];
						f2c.byte[1] = RxMessage.Data[2];
						f2c.byte[2] = RxMessage.Data[3];
						f2c.byte[3] = RxMessage.Data[4];
						TempWQS[0] = f2c.num;
						WQS_flag |= WQSPH;
						break;
					case WQSORP://0x02 0000 0010
						f2c.byte[0] = RxMessage.Data[1];
						f2c.byte[1] = RxMessage.Data[2];
						f2c.byte[2] = RxMessage.Data[3];
						f2c.byte[3] = RxMessage.Data[4];
						TempWQS[1] = f2c.num;
						WQS_flag |= WQSORP;
						break;
					case WQSEC://0x03 0000 0011
						f2c.byte[0] = RxMessage.Data[1];
						f2c.byte[1] = RxMessage.Data[2];
						f2c.byte[2] = RxMessage.Data[3];
						f2c.byte[3] = RxMessage.Data[4];
						TempWQS[2] = f2c.num;
						WQS_flag |= WQSEC;
						break;
					case WQSTUR:
						f2c.byte[0] = RxMessage.Data[1];
						f2c.byte[1] = RxMessage.Data[2];
						f2c.byte[2] = RxMessage.Data[3];
						f2c.byte[3] = RxMessage.Data[4];
						TempWQS[3] = f2c.num;
						WQS_flag |= WQSTUR;
						break;
					case WQSALL:
						break;
				}
			}
			break;
	}
}

void WQS2RaspberryPi(void)
{
		signed char i, k;
		union float2char f2c;
	
		UploadData.FrameHeadI = 0xca;
		UploadData.FrameHeadII = 0xac;
		UploadData.FrameIDI  = FrameID_R_WQSRead & 0xff;
		UploadData.FrameIDII = FrameID_R_WQSRead >> 8;
		UploadData.DataLength= 4*4 + 5;
		UploadData.DataSum   = UploadData.FrameIDI + UploadData.FrameIDII + UploadData.DataLength;
		
		USART1_SendByte(UploadData.FrameHeadI);
		USART1_SendByte(UploadData.FrameHeadII);
		USART1_SendByte(UploadData.FrameIDI);
		USART1_SendByte(UploadData.FrameIDII);
		USART1_SendByte(UploadData.DataLength);
	
		for (k = 0; k < 4; k++)
		{
			f2c.num = TempWQS[k];
			for (i = 0; i < 4; i++)
			{
				USART1_SendByte(f2c.byte[i]);
				UploadData.DataSum += f2c.byte[i];
			}
		}
		USART1_SendByte(UploadData.DataSum);
		UploadData.DataSum = 0x00;
		USART1_SendByte(0xCC);
}

/********************************************************************************************************************************************************************************/
/*
主机向从机发送写指令，准备读取相应的数据
id 要读取的板子的标号 channel 要读取的数据组数
*/
void Host_write(unsigned short int ID, unsigned char groupid)
{
	CanTxMsg TxMessage;

		/* 发送的是接收的ID */
	TxMessage.ExtId=ID;		
	TxMessage.IDE=CAN_ID_EXT;		
		/* 该帧的基本类型为数据帧 */
	TxMessage.RTR=CAN_RTR_DATA;				
	TxMessage.DLC = 1;
		/* 传感器数据通道 */
	TxMessage.Data[0] = groupid;
	
	CAN_Transmit(CAN1, &TxMessage);
}
/* 
主机(滑翔机控制板)读取从机(传感器采集板, 驱动板)数据
*/
void Host_Read()
{
	CanRxMsg RxMessage;
	union float2char f2c;
	
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	/* 0x1332 传感器采集板读  */
//	UART4_SendByte(RxMessage.Data[0]);
	if (RxMessage.ExtId == 0x1332 && RxMessage.RTR == CAN_RTR_DATA)
	{
		switch (RxMessage.Data[0])
		{
			
			case Label_PH:
				
				f2c.byte[0] = RxMessage.Data[1];
				f2c.byte[1] = RxMessage.Data[2];
				f2c.byte[2] = RxMessage.Data[3];
				f2c.byte[3] = RxMessage.Data[4];
				TempWQS[0] = f2c.num; 
				SensorBoard_Flag |= 0x01;
				break;
			case Label_ORP:
				f2c.byte[0] = RxMessage.Data[1];
				f2c.byte[1] = RxMessage.Data[2];
				f2c.byte[2] = RxMessage.Data[3];
				f2c.byte[3] = RxMessage.Data[4];
				TempWQS[1] = f2c.num;
				SensorBoard_Flag |= 0x02;
				break;
			case Label_EC:
				f2c.byte[0] = RxMessage.Data[1];
				f2c.byte[1] = RxMessage.Data[2];
				f2c.byte[2] = RxMessage.Data[3];
				f2c.byte[3] = RxMessage.Data[4];
				TempWQS[2] = f2c.num;
				SensorBoard_Flag |= 0x04;
				break;
			case Label_TUR:
				f2c.byte[0] = RxMessage.Data[1];
				f2c.byte[1] = RxMessage.Data[2];
				f2c.byte[2] = RxMessage.Data[3];
				f2c.byte[3] = RxMessage.Data[4];
				TempWQS[3] = f2c.num;
				SensorBoard_Flag |= 0x08;
//								if (SensorBoard_Flag == 0x0f)
//								{
									SensorBoard_Flag = 0x00;
									Salve_Sensor_SendGroup1();
//								}
				break;
			case Label_IRL:
				f2c.byte[0] = RxMessage.Data[1];
			f2c.byte[1] = RxMessage.Data[2];
				f2c.byte[2] = RxMessage.Data[3];
				f2c.byte[3] = RxMessage.Data[4];
				TempIRS[0] = f2c.num;
				SensorBoard_Flag |= 0x10;
				break;
			case Label_IRR:
				f2c.byte[0] = RxMessage.Data[1];
				f2c.byte[1] = RxMessage.Data[2];
				f2c.byte[2] = RxMessage.Data[3];
				f2c.byte[3] = RxMessage.Data[4];
				TempIRS[1] = f2c.num;
				SensorBoard_Flag |= 0x20;
//								if (SensorBoard_Flag == 0x30)
//								{
									SensorBoard_Flag = 0x00;
									Salve_Sensor_SendGroup2();
//								}	
				break;
			default:
				break;
		}
	}				/* 其它功能，未使用保留 */
	else if(RxMessage.ExtId == 0x1333 && RxMessage.RTR == CAN_RTR_DATA)
	{
		
	}
	else
	{
		
	}
}
/********************************************************************************************************************************************************************************/
float BigSmallEx(float data)
{
	union float2char f2c, f2ct;
	unsigned char temp;
	f2c.num = data;
	f2ct.byte[0] = f2c.byte[3];
	f2ct.byte[1] = f2c.byte[2];
	f2ct.byte[2] = f2c.byte[1];
	f2ct.byte[3] = f2c.byte[0];
	
	return f2ct.num;
}
/* 发送的第1组数据:IRL IRR */
void Salve_Sensor_SendGroup1(void)
{
	unsigned char i = 0, j = 0;
	unsigned char tempp = 0;
	union float2char f2c;
	
	UART4_SendByte(0xca); 
	UART4_SendByte(0xac); 
	UART4_SendByte(0x32); tempp += 0x32;
	UART4_SendByte(0x13); tempp += 0x13;
	UART4_SendByte(4*4+6); tempp += 4*4+6;
	UART4_SendByte(1); tempp += 1;
	
	for (i = 0; i < 4; i++)
	{
		f2c.num = TempWQS[i];
		for (j = 0; j < 4; j++)
		{
			UART4_SendByte(f2c.byte[j]);
			tempp += f2c.byte[j];
		}
	}
	UART4_SendByte(tempp);tempp = 0;
	UART4_SendByte(0xcc);
}
/* 发送的第二组数据:IRL IRR */
void Salve_Sensor_SendGroup2(void)
{
	unsigned char i = 0, j = 0;
	unsigned char tempp = 0;
	union float2char f2c;
	
	UART4_SendByte(0xca); 
	UART4_SendByte(0xac); 
	UART4_SendByte(0x32); tempp += 0x32;
	UART4_SendByte(0x13); tempp += 0x13;
	UART4_SendByte(4*2+6); tempp += 4*2+6;
	UART4_SendByte(2); tempp += 2;
	
	for (i = 0; i < 2; i++)
	{
		f2c.num = TempIRS[i];//BigSmallEx(3.14);
		for (j = 0; j < 4; j++)
		{
			UART4_SendByte(f2c.byte[j]);
			tempp += f2c.byte[j];
		}
	}
	UART4_SendByte(tempp);tempp = 0;
	UART4_SendByte(0xcc);
}

//extern float Angle[3];
//extern float fLon, fLat, fGpsHeight, fGPSyaw, fGPSvelocity;
//extern float depth_ms, temper_ms, depth_v;
//extern int position_c;

float tempNUM[6];

void Host2RaspberryPi(unsigned short int groupid)
{
	unsigned char i = 0, j = 0;
	unsigned char tempp = 0, paralen = 0;
	union float2char f2c;
	
	
	UART4_SendByte(0xca); 
	UART4_SendByte(0xac); 
	UART4_SendByte(0x30); tempp += 0x30;
	UART4_SendByte(0x13); tempp += 0x13;
	
	if (groupid == 1)
	{
		paralen = 3;
//		tempNUM[0] = Angle[0];
//		tempNUM[1] = Angle[1];
//		tempNUM[2] = Angle[2];
		UART4_SendByte(4*paralen+6); tempp += 4*paralen+6;
		UART4_SendByte(1); tempp += 1;
		
		for (i = 0; i < paralen; i++)
		{
			f2c.num = tempNUM[i];//BigSmallEx(tempNUM[i]);
			for (j = 0; j < 4; j++)
			{
				UART4_SendByte(f2c.byte[j]);
				tempp += f2c.byte[j];
			}
		}
	}
	else if(groupid == 2)
	{
		paralen = 6;
//		tempNUM[0] = fLon;
//		tempNUM[1] = fLat;
//		tempNUM[2] = fGpsHeight;
//		tempNUM[3] = fGPSyaw;
//		tempNUM[4] = fGPSvelocity;
		tempNUM[5] = 3.1415;
		UART4_SendByte(4*paralen+6); tempp += 4*paralen+6;
		UART4_SendByte(2); tempp += 2;
		
		for (i = 0; i < paralen; i++)
		{
			f2c.num = tempNUM[i];
			for (j = 0; j < 4; j++)
			{
				UART4_SendByte(f2c.byte[j]);
				tempp += f2c.byte[j];
			}
		}
	}
	else if (groupid == 3)
	{
		paralen = 6;
//		tempNUM[0] = depth_ms;
//		tempNUM[1] = temper_ms;
//		tempNUM[2] = depth_v;
//		tempNUM[3] = BoardVolt/4096.0f * 3.3f * 6.1;
//		tempNUM[4] = 3.1415;
//		tempNUM[5] = (float)position_c;	
		UART4_SendByte(4*paralen+6); tempp += 4*paralen+6;
		UART4_SendByte(3); tempp += 3;
		
		for (i = 0; i < paralen; i++)
		{
			f2c.num = tempNUM[i];
			for (j = 0; j < 4; j++)
			{
				UART4_SendByte(f2c.byte[j]);
				tempp += f2c.byte[j];
			}
		}
	}
	else
	{
		
	}
	UART4_SendByte(tempp);tempp = 0;
	UART4_SendByte(0xcc);
}

unsigned char tempHPS[50];
extern int check_angle;
extern short int IR_offst_L, IR_offst_R;
extern float outputVel;
float djangle = 0;
unsigned char djflag = 0;
extern struct sDEBUGParameter sDebug;
void HostsParameterSet(unsigned short int ID, unsigned char groupid)
{
	CanTxMsg TxMessage;
	union float2char f2c;
	
	memcpy(&stcHostsPara, &tempHPS ,23);
	if (djflag == 1)
	{
		f2c.num = djangle;//sDebug.debugYaw;
		djflag = 0;
	}
	else
	{
		f2c.num = stcHostsPara.setDJ;
	}
	
	check_angle = (int)stcHostsPara.setPitch;
	IR_offst_L  = (short int)stcHostsPara.setPropeller_L;
	IR_offst_R  = (short int)stcHostsPara.setPropeller_R;
	//outputVel  = stcHostsPara.setRoll;
		/* 发送的是接收的ID */
	TxMessage.ExtId=0x1333;		
	TxMessage.IDE=CAN_ID_EXT;		
		/* 该帧的基本类型为数据帧 */
	TxMessage.RTR=CAN_RTR_DATA;				
	TxMessage.DLC = 5;
		/* 传感器数据通道 */
	TxMessage.Data[0] = groupid;
	
	TxMessage.Data[1] = f2c.byte[0];
	TxMessage.Data[2] = f2c.byte[1];
	TxMessage.Data[3] = f2c.byte[2];
	TxMessage.Data[4] = f2c.byte[3];
	CAN_Transmit(CAN1, &TxMessage);
}

/********************************************************************************************************************************************************************************/