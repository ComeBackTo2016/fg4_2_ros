#include "Control.h"
#include "stm32f10x.h"
#include "timer.h"
#include "bsp_SysTick.h"
#include "init.h"
#include "CRC.h"
#include "GPIOLIKE51.h"
#include "ADS1115_LowLevel.h"
#include "conmunication.h"
#include "string.h"
/****************************************************************************/
#define IIC_BUFFER_SIZE			40
#define FLAG_RECV_REG_ADDR	0
#define FLAG_RECV_DATA			1
#define FLAG_SEND_DATA			2

/* 寄存器地址宏定义 */
#define REG_SENSOR 0xB1
#define REG_LENGTH 0xB2
#define REG_READ   0xB3

/****接收数据类型定义*****/
#define CPG_Weiqi 0X00
#define CPG_Xiongqi 0X01
#define Static_Offset_1_Sub 0X02
#define Static_Offset_2_Sum 0X03
#define AUTU_ACTION 0X04
#define Static_Offset_3_Sum 0X05
#define Static_Offset_3_Sub 0X06
#define Clear_data 0X07

unsigned char IIC_Buffer[IIC_BUFFER_SIZE];
unsigned char *P_Data = (&IIC_Buffer[0] + 2);
unsigned char Data_Length = 0;
unsigned char Sensor_NO = 0;
  signed int  Check = 0;
unsigned char Count = 0;
unsigned char IIC_Reg_Flag = 0;
unsigned char Bottom_Data[84];
unsigned char Start_Add;
/****************************************************************************/
extern float Motor_Speed; 
extern float Motor_Log_degree_12;
extern float Motor_Log_degree_13;
extern float Motor_Amplitude_degree[3];    
extern float Motor_Dynamic_Offset_degree[3];

extern float Pectoral_Volcety;                 // 胸鳍速度
extern float Pectoral_Phase_Dif;               // 胸鳍相角差
extern float Pectoral_Amplitude[2];            // 幅值参数
extern float Pectoral_Dynamic_Offset[2];       // 动角

float SensorBox[21];                            // 传感器数据上传缓存区
union FLOAT2CHAR f2c;
unsigned char float2byte[4];
uint8_t chrTemp[6] = {0};
float Angle[3] = {0};
float AngInit[3] = {0};
int GliderBox[8];

#define FRAME_BYTE_FST_360 0xca
#define FRAME_BYTE_SEC_360 0xac
#define FRAME_BYTE_LST_360 0xcc
#define FRAME_HEAD_FLAG_360 0x01
#define FRAME_OVER_FLAG_360 0x02
#define BUFFER_SIZE_360 64

uint8_t rec_data_360[BUFFER_SIZE_360];
uint8_t rec_index_360 = 0x00;
uint8_t rec_flag_360 = 0x00;

uint8_t SumCheck_360(void);
uint8_t XORCheck_360(void);

/*************************************************************/
unsigned char* Send_float_to_byte(float temp);
float Receive_byte_to_float(unsigned char* temp);
void Python_Data_Of_Glider4_Process(void);
void Python_Data_Of_Tail4_Process(void);
void SendBack2xbox360(void);
void Python_Data_Of_Rom_Save(void);
/**
  * @brief  float数据2char型
	* @discrib 
  * @param  
  * @retval 
  */
unsigned char* Send_float_to_byte(float temp)
{
	union UFLOAT
	{
		float f;
		unsigned char byte[4];
	};
	union UFLOAT data;	
	int i;
	data.f = temp;
	for(i=0;i<4;i++)
	float2byte[i] = data.byte[i];
	return float2byte;
}

float Receive_byte_to_float(unsigned char* temp)
{
	union UFLOAT
	{
		float f;
		unsigned char byte[4];
	};
	union UFLOAT data;
	unsigned char i;
	for(i=0;i<4;i++)
	data.byte[i] = *(temp+i);
	return data.f;
}

int Receive_byte_to_int(unsigned char* temp)
{
	union UINT
	{
		int in;
		unsigned char byte[2];
	};
	union UINT data;
	unsigned char i;
	for(i=0;i<2;i++)
	data.byte[i] = *(temp+i);
	return data.in;
}

short CharToShort(unsigned char cData[])
{
	return ((short)cData[1]<<8)|cData[0];
}

/***************************************************串口解析程序（开始）************************************************************/
#define FRAME5_BYTE_FST 0xca
#define FRAME5_BYTE_SEC 0xac
#define FRAME5_BYTE_LST 0xcc
#define FRAME5_HEAD_FLAG 0x01
#define FRAME5_OVER_FLAG 0x02
#define BUFFER5_SIZE 80

volatile unsigned char rec5_data[BUFFER5_SIZE];
volatile unsigned char rec5_index = 0x00;
volatile unsigned char rec5_flag = 0x00;
void Commond5Process(void);
void GetUart5Frame(unsigned char data)
{
	unsigned char i;

	if (data == FRAME5_BYTE_FST)
	{
		rec5_flag = rec5_flag | FRAME5_HEAD_FLAG;
		rec5_data[rec5_index] = data;
		rec5_index++;
	}
	else if (data == FRAME5_BYTE_SEC)
	{
		if (rec5_flag & FRAME5_HEAD_FLAG)
		{
			rec5_flag = rec5_flag & ~FRAME5_OVER_FLAG;
			rec5_index = 0;
		}
		else
		{
			rec5_data[rec5_index] = data;
			rec5_index++;
			rec5_flag = rec5_flag & ~FRAME5_HEAD_FLAG;
		}
	}
	else
	{
		rec5_data[rec5_index] = data;
		rec5_flag &= ~FRAME5_HEAD_FLAG;
		rec5_index++;
		if (rec5_index == rec5_data[2])
		{

			if (rec5_data[rec5_index-1] == FRAME5_BYTE_LST)
			{
				rec5_flag |= FRAME5_OVER_FLAG;
				Commond5Process();	
			}
		}
	}
	
	if (rec5_index == BUFFER5_SIZE)
	{
		rec5_index--;
	}
}

#define SENSOR0_PARAMETER 0x1330
#define SENSOR1_PARAMETER 0x1331
#define SENSOR2_PARAMETER 0x1332
int16_t frame5_id;
unsigned char Sum5Check(void)
{
	unsigned char i = 0;
	unsigned int sum5_byte = 0x00;
	
	for (i = 0; i < rec5_data[2] - 2; i++)
	{
		sum5_byte += rec5_data[i];
	}

	if ((sum5_byte & 0xff) == rec5_data[rec5_data[2] - 2])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
void Commond5Process(void)
{
	if (rec5_flag & FRAME5_OVER_FLAG)
	{
		rec5_flag &= ~FRAME5_OVER_FLAG;
		if (Sum5Check())
		{
			frame5_id = (rec5_data[1] << 8) | rec5_data[0];
			switch(frame5_id)
			{
				case SENSOR0_PARAMETER:

				break;
				case SENSOR1_PARAMETER:	
				    
				break;
				case SENSOR2_PARAMETER:
					
				break;
				default:
					break;
			}
		}
	}
}

/**************************************************************************
函数功能:接收字符型数据帧分类处理
入口参数: 串口接收到的每一帧字符数据数据
备    注:协议格式：'c' 'a' length flag data data sum 'b'
作    者:何宇帆(2016-09-20)
**************************************************************************/
#define FRAME_CHAR_FST 'c'
#define FRAME_CHAR_SEC 'a'
#define FRAME_CHAR_LST 'b'
#define FRAME_HEAD_FLAG 0x01
#define FRAME_OVER_FLAG 0x02
#define FRAME_LENGTH_FLAG 0x03
#define FRAME_NEW_FLAG 0x04
#define BUFFER_SIZE 88

volatile unsigned char rec_char_data[BUFFER_SIZE];
volatile unsigned char rec_char_index = 0;
volatile unsigned char rec_char_flag = 0x00;
volatile unsigned char rec_length = 0;

void FrameCharGet(unsigned char data)
{
	unsigned char i;
	unsigned char ch;
//	USART1_SendByte(data);
	
	if (data == 'c')																	// 帧首判断'c',说明可能一帧开始传输
	{
		rec_char_flag |= FRAME_HEAD_FLAG;							// 帧首标志位置位
		rec_char_data[rec_char_index] = data;						// 保存当前接受的字符
		rec_char_index++;																// 数组索引后移一位
	}
	else if (data == 'a')															// 当前接收字符为'a'
	{
		if (rec_char_flag & FRAME_HEAD_FLAG)						// 且上一字符为'a'
		{
			PCout(13) = 0;
			rec_char_flag &= ~FRAME_OVER_FLAG;						// 数据帧结束标志位清零
			rec_char_flag |= FRAME_LENGTH_FLAG;						// 数据帧长度标志位置位
			rec_char_index = 0;														// 数组索引清零，从新开始接收有用数据
		}
		else
		{
			rec_char_data[rec_char_index] = data;					// 说明当前字符为数据不是帧首
			rec_char_index++;
			rec_char_flag &= ~FRAME_HEAD_FLAG;						// 数据帧帧头标志位清零
		}
	}
	else
	{
		rec_char_data[rec_char_index] = data;						// 非帧头数据直接保存
		rec_char_flag &= ~FRAME_HEAD_FLAG;							// 帧头标志位清零
		rec_char_index++;
		/*数组索引为2(0，1为数据帧长度位)，并且长度标志位已经置位*/
		if (rec_char_index == 2 && (rec_char_flag & FRAME_LENGTH_FLAG))
		{
			/*计算当前帧数据长度*/
			rec_length = (rec_char_data[0] - '0')*10 + (rec_char_data[1] - '0');
			rec_char_flag &= ~FRAME_LENGTH_FLAG;
		}

		if (rec_char_index == rec_length)
		{
			if (rec_char_data[rec_char_index - 1] == 'b')
			{
				
				rec_char_flag |= FRAME_OVER_FLAG;
                
				/* 此处获取机器鱼标识ID. '1' '2' '3' 分别表示不同机器鱼编号 */
				if (rec_char_data[2] == '4')
				{
						ch = rec_char_data[3];
						switch(ch)
						{
								case 'a':
										Python_Data_Of_Glider4_Process();		
										break;
								case 'b':
										Python_Data_Of_Tail4_Process();
										break;
								case 'c':
										/* Data save */
										Python_Data_Of_Rom_Save();
										break;
								case 'd':
									
										break;
								default:
										break;
						}
				}
			}
		}
	}
	if (rec_char_index == BUFFER_SIZE)
	{
		rec_char_index--;
	}
}

uint8_t XORCheck_360(void)
{
	uint8_t i = 0;
	uint16_t xor_byte = 0x00;
	xor_byte = rec_data_360[0];
	for (i = 1; i < rec_data_360[2] - 2; i++)
	{
		xor_byte ^= rec_data_360[i];
	}

	if ((xor_byte & 0xff) == rec_data_360[rec_data_360[2] - 2])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t FrameGet_360(uint8_t data)
{
	uint8_t i;

	if (data == FRAME_BYTE_FST_360)
	{
		rec_flag_360 = rec_flag_360 | FRAME_HEAD_FLAG_360;
		rec_data_360[rec_index_360] = data;
		rec_index_360++;
	}
	else if (data == FRAME_BYTE_SEC_360)
	{
		if (rec_flag_360 & FRAME_HEAD_FLAG_360)
		{
			rec_flag_360 = rec_flag_360 & ~FRAME_OVER_FLAG_360;
			rec_index_360 = 0;
		}
		else
		{
			rec_data_360[rec_index_360] = data;
			rec_index_360++;
			rec_flag_360 = rec_flag_360 & ~FRAME_HEAD_FLAG_360;
		}
	}
	else
	{
		rec_data_360[rec_index_360] = data;
		rec_flag_360 &= ~FRAME_HEAD_FLAG_360;
		rec_index_360++;

		if (rec_index_360 == rec_data_360[2])
		{

			if (rec_data_360[rec_index_360-1] == FRAME_BYTE_LST_360)
			{
				rec_flag_360 |= FRAME_OVER_FLAG_360;
				/************************/	
				if (XORCheck_360())
				{
					return rec_data_360[1];
				}
				else
				{
					return 0x01;
				}
				/************************/	
			}
		}
		else if(rec_index_360 == BUFFER_SIZE_360)
		{
			rec_index_360--;
			return 0x01;
		}
		else
		{return 0x01;}
	}
}


FrameXbox360DJData frame360djrec;
FrameXbox360MTData frame360mtrec;
/* 人机操作模式切换标志 0:机器操作模式(默认) 1:人工操作模式 */
uint8_t OperateMode = 0;
void Xbox360DataProcess(uint8_t data)
{
	uint8_t datatemp = 0, modetemp = 0;
	union float2byte
	{
		float f;
		uint8_t b[4];
	}f2b;
	CanTxMsg TxMessage;
	datatemp = FrameGet_360(data);
	if (datatemp == 0x37)
	{
		/* 数据ID: dj_amp,dj_dseta, dj_v,  */
		if (rec_data_360[3] == 0x00)
		{
			/* 切换为人工操作模式 */
			if (OperateMode == 1)
			{
				memcpy(&frame360djrec, &rec_data_360[4], 2);//sizeof(frame360djrec)
				TxMessage.ExtId = 0X1330;
				TxMessage.IDE=CAN_ID_EXT;
				TxMessage.RTR=CAN_RTR_DATA;
				TxMessage.DLC = 7;
				TxMessage.Data[0] = 0x00;
				if ((frame360djrec.dj_amp < 0) || (frame360djrec.dj_amp > 60))
				{frame360djrec.dj_amp = 0;}
				TxMessage.Data[1] = frame360djrec.dj_amp;
				if (((frame360djrec.dj_dseta + (float)frame360djrec.dj_amp / 2.0) < -60) || ((frame360djrec.dj_dseta + (float)frame360djrec.dj_amp / 2.0) > 60))
				{frame360djrec.dj_dseta = 0;}
				TxMessage.Data[2] = frame360djrec.dj_dseta;
				f2b.b[0] = rec_data_360[6];
				f2b.b[1] = rec_data_360[7];
				f2b.b[2] = rec_data_360[8];
				f2b.b[3] = rec_data_360[9];
				frame360djrec.dj_vol =f2b.f;
				TxMessage.Data[3] = f2b.b[0];
				TxMessage.Data[4] = f2b.b[1];
				TxMessage.Data[5] = f2b.b[2];
				TxMessage.Data[6] = f2b.b[3];
				CAN_Transmit(CAN1, &TxMessage);
			}
		}
		else if (rec_data_360[3] == 0x01)
		{
			if (OperateMode == 1)
			{
				memcpy(&frame360mtrec, &rec_data_360[4], sizeof(frame360mtrec));
				TxMessage.ExtId = 0X1330;
				TxMessage.IDE=CAN_ID_EXT;
				TxMessage.RTR=CAN_RTR_DATA;
				TxMessage.DLC = 5;
				TxMessage.Data[0] = 0x01;
				/* 电调输出信号限幅 */
				if(frame360mtrec.motor_l >= 1980)
				{frame360mtrec.motor_l = 1980;}
				else if(frame360mtrec.motor_l <= 1020)
				{frame360mtrec.motor_l = 1020;}; 
				if(frame360mtrec.motor_r >= 1980)
				{frame360mtrec.motor_r = 1980;}
				else if(frame360mtrec.motor_r <= 1020)
				{frame360mtrec.motor_r = 1020;}; 
				TxMessage.Data[1] = frame360mtrec.motor_l & 0xff;
				TxMessage.Data[2] = frame360mtrec.motor_l >> 8;
				TxMessage.Data[3] = frame360mtrec.motor_r & 0xff;
				TxMessage.Data[4] = frame360mtrec.motor_r >> 8;
				CAN_Transmit(CAN1, &TxMessage);
			}
		}
		else if (rec_data_360[3] == 0x02)
		{
				modetemp = rec_data_360[4];
				if (modetemp == 0)
				{OperateMode = 0,frame360mtrec.motor_l = 1500, frame360mtrec.motor_r = 1500;}
				else if (modetemp == 1)
				{OperateMode = 1;}
				else
				{OperateMode = 1;}
		}
		SendBack2xbox360();
	}
	else if (datatemp == 0x01)
	{
		
	}
	else
	{}
}

typedef struct{
	int16_t gps_vel;
	int16_t gps_dir;
	float gps_longitude;
	float latitude;
}GPSdata;

extern float BatIV[2];
extern float DistanceIR;
extern float LeakageData[4];
void SendBack2xbox360(void)
{
	union float2byte
	{
		float f;
		uint8_t b[4];
	}f2b;
	int16_t temp16 = 0;
	float tempf = 0;
	static uint8_t databuf[32] = {0};
	uint8_t i = 0, j = 0, temp8 = 0;
	GPSdata myGPS;
	
	databuf[i] = 0x13;
	i++, databuf[i] = 0x37;
	i++, databuf[i] = 44;
	i++, databuf[i] = 0x01;
	i++, databuf[i] = (uint8_t)(myGPS.gps_vel);
	i++, databuf[i] = (uint8_t)(myGPS.gps_vel * 100 % 100);
	i++, databuf[i] = myGPS.gps_dir >> 8;
	i++, databuf[i] = myGPS.gps_dir & 0xff;
	f2b.f = myGPS.gps_longitude;
	i++, databuf[i] = f2b.b[0];
	i++, databuf[i] = f2b.b[1];
	i++, databuf[i] = f2b.b[2];
	i++, databuf[i] = f2b.b[3];
	f2b.f = myGPS.latitude;
	i++, databuf[i] = f2b.b[0];
	i++, databuf[i] = f2b.b[1];
	i++, databuf[i] = f2b.b[2];
	i++, databuf[i] = f2b.b[3];
	i++, databuf[i] = chrTemp[0];
	i++, databuf[i] = chrTemp[1];
	i++, databuf[i] = chrTemp[2];
	i++, databuf[i] = chrTemp[3];
	i++, databuf[i] = chrTemp[4];
	i++, databuf[i] = chrTemp[5];
	i++, databuf[i] = ((uint8_t)(BatIV[0]));
	i++, databuf[i] = (uint8_t)((BatIV[0] - (uint8_t)BatIV[0]) * 100);
	i++, databuf[i] = ((uint16_t)(BatIV[1]) >> 8);
	i++, databuf[i] = ((uint16_t)(BatIV[1]) & 0xff);
	i++, databuf[i] = (uint8_t)((DistanceIR - 30.0) / 80.0 * 100);
	temp16 = TIM2->CNT;
	i++, databuf[i] = (uint8_t)((temp16 - 5000) / 100);
	i++, databuf[i] = (1 == ((uint16_t)LeakageData[0] % 4000)) * (1 == ((uint16_t)LeakageData[1] % 4000)) *
	(1 == ((uint16_t)LeakageData[2] % 4000)) * (1 == ((uint16_t)LeakageData[3] % 4000));
	temp8 = databuf[0];
	for (j = 1; j <= i; j++)
	{
		temp8 ^= databuf[j];
	}
	i++, databuf[i] = temp8;
	UART5_SendByte(0xca);
	UART5_SendByte(0xac);
	for (j = 0; j <= i; j++)
	{
		UART5_SendByte(databuf[j]);
	}
	UART5_SendByte(0xcc);
}

typedef struct {
	uint8_t down_ang;
	uint8_t up_ang;
	uint8_t mid_ang;
	uint16_t down_tim;
	uint16_t keep_tim;
	uint16_t up_tim;
	uint16_t water_in;
	uint16_t water_mid;
	uint16_t water_ex;
	uint8_t glider_flag;
	uint8_t tail_freq;
	uint8_t tail_amp;
	 int8_t tail_dyn;
	uint16_t tail_vol_l;
	uint16_t tail_vol_r;
	 int16_t tail_vol_offest;
} FramePythonData;
FramePythonData framepythondata;

void PythonDataStructInit(void)
{
			framepythondata.glider_flag = 0;
			framepythondata.down_ang = 0;
			framepythondata.mid_ang = 50;
			framepythondata.up_ang  =  0;
			framepythondata.down_tim = 0;
			framepythondata.keep_tim = 0;
			framepythondata.up_tim  =  0;
			framepythondata.water_ex = 0;
			framepythondata.water_mid = 500;
			framepythondata.water_in = 0;
			framepythondata.tail_amp = 0;
			framepythondata.tail_dyn = 0;
			framepythondata.tail_freq= 0;
			framepythondata.tail_vol_l = 1500;
			framepythondata.tail_vol_r = 1500;
			framepythondata.tail_vol_offest = 0;
}

void Python_Data_Of_Glider4_Process(void)
{
	uint8_t index, i;
	uint16_t _4th, _3th, _2nd, _1st;

	index = 4;

	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[4]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[5]);
	_2nd = (rec_char_data[index + 2] - '0') *   10;		//USART1_SendByte(rec_char_data[6]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[7]);
	framepythondata.down_ang = (int)((_4th + _3th + _2nd + _1st) - 2000);		//USART1_SendByte('X');		USART1_SendByte(' ');

	index += 4;	// index = 8
	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[8]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[9]);
	_2nd = (rec_char_data[index + 2] - '0')*   10;		//USART1_SendByte(rec_char_data[10]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[11]);
	framepythondata.mid_ang = (int)(_4th + _3th + _2nd + _1st) - 2000;		//USART1_SendByte('Y');		USART1_SendByte(' ');
	
	index += 4;	// index = 8
	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[8]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[9]);
	_2nd = (rec_char_data[index + 2] - '0')*   10;		//USART1_SendByte(rec_char_data[10]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[11]);
	framepythondata.up_ang = (int)(_4th + _3th + _2nd + _1st) - 2000;		//USART1_SendByte('Y');		USART1_SendByte(' ');

	index += 4;		// index = 12
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.down_tim = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 16
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.keep_tim = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.up_tim = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.water_in = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.water_mid = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.water_ex = (int)(_4th + _3th + _2nd + _1st) - 2000;
    
	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.glider_flag = (int)(_4th + _3th + _2nd + _1st) - 2000;
	for (i = 0; i < rec_char_index; i++)
	{
		rec_char_data[i] = 0x00;
	} 
}

void Python_Data_Of_Tail4_Process(void)
{
	union float2byte
	{
		float f;
		uint8_t b[4];
	}f2b;
	uint8_t index, i;
	uint16_t _4th, _3th, _2nd, _1st;
	CanTxMsg TxMessage;
	index = 4;

	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[4]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[5]);
	_2nd = (rec_char_data[index + 2] - '0') *   10;		//USART1_SendByte(rec_char_data[6]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[7]);
	framepythondata.tail_freq = (int)((_4th + _3th + _2nd + _1st) - 2000);		//USART1_SendByte('X');		USART1_SendByte(' ');

	index += 4;	// index = 8
	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[8]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[9]);
	_2nd = (rec_char_data[index + 2] - '0')*   10;		//USART1_SendByte(rec_char_data[10]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[11]);
	framepythondata.tail_amp = (int)(_4th + _3th + _2nd + _1st) - 2000;		//USART1_SendByte('Y');		USART1_SendByte(' ');

	index += 4;		// index = 12
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.tail_dyn = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 16
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.tail_vol_l = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.tail_vol_r = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	framepythondata.tail_vol_offest = (int)(_4th + _3th + _2nd + _1st) - 2000;	
	
				TxMessage.ExtId = 0X1330;
				TxMessage.IDE=CAN_ID_EXT;
				TxMessage.RTR=CAN_RTR_DATA;
				TxMessage.DLC = 7;
				TxMessage.Data[0] = 0x00;
				TxMessage.Data[1] = framepythondata.tail_amp;
				TxMessage.Data[2] = framepythondata.tail_dyn;
				f2b.f = (float)framepythondata.tail_freq / 10;
				TxMessage.Data[3] = f2b.b[0];
				TxMessage.Data[4] = f2b.b[1];
				TxMessage.Data[5] = f2b.b[2];
				TxMessage.Data[6] = f2b.b[3];
				CAN_Transmit(CAN1, &TxMessage);
				
				TxMessage.ExtId = 0X1330;
				TxMessage.IDE=CAN_ID_EXT;
				TxMessage.RTR=CAN_RTR_DATA;
				TxMessage.DLC = 5;
				TxMessage.Data[0] = 0x01;
				TxMessage.Data[1] = framepythondata.tail_vol_l & 0xff;
				TxMessage.Data[2] = framepythondata.tail_vol_l >> 8;
				TxMessage.Data[3] = framepythondata.tail_vol_r & 0xff;
				TxMessage.Data[4] = framepythondata.tail_vol_r >> 8;
				CAN_Transmit(CAN1, &TxMessage);
	for (i = 0; i < rec_char_index; i++)
	{
		rec_char_data[i] = 0x00;
	} 
}
extern FrameROMMotionData rom_motion_data;
extern uint8_t ROM_UPDATE_FLAG;
void Python_Data_Of_Rom_Save(void)
{
	uint8_t index, i;
	uint16_t _4th, _3th, _2nd, _1st;
	index = 4;
	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[4]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[5]);
	_2nd = (rec_char_data[index + 2] - '0') *   10;		//USART1_SendByte(rec_char_data[6]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[7]);
	rom_motion_data.rom_motor_l_a_offest = (int)((_4th + _3th + _2nd + _1st) - 2000);		//USART1_SendByte('X');		USART1_SendByte(' ');

	index += 4;	// index = 8
	_4th = (rec_char_data[index + 0] - '0') * 1000;		//USART1_SendByte(rec_char_data[8]);
	_3th = (rec_char_data[index + 1] - '0') *  100;		//USART1_SendByte(rec_char_data[9]);
	_2nd = (rec_char_data[index + 2] - '0')*   10;		//USART1_SendByte(rec_char_data[10]);
	_1st = (rec_char_data[index + 3] - '0');					//USART1_SendByte(rec_char_data[11]);
	rom_motion_data.rom_motor_l_b_offest = (int)(_4th + _3th + _2nd + _1st) - 2000;		//USART1_SendByte('Y');		USART1_SendByte(' ');

	index += 4;		// index = 12
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_motor_r_a_offest = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 16
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_motor_r_b_offest = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_tail_freq = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_tail_amp = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_tail_dyn = (int)(_4th + _3th + _2nd + _1st) - 2000;	
	
	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_st_ratio = (int)(_4th + _3th + _2nd + _1st) - 2000;	

	index += 4;		// index = 20
	_4th = (rec_char_data[index + 0] - '0') * 1000;		
	_3th = (rec_char_data[index + 1] - '0') *  100;		
	_2nd = (rec_char_data[index + 2] - '0')*   10;		
	_1st = (rec_char_data[index + 3] - '0');					
	rom_motion_data.rom_boom_init_posi = (int)(_4th + _3th + _2nd + _1st) - 2000;
	
	for (i = 0; i < rec_char_index; i++)
	{
		rec_char_data[i] = 0x00;
	}
	/* rom参数更新标志，为1时可以更新 */
	ROM_UPDATE_FLAG = 1;
}

extern float BarycenterRatio;
extern uint8_t StepGoOver, BoomGoOver;
extern uint16_t GoPosition;
extern uint8_t boom_status_flag;
extern uint8_t StepMotorLockState;
extern uint8_t FlagOfStepClose;
extern uint8_t FlagOfBoomClose;
extern float StepSetAngle;
extern float BoomSetDeepth;
void Glider4_Motion_Mode(void)
{
	static uint32_t g_cnt = 0;
	static uint16_t dt = 20;
	
	uint16_t T1, T2, T3, T4;
	
	switch(framepythondata.glider_flag)
	{
		/* 滑翔机调节机构复位模式 */
		case 0:
			g_cnt = 0;
			FlagOfStepClose = 0;
		/* 丝杆机构回中位 */
//		BarycenterRatio = 0.5;
		/* 水泵位置回中位 */
//		GoPosition = 500;
		if (StepGoOver == 1 && BoomGoOver == 1)
		{

		}
			break;
		case 1:
			FlagOfStepClose = 0;
			T1 = (uint16_t)((float)framepythondata.down_tim * 1000 / dt);
			T2 = T1 + (uint16_t)((float)framepythondata.keep_tim * 1000 / dt);
			T3 = T2 + (uint16_t)((float)framepythondata.up_tim * 1000 / dt);
			T4 = T3 + (uint16_t)((float)framepythondata.keep_tim * 1000 / dt);
			if (g_cnt == 1)
			{
				BarycenterRatio = (float)framepythondata.down_ang / 100 + 0.5;
				if (BarycenterRatio >= 0.92)BarycenterRatio = 0.92;
				GoPosition = 500 - framepythondata.water_in;
				if (GoPosition >= 980)GoPosition = 980;
			}
			else if(g_cnt == T1)
			{
				BarycenterRatio = (float)framepythondata.mid_ang / 100;
				GoPosition = framepythondata.water_mid;
			}
			else if(g_cnt == T2)
			{
				BarycenterRatio = 0.5 - (float)framepythondata.down_ang / 100;
				if (BarycenterRatio <= 0.08)BarycenterRatio = 0.08;
				GoPosition = 500 + framepythondata.water_ex;
				if (GoPosition <= 20)GoPosition = 20;
			}
			else if(g_cnt == T3)
			{
				BarycenterRatio = (float)framepythondata.mid_ang / 100;
				GoPosition = framepythondata.water_mid;
			}
			else if(g_cnt == T4)
			{
				g_cnt = 0;
			}
			else
			{}
			g_cnt++;
			if(g_cnt > T4)
			{
				g_cnt = 0;
				framepythondata.glider_flag = 0;
			}
			break;
		case 2:
			g_cnt = 0;
			FlagOfStepClose = 0;
			FlagOfBoomClose = 0;
			BarycenterRatio = (float)framepythondata.mid_ang / 100;
			GoPosition = framepythondata.water_mid;
			break;
		case 3:
			g_cnt = 0;
			FlagOfStepClose = 1;
			StepSetAngle = (float)framepythondata.mid_ang;
			break;
		case 4:
			g_cnt = 0;
			FlagOfBoomClose = 1;
			BoomSetDeepth = (float)framepythondata.water_mid;
			break;
		case 5:
			g_cnt = 0;
			OperateMode = 1;
			break;
		case 6:
			g_cnt = 0;
			OperateMode = 0;
			break;
		default:
			break;
	}
}


extern float DistanceIR;
void GetPoseJY901(void)
{
	static uint16_t j_cnt = 0;

	j_cnt++;
	if (j_cnt == 10)
	{
		j_cnt = 0;
		/* 姿态角度获華ngle 0 1 2 分别为:pitch roll yaw */
		IICreadBytes(0x50, Roll, 6, &chrTemp[0]);
		Angle[0] = (float)CharToShort(&chrTemp[0])/32768*180 - AngInit[0];
		Angle[1] = (float)CharToShort(&chrTemp[2])/32768*180 - AngInit[1];//俯仰角
		Angle[2] = (float)CharToShort(&chrTemp[4])/32768*180 - AngInit[2];//俯仰角
//		printf("The Pitch: %.2f, IRL:%.2f ST:%d \n", Angle[0], DistanceIR, TIM2->CNT);
	}
}

void GetPoseJY901Init(float times)
{
	uint16_t i;
	float temp[3] = {0};
	for (i = 0; i < times; i++)
	{
		IICreadBytes(0x50, Roll, 6, &chrTemp[0]);
		temp[0] += (float)CharToShort(&chrTemp[0])/32768*180;
		temp[1] += (float)CharToShort(&chrTemp[2])/32768*180;//俯仰角
		temp[2] += (float)CharToShort(&chrTemp[4])/32768*180;//俯仰角
	}
	AngInit[0] = temp[0] / times;
	AngInit[1] = temp[0] / times;
	AngInit[2] = temp[0] / times;
//	printf("The Init Pitch: %.2f, Roll: %.2f Yaw: %.2f \n", AngInit[0], AngInit[1], AngInit[2]);
}


/* **************************************ROS serial port data rec************************************** */
#define FRAME_BYTE_FST_ROS 0xca
#define FRAME_BYTE_SEC_ROS 0xac
#define FRAME_BYTE_LST_ROS 0xcc
#define FRAME_HEAD_FLAG_ROS 0x01
#define FRAME_OVER_FLAG_ROS 0x02
#define BUFFER_SIZE_ROS 64

uint8_t rec_data_ROS[BUFFER_SIZE_ROS];
uint8_t rec_index_ROS = 0x00;
uint8_t rec_flag_ROS = 0x00;

uint8_t SumCheck_ROS(void);
uint8_t XORCheck_ROS(void);

uint8_t XORCheck_ROS(void)
{
	uint8_t i = 0;
	uint16_t xor_byte = 0x00;
	xor_byte = rec_data_ROS[0];
	for (i = 1; i < rec_data_ROS[2] - 2; i++)
	{
		xor_byte ^= rec_data_ROS[i];
	}

	if ((xor_byte & 0xff) == rec_data_ROS[rec_data_ROS[2] - 2])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t FrameGet_ROS(uint8_t data)
{
	uint8_t i;

	if (data == FRAME_BYTE_FST_ROS)
	{
		rec_flag_ROS = rec_flag_ROS | FRAME_HEAD_FLAG_ROS;
		rec_data_ROS[rec_index_ROS] = data;
		rec_index_ROS++;
	}
	else if (data == FRAME_BYTE_SEC_ROS)
	{
		if (rec_flag_ROS & FRAME_HEAD_FLAG_ROS)
		{
			rec_flag_ROS = rec_flag_ROS & ~FRAME_OVER_FLAG_ROS;
			rec_index_ROS = 0;
		}
		else
		{
			rec_data_ROS[rec_index_ROS] = data;
			rec_index_ROS++;
			rec_flag_ROS = rec_flag_ROS & ~FRAME_HEAD_FLAG_ROS;
		}
	}
	else
	{
		rec_data_ROS[rec_index_ROS] = data;
		rec_flag_ROS &= ~FRAME_HEAD_FLAG_ROS;
		rec_index_ROS++;

		if (rec_index_ROS == rec_data_ROS[2])
		{

			if (rec_data_ROS[rec_index_ROS-1] == FRAME_BYTE_LST_ROS)
			{
				rec_flag_ROS |= FRAME_OVER_FLAG_ROS;
				/************************/	
				if (XORCheck_ROS())
				{
					return rec_data_ROS[1];
				}
				else
				{
					return 0x01;
				}
				/************************/	
			}
		}
		else if(rec_index_ROS == BUFFER_SIZE_ROS)
		{
			rec_index_ROS--;
			return 0x01;
		}
		else
		{return 0x01;}
	}
}

void ROSDataProcess(uint8_t data)
{
	uint8_t id_temp = 0;
	union float2byte
	{
		float f;
		uint8_t b[4];
	}f2b;
	id_temp = FrameGet_ROS(data);
	if (id_temp == 0x37)
	{
		/* 数据ID:  */
		if (rec_data_ROS[3] == 0x00)
		{
		}

	}
	else if (id_temp == 0x01)
	{
		
	}
	else
	{}
}
/***************************************************传感器数据串口发送至上位机***************************************/
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}

signed short OutData[4]; 
void OutPut_Data(void)
{
    int temp[4] = {0};
    unsigned int temp1[4] = {0};
    unsigned char databuf[10] = {0};
    unsigned char i;
    unsigned short CRC16 = 0;
    for(i=0;i<4;i++)
    {
        temp[i]  = (int)OutData[i];
        temp1[i] = (unsigned int)temp[i];
    }

    for(i=0;i<4;i++)
    {
        databuf[i*2]   = (unsigned char)(temp1[i]%256);
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);
    }

    CRC16 = CRC_CHECK(databuf,8);
    databuf[8] = CRC16%256;
    databuf[9] = CRC16/256;

    for(i=0;i<10;i++)        
    {
//        UART5_SendByte((char)databuf[i]);
			USART_SendData(UART5, (uint8_t)databuf[i]);
    }
}

/********************************************************************************************************************/

