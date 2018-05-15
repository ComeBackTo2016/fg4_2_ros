#include "SerialDMA.h"
#include "string.h"
#include "Fifo4serial.h" 
#include "stm32f10x_can.h"
#include "WaterSensor.h"
#include "stdio.h"

#define FRAME_BYTE_FST 0xca
#define FRAME_BYTE_SEC 0xac
#define FRAME_BYTE_LST 0xcc
#define FRAME_HEAD_FLAG 0x01
#define FRAME_OVER_FLAG 0x02
#define BUFFER_SIZE 128

extern Fifo4Serial QueueOfUart1Rec;
extern Fifo4Serial QueueOfUart5Rec;

unsigned char rec_data[BUFFER_SIZE];
unsigned char rec_index = 0x00;
unsigned char rec_flag = 0x00;

unsigned char SumCheck(void);
unsigned char XORCheck(void);

typedef struct {
	uint8_t ec_start_flag;
	uint8_t  ec_stop_flag;
	float ec_tempter;
	float ec_ecdata;
	float ec_calbr_k;
	float ec_calbr_b;
	float imu_pitch;
	float imu_roll;
	float imu_yaw;
}RS485FrameData;
RS485FrameData RS485_RECEIVED_DATA;
extern uint8_t RS485Rec[RS485BUFFERLEN];

typedef struct {
	uint16_t motor_l;
	uint16_t motor_r;
	float dj_vol;
	int8_t dj_dseta;
	int8_t dj_amp;
} FramePushData;
FramePushData framepushrec;

void FramePushDataInit(void)
{
	framepushrec.dj_amp = 0;
	framepushrec.dj_dseta = 0;
	framepushrec.dj_vol = 0;
	framepushrec.motor_l = 1500;
	framepushrec.motor_r = 1500;
}
/**************************************************************************
函数功能:控制参数解析函数（中断中执行）
入口参数:data 串口接收到的每一帧
返 回 值:无
数据格式:0xca 0xac num reg ... data ... crc 0x
作    者:何宇帆(2016-09-20)
**************************************************************************/
/* 通信协议V1.2，见onenote */
uint8_t FrameGet(uint8_t data)
{
	unsigned char i;

	if (data == FRAME_BYTE_FST)
	{
		rec_flag = rec_flag | FRAME_HEAD_FLAG;
		rec_data[rec_index] = data;
		rec_index++;
	}
	else if (data == FRAME_BYTE_SEC)
	{
		if (rec_flag & FRAME_HEAD_FLAG)
		{
			rec_flag = rec_flag & ~FRAME_OVER_FLAG;
			rec_index = 0;
		}
		else
		{
			rec_data[rec_index] = data;
			rec_index++;
			rec_flag = rec_flag & ~FRAME_HEAD_FLAG;
		}
	}
	else
	{
		rec_data[rec_index] = data;
		rec_flag &= ~FRAME_HEAD_FLAG;
		rec_index++;
		/* 当索引数值rec_index等于帧长度rec_data[1] */
		if (rec_index == rec_data[2])
		{

			if (rec_data[rec_index-1] == FRAME_BYTE_LST)
			{
				rec_flag |= FRAME_OVER_FLAG;
				/************************/	
				if (XORCheck())
				{
					/* 此处待定 */
					return rec_data[1];
				}
				else
				{
					return 0x01;
				}
				/************************/	
			}
		}
		else if(rec_index == BUFFER_SIZE)
		{
			rec_index--;
			return 0x01;
		}
		else
		{return 0x01;}
	}
}

/**************************************************************************
函数功能:接收数据帧和校验
入口参数: 串口接收到的每一帧数据
返 回 值:和校验标志位 0 失败    1 成功
数据格式:num reg ... data ... crc 0x
作    者:何宇帆(2016-09-20)
**************************************************************************/
unsigned char SumCheck(void)
{
	unsigned char i = 0;
	unsigned int sum_byte = 0x00;
	
	for (i = 0; i < rec_data[2] - 2; i++)
	{
		sum_byte += rec_data[i];
	}

	if ((sum_byte & 0xff) == rec_data[rec_data[2] - 2])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/**************************************************************************
函数功能:接收数据帧异或校验
入口参数: 串口接收到的每一帧数据
返 回 值:异或校验标志位 0 失败    1 成功
数据格式:num reg ... data ... crc 0x
作    者:何宇帆(2016-09-20)
**************************************************************************/
unsigned char XORCheck(void)
{
	unsigned char i = 0;
	unsigned int xor_byte = 0x00;
	xor_byte = rec_data[0];
	for (i = 1; i < rec_data[2] - 2; i++)
	{
		xor_byte ^= rec_data[i];
	}

	if ((xor_byte & 0xff) == rec_data[rec_data[2] - 2])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

extern float temper_ms,depth_ms,pressure_ms;
extern float Angle[3];
extern float LeakageData[4];
extern float BatIV[2];
extern float IRData[3];
float RaspiDataSend[13] = {0};
uint8_t uart1sendflag = 1;
uint8_t uart2sendflag = 1;
uint8_t uart4sendflag = 1;
extern float DistanceIR;
extern uint8_t OperateMode;
void Raspi_Data_Process(void)
{
	union float2byte
	{
		float f;
		uint8_t b[4];
	}f2b;
	int8_t temp, i, j;
	uint8_t sendbuf[60] = {0};
	CanTxMsg TxMessage;
	int8_t xor_temp = 0;
	uint16_t tim2_cnt_temp = 0;
	if (QueueOut(&QueueOfUart1Rec, &temp) == QUEUE_OK)
	{
		switch (FrameGet(temp))
		{
			/* Error flag */
			case 0x34:
				while(uart1sendflag);
				uart1sendflag = 1;

				RaspiDataSend[RDS_DEPTH] = depth_ms;
				RaspiDataSend[RDS_TEMP]  = temper_ms;
				RaspiDataSend[RDS_EC]    = pressure_ms;
				RaspiDataSend[RDS_L1]    = RS485_RECEIVED_DATA.ec_ecdata;
				RaspiDataSend[RDS_L2]    = Angle[0];
				RaspiDataSend[RDS_L3]    = IRData[0];
				RaspiDataSend[RDS_L4]    = IRData[1];
				RaspiDataSend[RDS_BATV]  = BatIV[0];
				RaspiDataSend[RDS_BATI]  = BatIV[1];
				RaspiDataSend[RDS_BOOM]  = DistanceIR;
				tim2_cnt_temp = TIM2->CNT;
				RaspiDataSend[RDS_POSI]  = (float)tim2_cnt_temp;
				RaspiDataSend[RDS_IRL]   = IRData[0];
				RaspiDataSend[RDS_IRR]   = IRData[1];
				
				sendbuf[0] = 0xca;
				sendbuf[1] = 0xac;
				sendbuf[2] = 0x13;
				sendbuf[3] = 0x34;
				sendbuf[4] = 6 + 4 * 13;
				sendbuf[5] = 0x00;
				xor_temp = sendbuf[2] ^ sendbuf[3] ^ sendbuf[4] ^ sendbuf[5];
				memcpy(&sendbuf[6], &RaspiDataSend[0], 52);
				for (i = 0; i < 13; i++)
				{
					f2b.f = RaspiDataSend[i];
					for (j = 0; j < 4; j++)
					{
						xor_temp ^=  f2b.b[j];
					}
				}
				sendbuf[58] = xor_temp;
				sendbuf[59] = 0xcc;
				Raspi_DMA_Send(DMA1_Channel4, &sendbuf, 60);
				break;
			case 0x37:
				/* 进入机器操作模式 */
				if (OperateMode == 0)
				{
					/* 推进板canbus数据转发:舵机+推进器 */
					memcpy(&framepushrec, &rec_data[4], sizeof(framepushrec));
					TxMessage.ExtId = 0X1330;
					TxMessage.IDE=CAN_ID_EXT;
					TxMessage.RTR=CAN_RTR_DATA;
					TxMessage.DLC = 7;
					TxMessage.Data[0] = 0x00;
					TxMessage.Data[1] = framepushrec.dj_amp;
					TxMessage.Data[2] = framepushrec.dj_dseta;
					f2b.f = framepushrec.dj_vol;
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
					TxMessage.Data[1] = framepushrec.motor_l & 0xff;
					TxMessage.Data[2] = framepushrec.motor_l >> 8;
					TxMessage.Data[3] = framepushrec.motor_r & 0xff;
					TxMessage.Data[4] = framepushrec.motor_r >> 8;
					CAN_Transmit(CAN1, &TxMessage);
				}
				break;
			/* Function flag */
			case 0x01:
				
				break;
			case 0x35:
				
				break;
		}
	}
}

int8_t ec_cmd_start_conv[] = {0x01, 0x10, 0x1c, 0x00, 0x00, 0x00, 0x00, 0xd8, 0x92};
int8_t ec_cmd_get_temper_ec[] = {0x01, 0x03, 0x26, 0x00, 0x00, 0x05, 0x8e, 0x81};
int8_t ec_cmd_stop_conv[] = {0x01, 0x03, 0x2e, 0x00, 0x00, 0x01, 0x8d, 0x22};
int8_t ec_cmd_get_calbr_para[] = {0x01, 0x03, 0x11, 0x00, 0x00, 0x04, 0x41, 0x35};

int8_t imu_cmd_get_pry[] = {0x77, 0x40, 0x00, 0x40, 0x08};
uint32_t rs485flag = 0;
void RS485_Data_Send(uint32_t flag)
{
	switch(flag)
	{
		case ecstartflag:
			RS485_DMA_Send(DMA2_Channel5, &ec_cmd_start_conv, sizeof(ec_cmd_start_conv)/sizeof(ec_cmd_start_conv[0]));
			rs485flag |= ecstartflag;
			break;
		case ecdataflag:
			RS485_DMA_Send(DMA2_Channel5, &ec_cmd_get_temper_ec, sizeof(ec_cmd_get_temper_ec)/sizeof(ec_cmd_get_temper_ec[0])); //sizeof(ec_cmd_get_temper_ec)/sizeof(uint8_t)
			rs485flag |= ecdataflag;
			break;
		case ecstopflag:
			RS485_DMA_Send(DMA2_Channel5, &ec_cmd_stop_conv, sizeof(ec_cmd_stop_conv)/sizeof(ec_cmd_stop_conv[0]));
			rs485flag |= ecstopflag;
			break;
		case eccalbrflag:
			RS485_DMA_Send(DMA2_Channel5, &ec_cmd_get_calbr_para, sizeof(ec_cmd_get_calbr_para)/sizeof(ec_cmd_get_calbr_para[0]));
			rs485flag |= eccalbrflag;
			break;
		case ecimuflag:
			RS485_DMA_Send(DMA2_Channel5, &imu_cmd_get_pry, sizeof(imu_cmd_get_pry)/sizeof(imu_cmd_get_pry[0]));
			rs485flag |= ecimuflag;
			break;
		default:
			break;
	}
}

uint32_t EC_CNT_MAX = 0;
static uint32_t ec_cnt = 0;

void EC_TEData_Send(void)
{	
	uint16_t dt = 50;
	if (ec_cnt == dt*50)
	{
		RS485_Data_Send(ecstartflag);
	}
	else if(ec_cnt == dt*100)
	{
		RS485_Data_Send(ecdataflag);
	}
	else if (ec_cnt == dt*130)
	{
		RS485_Data_Send(ecdataflag);
	}
	else if (ec_cnt == dt*160)
	{
		ec_cnt = dt*100;
		ec_cnt--;
	}
	ec_cnt++;
}


void IMU_PRYData_Send(void)
{
	static uint32_t imu_cnt = 0;
	
	if (imu_cnt <= 500)
	{
		imu_cnt++;
	}
	else
	{
		RS485_Data_Send(ecimuflag);
	}
}
	
void RS485_Data_Process(uint8_t length)
{
	int8_t SX, XX, YY;
	typedef struct{float f_t; float f_ec;}ecf;
	ecf f;
//	union ec_data
//	{
//		ecf f;
//		uint8_t b[8];
//	}ed;
	if 		 ((rs485flag & ecstartflag) && (length == 8))
	{
		RS485_RECEIVED_DATA.ec_start_flag = 1;
		rs485flag &= ~ecstartflag;
	}
	else if((rs485flag & ecdataflag) && (length == 15))
	{
		if (RS485Rec[11] == 0x00)
		{
			memcpy(&f.f_t, &RS485Rec[3], 4);
			RS485_RECEIVED_DATA.ec_tempter = f.f_t;
			memcpy(&f.f_ec, &RS485Rec[7], 4);
			RS485_RECEIVED_DATA.ec_ecdata  = f.f_ec;
		}
		else
		{}
		rs485flag &= ~ecdataflag;
	}
	else if((rs485flag & ecstopflag) && (length == 7))
	{
		RS485_RECEIVED_DATA.ec_stop_flag = 1;
		rs485flag &= ~ecstopflag;
	}
	else if((rs485flag & eccalbrflag) && (length == 13))
	{
		RS485_RECEIVED_DATA.ec_calbr_k = 1;
		RS485_RECEIVED_DATA.ec_calbr_b = 0;
		rs485flag &= ~eccalbrflag;
	}
	else if((rs485flag & ecimuflag) && (length == 14))
	{
		SX = RS485Rec[4];
		XX = RS485Rec[5];
		YY = RS485Rec[6];
		RS485_RECEIVED_DATA.imu_pitch = (-1*(SX >> 4))*((SX & 0x0f)*100 + (XX >> 4)*10 + (XX & 0x0f) + (float)(YY >> 4)/10.0 + (float)(YY & 0x0f)/100.0);
		SX = RS485Rec[7];
		XX = RS485Rec[8];
		YY = RS485Rec[9];
		RS485_RECEIVED_DATA.imu_roll  = (-1*(SX >> 4))*((SX & 0x0f)*100 + (XX >> 4)*10 + (XX & 0x0f) + (float)(YY >> 4)/10.0 + (float)(YY & 0x0f)/100.0);
		SX = RS485Rec[10];
		XX = RS485Rec[11];
		YY = RS485Rec[12];	
		RS485_RECEIVED_DATA.imu_yaw   = (-1*(SX >> 4))*((SX & 0x0f)*100 + (XX >> 4)*10 + (XX & 0x0f) + (float)(YY >> 4)/10.0 + (float)(YY & 0x0f)/100.0);
		rs485flag &= ~ecimuflag;
	}
	else
	{
		
	}
}

#include "HwInit.h"
#include "GPIOLIKE51.h"
void Boradcast_Glider_Parameter(void)
{
	static uint16_t cnt = 0;
	union float2byte
	{
		float f;
		uint8_t b[4];
	}f2b;
	union int2byte
	{
		uint16_t i;
		uint8_t b[2];
	}i2b;
	int16_t temp16 = 0;
	float tempf = 0;
	uint8_t databuf[64] = {0};
	uint8_t sendbuf[64] = {0};
	uint8_t i = 0, j = 0, temp8 = 0;
	
	cnt++;
	if (cnt >= 600)
	{
		cnt = 0;
		databuf[i] = 0x13;
		i++, databuf[i] = 0x37;
		i++, databuf[i] = 22;
		i++, databuf[i] = 0x01;
		
		f2b.f = 1.12;
		i++, databuf[i] = f2b.b[0];
		i++, databuf[i] = f2b.b[1];
		i++, databuf[i] = f2b.b[2];
		i++, databuf[i] = f2b.b[3];
		f2b.f = 2.16;
		i++, databuf[i] = f2b.b[0];
		i++, databuf[i] = f2b.b[1];
		i++, databuf[i] = f2b.b[2];
		i++, databuf[i] = f2b.b[3];
		i2b.i = (uint16_t)(15 * 100);
		i++, databuf[i] = i2b.b[0];
		i++, databuf[i] = i2b.b[1];
		temp16 = TIM2->CNT;
		i2b.i = (uint16_t)(temp16);
		i++, databuf[i] = i2b.b[0];
		i++, databuf[i] = i2b.b[1];

		i++, databuf[i] = CLK_F;
		i++, databuf[i] = CLK_B;
		i++, databuf[i] = CLK_IR;
		i++, databuf[i] = (1 == ((uint16_t)LeakageData[0] % 4000)) * (1 == ((uint16_t)LeakageData[1] % 4000)) *
		(1 == ((uint16_t)LeakageData[2] % 4000)) * (1 == ((uint16_t)LeakageData[3] % 4000));
	
		temp8 = databuf[0];
		
		for (j = 1; j <= i; j++)
		{
			temp8 ^= databuf[j];
		}
		i++, databuf[i] = temp8;
		databuf[2] = i + 2;
		sendbuf[0] = 0xca;
		sendbuf[1] = 0xac;
		memcpy(&sendbuf[2], &databuf[0], i + 2);
		sendbuf[i + 3] = 0xcc;// index = i + 1 + 2
		ROS_DMA_Send(DMA1_Channel7, &sendbuf, i + 4);
	}
}
