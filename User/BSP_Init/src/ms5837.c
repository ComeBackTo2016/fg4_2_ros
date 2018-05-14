#include "ms5837.h"
#include "IOI2C.h"
#include "bsp_SysTick.h"
#include "math.h"
#include "imu_data_decode.h"
#include "IMU.h"
#include "AHRSREG.h"
#include "string.h"
#include "Control.h"
#include "uart.h"

#define MS5837_ADDR_R             0xED //command read and MS5837 address
#define MS5837_ADDR_W             0xEC //command write and MS5837 address
#define MS5837_RESET              0x1E //command reset
#define MS5837_ADC_READ           0x00 //command read ADC 
#define MS5837_PROM_READ          0xA0 //�Դ�Ϊ��ʼ��ַ��������ȡ7��16λ����ֵ
#define MS5837_CONVERT_D1_8192    0x4A //���Ͷ�ȡD1������ֱ���Ϊ8192
#define MS5837_CONVERT_D2_8192    0x5A //���Ͷ�ȡD2������ֱ���Ϊ8192

#define uchar unsigned char
#define uint  unsigned int
	
uint16_t MS_C[8];
uint32_t D1, D2;//D1��D2��ֵ��24λ��С�����Զ�������32λ�ı�����ţ�D1 is pressure and D2 is temperature
int32_t TEMP;
int32_t P;
uint8_t crcRead;
uint8_t crcCalculated;

float fluidDensity=1029;    //�����ܶȣ��ݶ�Ϊ1029�����Ը����������

const float Pa = 100.0f;
const float bar = 0.001f;
const float mbar = 1.0f;

const uint8_t MS5837_30BA = 0;
const uint8_t MS5837_02BA = 1;
//��������
uint8_t crc4(uint16_t *n_prom);
void calculat();
void MS_WriteOneByte(u8,u8);
//���������ܶ�
void setFluidDensity(float density) 
{
	fluidDensity = density;
}

//���͸�λ����
void ResetForMs5837()
{
  MS_WriteOneByte(MS5837_ADDR_W,MS5837_RESET);                   
}

unsigned int MS5837_init()
{
	int i;	
	for (i=0;i<7;i++) 
	{
		MS_WriteOneByte(MS5837_ADDR_W,MS5837_PROM_READ+i*2);
		IIC_Start();
		IIC_Send_Byte(MS5837_ADDR_R);	           //���͵�ַ
		IIC_Wait_Ack();
		MS_C[i]=IIC_Read_Byte(1);						  //��ȡ��ӦPROM�����ݣ�����ŵ�������,������Դ�Ӧ��
		MS_C[i]=(MS_C[i]<<8)|IIC_Read_Byte(0);//������Դ���Ӧ���ź�
		IIC_Stop(); 
	}
	
	crcRead = MS_C[0]>>12;
	crcCalculated=crc4(MS_C);
	
	if ( crcCalculated==crcRead )
	{
		return 1; // Initialization success
	}
	else 
	{ 
		return 0;
	}	
}

extern float temper_ms,depth_ms,pressure_ms;
extern float pressure_ms_last, pressure_v;


unsigned int cnt_max = 140;
float Init_depth_ms = -9.86;
extern signed short OutData[4]; 
void MS5837_read()
{
	static unsigned long int cnt1 = 0;
	
	cnt1++;
	
	if (cnt1 == 1)
	{
		D1 = 0;
		MS_WriteOneByte(MS5837_ADDR_W,MS5837_CONVERT_D1_8192);
	}
	else if (cnt1 == cnt_max)
	{
		MS_WriteOneByte(MS5837_ADDR_W,MS5837_ADC_READ);
		IIC_Start();
		IIC_Send_Byte(MS5837_ADDR_R);	           //���Ͷ���ַ
		IIC_Wait_Ack();	                           
		D1=IIC_Read_Byte(1);						 //��ȡ24���ֽڵ����ݣ��������ηŵ�D1����
		D1=(D1<<8)|IIC_Read_Byte(1);
		D1=(D1<<8)|IIC_Read_Byte(0);
		IIC_Stop();
		D2 = 0;
		MS_WriteOneByte(MS5837_ADDR_W,MS5837_CONVERT_D2_8192);
	}
	else if (cnt1 == cnt_max*2)
	{
		MS_WriteOneByte(MS5837_ADDR_W,MS5837_ADC_READ);
		IIC_Start();
		IIC_Send_Byte(MS5837_ADDR_R);	           //���Ͷ���ַ
		IIC_Wait_Ack();	                           
		D2=IIC_Read_Byte(1);						 //��ȡ24���ֽڵ����ݣ��������ηŵ�D2����
		D2=(D2<<8)|IIC_Read_Byte(1);
		D2=(D2<<8)|IIC_Read_Byte(0);
		IIC_Stop();
		calculat();//����

		temper_ms=temperature();
		depth_ms=depth()*100 - Init_depth_ms;
		pressure_ms=pressure(1.0);
		OutData[0] = (int16_t)temper_ms;
		OutData[1] = (int16_t)depth_ms;
	}
	else if(cnt1 == (cnt_max*2 + 5))
	{
		cnt1 = 0;
	}
}

	int32_t dT = 0;//���¶����м����
	int64_t SENS = 0;
	int64_t OFF = 0;
	int32_t SENSi = 0;
	int32_t OFFi = 0;  
	int32_t Ti = 0;    
	int64_t OFF2 = 0;
	int64_t SENS2 = 0;
void calculat()
{
	// Given C1-C6 and D1, D2, calculated TEMP and P
	// Do conversion first and then second order temp compensation	

	dT = D2-(uint32_t)(MS_C[5])*256l;//256������L������1����ͬ
	SENS = (int64_t)(MS_C[1])*32768l+((int64_t)(MS_C[3])*dT)/256l;
	OFF = (int64_t)(MS_C[2])*65536l+((int64_t)(MS_C[4])*dT)/128l;
	P = (D1*SENS/(2097152l)-OFF)/(8192l);
	
	// Temp conversion�¶�ת��
	TEMP = 2000l+(int64_t)(dT)*MS_C[6]/8388608LL;
//	TEMP = TEMPDATA;
	/*Second order compensation�¶ȵĶ��ײ���*/

	if((TEMP/100)<20)//Low temp
	 {         
			Ti = ((3*(int64_t)(dT)*(int64_t)(dT))/(8589934592LL));
			OFFi = (3*(TEMP-2000)*(TEMP-2000))/2;
			SENSi = (5*(TEMP-2000)*(TEMP-2000))/8;
			if((TEMP/100)<-15)//Very low temp
			{    
				OFFi = OFFi+7*(TEMP+1500l)*(TEMP+1500l);
				SENSi = SENSi+4*(TEMP+1500l)*(TEMP+1500l);
			}
		}
	else if((TEMP/100)>=20)//High temp
		{    
			Ti = 2*(dT*dT)/(137438953472LL);
			OFFi = (1*(TEMP-2000)*(TEMP-2000))/16;
			SENSi = 0;
		}

	OFF2 = OFF-OFFi;           //Calculate pressure and temp second order����
	SENS2 = SENS-SENSi;

	TEMP = (TEMP-Ti);
	P = (((D1*SENS2)/2097152l-OFF2)/8192l);

}

//����ѹ��
float pressure(float conversion) 
{
	return P/10.0f*conversion;
}
//�����¶�
float temperature() 
{
	return TEMP/100.0f;
}
//�������
float depth() 
{
	return (pressure(Pa)-101300)/(fluidDensity*9.80665);
}
float altitude_t = 0;
float altitude()
{
	altitude_t = (1-pow((pressure(1.0)/1013.25),.190284))*145366.45*.3048;
	return altitude_t;
}

uint8_t crc4(uint16_t *n_prom) 
{
	int cnt;
	unsigned int n_rem = 0;
	unsigned char n_bit;

			n_prom[0] =((n_prom[0])&0x0FFF);
			n_prom[7] =0;

			for (cnt=0;cnt<16;cnt++) 
			{
					if (cnt%2==1) 
					{
							n_rem^=(unsigned short)((n_prom[cnt>>1])&0x00FF);
					} 
					else 
					{
							n_rem^=(unsigned short)(n_prom[cnt>>1]>>8);
					}
					for (n_bit=8;n_bit>0;n_bit--) 
					{
							if (n_rem&0x8000) 
							{
									n_rem=(n_rem<<1)^0x3000;
							} 
							else 
							{
									n_rem=(n_rem<<1);
							}
					}
			}
	n_rem =((n_rem >> 12)&0x000F);
	return (n_rem^0x00);
}


void MS_WriteOneByte(u8 WriteAddr,u8 DataToWrite)
{
	IIC_Start();
	IIC_Send_Byte(WriteAddr); 
	IIC_Wait_Ack();
	IIC_Send_Byte(DataToWrite); 
	IIC_Wait_Ack();
	IIC_Stop(); 
}

