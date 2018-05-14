#include "WaterSensor.h"
#include <stdio.h>
#include "bsp_adc.h"
#include "ads1115.h"
#include "SerialDMA.h"
#include <math.h>
#include "MS5837.h"
#include "bsp_SysTick.h"

extern __IO uint16_t fliterValue[];
float LeakageData[4] = {0};
float BatIV[2] = {0};
float IRData[3] = {0};

void LeakageSensorInit(void)
{
	ADC1_Init();
}

float GetLeakageSensorStaus(LeakageSensorChannelTypedef ch)
{
	int16_t data = 0;
	data = fliterValue[ch];
	return (float)data;
}

float GetIRSensorData(ADS1115Typedef ch)
{
	double cal_temp = 6.1440f / 32768.0f;
	
	return (float)(getad(0xc3, ch) * cal_temp);
}
uint16_t dt = 12;
void ADS1115GetData(void)
{
	double cal_temp = 6.1440f / 32768.0f;
	static uint16_t cnt = 0;
	
	cnt++;
	if(cnt == 1)
	{
		ADS1115Config_A(0xc3,BatMeter);
	}
	else if (cnt == (dt * 2))
	{
		BatIV[0] = (float)(ReadAD_A() * cal_temp) * 6.7f;
	}
	else if (cnt == (dt * 3))
	{
		ADS1115Config_A(0xc3,IRPomp);
	}
	else if (cnt == (dt * 4))
	{
		IRData[2] = (float)(ReadAD_A() * cal_temp);
	}
	else if (cnt == (dt * 5))
	{
		ADS1115Config_A(0xc3,IRL);
	}
	else if (cnt == (dt * 6))
	{
		IRData[0] = (float)(ReadAD_A() * cal_temp);
	}
	else if (cnt == (dt * 7))
	{
		ADS1115Config_A(0xc3,IRR);
	}
	else if (cnt == (dt * 8))
	{
		IRData[1] = (float)(ReadAD_A() * cal_temp);
	}
	else if (cnt >= (dt * 9))
	{cnt = 0;}
}

void SwitchOfWaterQualitySensor(SwitchChannelTypedef ch, SwitchStateTypedef state)
{
	BIT_ADDR(GPIOB_ODR_Addr, ch) = state;
	if (SWITCH_ALL == ch)
	{
		BIT_ADDR(GPIOB_ODR_Addr, 14) = state;
		BIT_ADDR(GPIOB_ODR_Addr, 13) = state;
		BIT_ADDR(GPIOB_ODR_Addr, 12) = state;
	}
}
extern uint8_t uart4sendflag;
uint8_t HEC295_READ_PRY[5] = {0X77, 0X04, 0X00, 0X04, 0X08};
void WaterQualityCheck(WaterQualityTypedef sensor)
{
	switch(sensor)
	{
		case SENSOR_EC:
			while(uart4sendflag);
			uart4sendflag = 1;
			RS485_DMA_Send(DMA2_Channel5, &HEC295_READ_PRY, 5);
			break;
		case SENSOR_OD:
			break;
		case SENSOR_PH:
			break;
		default:
			break;
	}
}

float SharpSensorLengthGet(float v)
{
	double Distance = 0;
	if (v < 1.99 && v > 0.40)
	{//y = 3.0173x4 - 18.111x3 + 41.611x2 - 46.122x + 24.347
		Distance = 3.0173 * pow(v, 4) - 18.111 * pow(v, 3) + 41.611 * pow(v, 2) - 46.122 * v + 24.347;
	}
	else
	{
		Distance = 0;
	}
	return (float)Distance;
}

int16_t MS5837_STATE = 0;
void MS5837DeviceInit(void)
{
	int16_t ms_flag = 0, ms_cnt = 0;

	ResetForMs5837();
	do
	{
		ms_flag = MS5837_init();
		MS5837_STATE = ms_flag;
		ms_cnt++;
		if (ms_cnt == 100)
		{
			ms_cnt = 0;
			MS5837_STATE = 2;
			break;
		}
	}while(ms_flag == 0);
}


//WaterQuality