#ifndef __WATERSENSOR_H
#define	__WATERSENSOR_H

#include "stm32f10x.h"

#define RDS_DEPTH 0
#define RDS_TEMP  1
#define RDS_EC    2

#define RDS_L1    3
#define RDS_L2    4
#define RDS_L3    5
#define RDS_L4    6

#define RDS_BATV  7
#define RDS_BATI  8
#define RDS_BOOM  9
#define RDS_POSI  10
#define RDS_IRL   11
#define RDS_IRR   12
typedef enum
{
	S1 = 0x00,
	S2 = 0x01,
	S3 = 0x02,
	S4 = 0x03,
	BatI = 0x04
}LeakageSensorChannelTypedef;

typedef enum
{
	IRL = 0x40,
	IRR = 0x50,
	BatMeter = 0x60,
	IRPomp = 0x70
}ADS1115Typedef;

typedef enum
{
	SWITCH_EC = 14,
	SWITCH_OD = 13,
	SWITCH_PH = 12,
	SWITCH_ALL= 11,
}SwitchChannelTypedef;

typedef enum
{
	SW_ON = 1,
	SW_OFF = 0
}SwitchStateTypedef;

typedef enum
{
	SENSOR_EC = 0x00,
	SENSOR_OD = 0x01,
	SENSOR_PH = 0x02
}WaterQualityTypedef;

void LeakageSensorInit(void);
float GetLeakageSensorStaus(LeakageSensorChannelTypedef ch);
float GetIRSensorData(ADS1115Typedef ch);
void SwitchOfWaterQualitySensor(SwitchChannelTypedef ch, SwitchStateTypedef state);
void WaterQualityCheck(WaterQualityTypedef sensor);
float SharpSensorLengthGet(float v);
void MS5837DeviceInit(void);
void ADS1115GetData(void);
void FramePushDataInit(void);
void Boradcast_Glider_Parameter(void);
#endif 