#ifndef __CONMUNICATION_H
#define	__CONMUNICATION_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"

#define FrameID_W_DriverWrite 0x1300
#define FrameID_R_DriverMotion 0x1301
#define FrameID_R_DriverCapcity 0x1302
#define FrameID_R_WQSRead 0x1321
#define FrameID_W_GliderWrite	0x1330
#define FrameID_R_GliderRead	0x1331

union float2char
{
	float num;
	unsigned char byte[4];
};

typedef enum
{
    Volt_12 = 12,
    Volt_24 = 24,
}VoltageClass;

typedef enum
{
		DriverFrameWrite = FrameID_W_DriverWrite,
    DriverFrameMotion = FrameID_R_DriverMotion,
    DriverFrameCapacity = FrameID_R_DriverCapcity,
}DriverFrameID;

struct DriverDataStruct
{
	VoltageClass Volt;
	DriverFrameID Dfi;
	int Volocity;
	float Kp;
	float Ki;
	unsigned char Tc;
};

typedef enum
{
		WQSFrameWrite = 0x1320,
    WQSFrameRead  = 0x1321,
}WQSFrameID;

typedef enum
{
		WQSPH = 0X01,
    WQSORP  = 0X02,
		WQSEC = 0X04,
		WQSTUR = 0X08,
		WQSALL = 0X00
}WQSDataCH;

struct WQSDataStruct
{
	WQSFrameID WQSFi;
	WQSDataCH WQSchannel;
	int iPH;
	float fPH;
	int iORP;
	float fORP;
	int iEC;
	float fEC;
	int iTurbity;
	float fTurbity;
	float Tempreture;
	int iSampling;
};

typedef enum
{
		GliderFrameWrite = 0x1330,
    GliderFrameRead  = 0x1331,
}GliderFrameID;
typedef enum
{
		GliderAng = 0x01,
    GliderTim  = 0X02,
		GliderDeep = 0X04,
		ReadDeep = 0xff,
	
	  GliderDeepth = 0x01,
	  GliderVdeep  = 0x02,
	  GliderPitch  = 0x04,
	  GliderRoll   = 0x08,
	  GliderYaw    = 0x10,
	  GliderTemp   = 0x20,
	  GliderBoom   = 0x40,
	  GliderVoltage= 0x80,
	
}GliderDataCH;
struct GliderDataStruct
{
	GliderFrameID GLIDERFi;
	GliderDataCH Gliderchannel;
	int angSink;
	int angFloat;
	int timSink;
	int timFloat;
	int deepInit;
	int deepEnd;
};

struct MCU2Raspi
{
	unsigned char FrameHeadI;
	unsigned char FrameHeadII;
	unsigned char FrameIDI;
	unsigned char FrameIDII;
	unsigned char FrameTail;
	unsigned char DataSum;
	unsigned char DataLength;
};

struct setHostPara
{
	float setDepth;
	float setPitch;
	float setRoll;
	float setDJ;
	short int setPropeller_L;
	short int setPropeller_R;
	unsigned char setSysHour;
	unsigned char setSysMin;
	unsigned char setSysSec;
};

unsigned char DriverReadWriteData(struct DriverDataStruct *DriverSendData);
unsigned char WQSReadWriteData(struct WQSDataStruct *WQSSendData);
unsigned char GliderReadWriteData(struct GliderDataStruct *GliderSendData);
void MatserHandler(void);
void DriverSalveHander(void);
void GliderSalveHander(void);
void WQS2RaspberryPi(void);



/* 传感器采集板数据标签 */
#define Label_PH 0x01
#define Label_ORP 0x02
#define Label_EC 0x03
#define Label_TUR 0x04
#define Label_WATERCHECK 0x05
#define Label_IRL 0x06
#define Label_IRR 0x07
#define Label_SONIC 0x08

void Salve_Sensor_SendGroup1(void);
void Salve_Sensor_SendGroup2(void);
void Host_Read(void);
void Host_write(unsigned short int ID, unsigned char channel);
void Host2RaspberryPi(unsigned short int groupid);
void HostsParameterSet(unsigned short int ID, unsigned char groupid);

#endif