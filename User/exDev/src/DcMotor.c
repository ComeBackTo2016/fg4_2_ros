#include "HwInit.h"
#include "GPIOLIKE51.h"
#include "DcMotor.h"
#include "ads1115.h"
#include "WaterSensor.h"
#include "HwInit.h"

uint8_t boom_status_flag = 0;
uint16_t SetPWM = 0, GoPosition = 500;
float DistanceIR = 0;
uint8_t FlagOfBoomClose = 0;
float BoomSetDeepth = 0;

void WaterBoomPositionControl(int16_t setP, int16_t realP);
void WaterBoomCloseLoopControl(int16_t setMeter, int16_t realMeter);
void SetBoomMotor(int16_t data)
{
	if (data >= 0)
	{
		PWM_DCM_11 = data;
		PWM_DCM_12 = 0;
	}
	else
	{
		PWM_DCM_11 = 0;
		PWM_DCM_12 = -data;
	}
}
/* 
 *
 *SharpSensorLengthGet(GetIRSensorData(IRPomp););
 */
int BOOMVOLTAGE = 600;
uint8_t BoomGoOver = 0;
extern float IRData[3];
extern float depth_ms;
void BuoyancyControl(void)
{
	if (CLK_IR == 1 && boom_status_flag == 0x00)
	{
		SetBoomMotor(-BOOMVOLTAGE);
	}
	else if (CLK_IR == 0 && boom_status_flag == 0x00)
	{
		SetBoomMotor(BOOMVOLTAGE);
		boom_status_flag = 0x80;
	}
	else if (CLK_IR == 1 && boom_status_flag == 0x80)
	{
		if (BoomSetDeepth == 1)
		{
//			WaterBoomCloseLoopControl(BoomSetDeepth, depth_ms - 10.0);
		}
		else
		{
			WaterBoomPositionControl(GoPosition, (int16_t)(DistanceIR));
		}
		
	}
	else if (CLK_IR == 0 && boom_status_flag == 0x80)
	{
		SetBoomMotor(BOOMVOLTAGE);
	}
	DistanceIR = SharpSensorLengthGet(IRData[2]);
}
#define DISTANCCE_DEAD 30.0
#define DISTANCCE_RANGE 80.0
#define DISTANCCE_RATIO 1000.0
int16_t deadzone = 20;
float rP = 0;
float sP = 0;
static void WaterBoomPositionControl(int16_t setP, int16_t realP)
{
	
//	static int16_t deadzone = 50;
	rP = (((float)realP * 10 - DISTANCCE_DEAD) / DISTANCCE_RANGE * DISTANCCE_RATIO);
	sP  = setP;
	if (sP - rP > deadzone)
	{
		SetBoomMotor(BOOMVOLTAGE);
		BoomGoOver = 0;
	}
	else if (sP - rP < -deadzone)
	{
		SetBoomMotor(-BOOMVOLTAGE);
		BoomGoOver = 0;
	}
	else
	{
		BoomGoOver = 1;
		SetBoomMotor(0);
	}
}
struct BoomPID{
	float Kp;
	float Ki;
	float Kd;
	int16_t errMeter;
	int16_t errMeter_last;
	int16_t derrMeter;
	float errSum;
	uint16_t T;
	int16_t pidOut;
}boompidpara;
void WaterBoomCloseLoopControl(int16_t setMeter, int16_t realMeter)
{
	boompidpara.errMeter = setMeter - realMeter;
	if (boompidpara.errMeter <= 10 && boompidpara.errMeter >= -10)
	{
		boompidpara.errSum += boompidpara.errMeter / boompidpara.T;/* T of pid cal period */
	}
	boompidpara.pidOut = (int16_t)(boompidpara.Kp * boompidpara.errMeter + boompidpara.Ki * boompidpara.errSum + 
	boompidpara.Kd * (boompidpara.errMeter - boompidpara.errMeter_last));
	boompidpara.errMeter_last = boompidpara.errMeter;
	
	SetBoomMotor(boompidpara.pidOut);
}
