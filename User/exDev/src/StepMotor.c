#include "HwInit.h"
#include "GPIOLIKE51.h"
#include "StepMotor.h"
#include "math.h"
#include "stdlib.h"
#include "uart.h"
#include "bsp_SysTick.h"

uint8_t status_flag = 0;
uint8_t StepMotorLockState = 0x00;
uint16_t SetPluse = 0;

void TIM4PluseCountSwitch(FunctionalState NewState);
void GotoSetPositionClosedLoopControl(int16_t setPosition, int16_t realPosition, int16_t fullPosition);
void BarycenterClosedLoopControl(int16_t setAngle, int16_t realAngle);

void StepMotorInit(void)
{
	LV_MS3_1 = 0;
	LV_MS2_1 = 0;
	LV_MS1_1 = 1;
	LV_EN_1  = 1;
	MO_EN_1 = 0;
	LV_DIR_1 = 1;
}

void SetStepMotor(int16_t num_st)
{
		if ((status_flag & stepposi_working) || (status_flag & stepnegi_working))
		{
			
		}
		else 
		{
//			while(status_flag & stepposi_working);
//			while(status_flag & stepnegi_working);
			/* 开启定时器4用作脉冲计数 */
			TIM4PluseCountSwitch(ENABLE);
			SetPluseStarter(0);
			if (num_st>0)
			{
					LV_DIR_1 = 0;
					SetPluseContainer((unsigned int)num_st);
					status_flag |= stepposi_working;
			}
			else if(num_st < 0)
			{
					LV_DIR_1 = 1;
					num_st = 0 - num_st;
					SetPluseContainer((unsigned int)num_st);
					status_flag |= stepnegi_working;
			}
			else
			{
					status_flag &= ~stepnegi_working;
			}
			SetPluseDev(0);
			if (num_st != 0)
			{
				 SetPWMState(500);
			}
			else
			{
				 SetPWMState(0);
			}
		}
}

void ChickStepMotorStatus(void)
{
		SetPWMState(0);
		
		if (status_flag & stepposi_working)
		{
			status_flag &= (~stepposi_working);
		}
		else if (status_flag & stepnegi_working)
		{
			status_flag &= (~stepnegi_working);
		}
    else
		{
			status_flag |= glider_error;
		}			
    TIM4PluseCountSwitch(DISABLE);
}

void TIM4PluseCountSwitch(FunctionalState NewState)
{
    TIM_ITConfig(TIM4, TIM_IT_Update, NewState);
    TIM_Cmd(TIM4, NewState);
}
//#define FSTATELOCKER		0x80//10000000b
//#define BSTATELOCKER		0x40//01000000b
//#define INITSTATELOCKER 0x20//00100000b
//#define STEPFORWARD 0
//#define STEPBACK    1 
//void BarycenterControl(void)
//{
//	int8_t p1, p2;
//	float fullstep = 9000.0f;
//	uint16_t centerstep = 0;
//	/* CLKB未触发,CKLF未触发,状态锁未置位 */
//	if ((CLK_B) == 1 && (CLK_F) == 1 && (StepMotorLockState == 0x00))
//	{
//		/* 电机全速向后运动 */
//		LV_DIR_1 = STEPBACK;
//		SetPWMState(500);
//		UART5_SendByte(0X01);
//	}
//		/* CLKB刚触发,CKLF未触发,状态锁未置位 */
//	else if ((CLK_B == 0) && (CLK_F == 1) && (StepMotorLockState == 0x00))
//	{
//		/* 置位后端状态锁 */
//		StepMotorLockState |= BSTATELOCKER; //01000000
//		GetPluseCount = 0;
//		/* 使能TIM4内部计数器 */
//		TIM4PluseCountSwitch(ENABLE);
//		/* 设置电机运动方向向前 */
//		LV_DIR_1 = STEPFORWARD;
//		fullstep = 0;
//		SetPWMState(500);
//		UART5_SendByte(0X02);
//	}
//	/* CLKB未触发，CLKF刚触发，状态锁未置位 */
//	else if ((CLK_B == 1) && (CLK_F == 0) && (StepMotorLockState == 0x00))
//	{
//		/* 置位前置状态锁 */
//		StepMotorLockState |= FSTATELOCKER; //10000000
//		GetPluseCount = 0;
//		TIM4PluseCountSwitch(ENABLE);
//		LV_DIR_1 = STEPBACK;
//		SetPWMState(500);
//		UART5_SendByte(0X03);
//	}
//	/* CLKB被触发,且前置状态锁已经置位 */
//	else if ((CLK_B == 0) && (CLK_F == 1) && (StepMotorLockState & FSTATELOCKER) && (!(StepMotorLockState & BSTATELOCKER)))
//	{
//		/* 置位后置状态锁,此时前后状态所军被触发 */
//		StepMotorLockState |= BSTATELOCKER;
//		fullstep = GetPluseCount;
//		GetPluseCount = 0;
//		TIM4PluseCountSwitch(DISABLE);
//		LV_DIR_1 = STEPFORWARD;
//		SetStepMotor((int16_t)(fullstep / 2.0f));
//		fullstep = 0;
//			UART5_SendByte(0X04);
//	}
//	else if ((CLK_B == 1) && (CLK_F == 0) && (StepMotorLockState & BSTATELOCKER) && (!(StepMotorLockState & FSTATELOCKER)))
//	{
//		/* 置位前置状态锁,此时前后状态所军被触发 */
//		StepMotorLockState |= FSTATELOCKER;
//		TIM4PluseCountSwitch(DISABLE);
//		fullstep = GetPluseCount;
//		GetPluseCount = 0;
//		LV_DIR_1 = STEPBACK;
//		SetStepMotor((int16_t)(fullstep / 2.0));
//		fullstep = 0;
//		UART5_SendByte(0X05);
//	}
//	/* CLKFB均未触发,且前后状态锁均已经置位,说明此时丝杆滑块已经完成端点触发 */
//	else if ((CLK_B == 1) && (CLK_F == 1) && (StepMotorLockState & FSTATELOCKER) && (StepMotorLockState & BSTATELOCKER))
//	{
//		if (SetPluse != 0)
//		{
//			SetStepMotor(SetPluse);
//			SetPluse = 0;
//			UART5_SendByte(0X06);
//		}
//	}
//	else{
//		UART5_SendByte(0X07);
//		UART5_SendByte(StepMotorLockState);
//	}

//}
#define FSTATELOCKER		0x80//10000000b
#define BSTATELOCKER		0x40//01000000b
#define INITSTATELOCKER 0x20//00100000b
#define OVERLOCKER      0X10//00010000B
#define STEPFORWARD 1
#define STEPBACK    0

uint8_t StepGoOver = 0;
float BarycenterRatio = 0.5;
float setAngl = 0;
float MaxAngle = 0;
uint8_t FlagOfStepClose = 0;

float StepSetAngle = 0;
extern float Angle[3];
#define NomalMode 1
#define OpenLoopMode 0

int16_t fullstep = 0;
int16_t setpoint = 0;

uint8_t data_a = 0, data_b = 0;

void BarycenterControl(void)
{
	int16_t temp = 0;
	static int16_t  b_cnt = 0, f_cnt = 0;
	
	
	if ((CLK_B == 1) && (CLK_F == 1) && (StepMotorLockState == 0x00))
	{
		TIM4PluseCountSwitch(DISABLE);
		LV_DIR_1 = STEPBACK;
		TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
		SetPWMState(500);
	}
	else if((CLK_B == 0) && (CLK_F == 1) && (StepMotorLockState == 0x00))
	{
		StepMotorLockState |= BSTATELOCKER;
		TIM2->CR1 |= TIM_CR1_CEN;
		TIM2->CNT = 5000;
		LV_DIR_1 = STEPFORWARD;
		SetPWMState(500);
	}
	else if((CLK_B == 1) && (CLK_F == 0) && (StepMotorLockState & BSTATELOCKER) && ((StepMotorLockState & FSTATELOCKER) == 0))
	{
		StepMotorLockState |= FSTATELOCKER;
		fullstep = TIM2->CNT;
		fullstep = fullstep - 5000;
		LV_DIR_1 = STEPBACK;
		SetPWMState(500);
	}
	else if((CLK_B == 1) && (CLK_F == 1) && (StepMotorLockState & BSTATELOCKER) && (StepMotorLockState & FSTATELOCKER))
	{
		temp = TIM2->CNT;
		StepMotorLockState |= INITSTATELOCKER;
		
		if (FlagOfStepClose == 0)
		{
			if (BarycenterRatio > 0.92)
			{BarycenterRatio = 0.92;}
			else if (BarycenterRatio < 0.08)
			{BarycenterRatio = 0.08;}
			setpoint = (int16_t )(fullstep * BarycenterRatio);
			if((temp - 5000 - setpoint) > 20)
			{
				LV_DIR_1 = STEPBACK;
				SetPWMState(500);
				StepGoOver = 0;
			}
			else if((temp - 5000 - setpoint) < -20)
			{
				LV_DIR_1 = STEPFORWARD;
				SetPWMState(500);
				StepGoOver = 0;
			}
			else
			{
				SetPWMState(0);
				StepGoOver = 1;
				StepMotorLockState = 0x00;
				StepGoOver = 0;
			}
		}
		else if(FlagOfStepClose == 1)
		{
			SetPWMState(500);
			BarycenterClosedLoopControl(StepSetAngle, Angle[0]);
		}
		else{}
	}
	else
	{}
	if (CLK_B == 0)
	{
		b_cnt++;
		if(b_cnt >= 20)
		{
			LV_DIR_1 = STEPFORWARD;
			Delay_ms(50);
			b_cnt = 0;
		}
	}
	if (CLK_F == 0)
	{
		f_cnt++;
		if(f_cnt >= 20)
		{
			LV_DIR_1 = STEPBACK;
			Delay_ms(50);
			f_cnt = 0;
		}
	}	
	data_a = CLK_F;
	data_b = CLK_B;
}
extern float BatIV[2];
extern float distan;
void BarycenterControlTest(void)
{
	static int16_t b_cnt = 0, f_cnt = 0, t = 0;
//	SetPWMState(500);
	
//	while (CLK_B == 0)
//	{
//		SetPWMState(500);
//		LV_DIR_1 = STEPBACK;
//	}
//	
//	while (CLK_F == 0)
//	{
//		SetPWMState(500);
//		LV_DIR_1 = STEPFORWARD;
//	}
//	if (CLK_B == 0)
//	{
//		b_cnt++;
//		if(b_cnt >= 20)
//		{
////			SetPWMState(0);
//			b_cnt = 0;
//		}
//		LV_DIR_1 = STEPFORWARD;
//		while(CLK_B == 0)
//		{
//		}
		b_cnt = CLK_B;
		f_cnt = CLK_F;
//	}
//	if (CLK_F == 0)
	{
//		f_cnt++;
//		if(f_cnt >= 20)
//		{
////			SetPWMState(0);
//			f_cnt = 0;
//		}
//		LV_DIR_1 = STEPBACK;
	}
//	printf("当前测试周期: %d, 剩余电压: %d 电流: %d 距离: %.2fmm \n", f_cnt, b_cnt, 89, BatIV[1]);
}

void BarycenterClosedLoopControl(int16_t setAngle, int16_t realAngle)
{
	float errAng = (float)(setAngle - realAngle);
	int8_t errCode = 0;
	static uint8_t b_cnt = 0, f_cnt = 0;

	if (CLK_B == 0)
	{
		b_cnt++;
		if(b_cnt >= 20)
		{
			LV_DIR_1 = STEPFORWARD;
			Delay_ms(50);
			b_cnt = 0;
		}
	}
	else if (CLK_F == 0)
	{
		f_cnt++;
		if(f_cnt >= 20)
		{
			LV_DIR_1 = STEPBACK;
			Delay_ms(50);
			f_cnt = 0;
		}
	}
	else
	{
			if (errAng >= 50) errAng = 50;
			if (errAng < -50) errAng =-50;

			errAng = errAng / 50.0f * 100.0f;
			errCode = (int8_t)(errAng/12.5);
			if (errCode > 0) LV_DIR_1 = STEPBACK;
			if (errCode < 0) LV_DIR_1 = STEPFORWARD;
			errCode = abs(errCode);
			switch (errCode)
			{
				case 1:
					/* 1/8 steper */
					LV_MS3_1 = 0;
					LV_MS2_1 = 1;
					LV_MS1_1 = 1;
					break;
				case 2:
					/* 1/4 steper */
					LV_MS3_1 = 0;
					LV_MS2_1 = 1;
					LV_MS1_1 = 0;
					break;
				case 3:
					LV_MS3_1 = 0;
					LV_MS2_1 = 1;
					LV_MS1_1 = 0;
					break;
				case 4:
					LV_MS3_1 = 0;
					LV_MS2_1 = 1;
					LV_MS1_1 = 0;
					break;
				case 5:
					/* 1/2 steper */
					LV_MS3_1 = 0;
					LV_MS2_1 = 0;
					LV_MS1_1 = 1;
					break;
				case 6:
					LV_MS3_1 = 0;
					LV_MS2_1 = 0;
					LV_MS1_1 = 0;
					break;
				case 7:
					LV_MS3_1 = 0;
					LV_MS2_1 = 0;
					LV_MS1_1 = 0;
					break;
				case 8:
					LV_MS3_1 = 0;
					LV_MS2_1 = 0;
					LV_MS1_1 = 0;
					break;
				default:
					LV_MS3_1 = 1;
					LV_MS2_1 = 0;
					LV_MS1_1 = 1;
					break;
			}
	}
}

void GotoSetPositionClosedLoopControl(int16_t setPosition, int16_t realPosition, int16_t fullPosition)
{
	float errPosition = ((setPosition / 1000) * fullPosition) - realPosition;
	int8_t errCode = 0;

	if (errPosition >= fullPosition) errPosition = fullPosition;
	if (errPosition < -fullPosition) errPosition =-fullPosition;

	errPosition = errPosition / (float)fullPosition * 100.0f;
	errCode = (int8_t)(errPosition / 12.50f);
	if (errCode > 0) LV_DIR_1 = STEPBACK;
	if (errCode < 0) LV_DIR_1 = STEPFORWARD;
	errCode = abs(errCode);
	switch (errCode)
	{
		case 1:
			/* 1/8 steper */
			LV_MS3_1 = 0;
			LV_MS2_1 = 1;
			LV_MS1_1 = 0;
			break;
		case 2:
			/* 1/4 steper */
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 1;
			break;
		case 3:
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 1;
			break;
		case 4:
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 1;
			break;
		case 5:
			/* 1/2 steper */
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 1;
			break;
		case 6:
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 0;
			break;
		case 7:
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 0;
			break;
		case 8:
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 0;
			break;
		default:
			LV_MS3_1 = 0;
			LV_MS2_1 = 0;
			LV_MS1_1 = 0;
			break;
	}
}

#define prd    44
#define Vbreak 44
uint16_t cnt1;
int32_t CNT1;
int32_t V1;
int32_t rcnt1;
void get_encoder_ONE(void)//*******************?????
{
  int32_t CNT1_temp,CNT1_last;
  
  cnt1 = TIM2->CNT;
  CNT1_last = CNT1;
  CNT1_temp = rcnt1 * prd + cnt1;  
  V1 = CNT1_temp - CNT1_last;		
  
  while (V1>Vbreak)				 
  {							      
   rcnt1--;					      
   CNT1_temp = rcnt1 * prd + cnt1;
   V1 = CNT1_temp - CNT1_last;		 
  }							     
  while (V1<-Vbreak)			   
  {							      
   rcnt1++;					      
   CNT1_temp = rcnt1 * prd + cnt1;
   V1 = CNT1_temp - CNT1_last;		 
  }
  CNT1 = CNT1_temp;						 
}
