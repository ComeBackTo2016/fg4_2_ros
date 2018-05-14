#ifndef __STEPMOTOR_H
#define	__STEPMOTOR_H

#include "stm32f10x.h"

#define stepposi_working 00000001
#define stepnegi_working 00000010
#define glider_error     10000000

#define SetPluseContainer(n)  	TIM4->ARR = n
#define SetPluseStarter(n)  		TIM4->CNT = n
#define SetPluseDev(n)  				TIM4->PSC = n
#define SetPWMState(n)  	  		TIM1->CCR1 = n
#define GetPluseCount						TIM4->CNT

void StepMotorInit(void);
void ChickStepMotorStatus(void);
void SetStepMotor(int16_t num_st);
void BarycenterControl(void);
void BarycenterClosedLoopControl(int16_t setAngle, int16_t realAngle);
void get_encoder_ONE(void);
void GotoSetPositionClosedLoopControl(int16_t setPosition, int16_t realPosition, int16_t fullPosition);
void BarycenterControlTest(void);
#endif 