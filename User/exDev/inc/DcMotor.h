#ifndef __DCMOTOR_H
#define	__DCMOTOR_H

#include "stm32f10x.h"

#define PWM_DCM_11 TIM8->CCR1
#define PWM_DCM_12 TIM8->CCR2


void BuoyancyControl(void);
void SetBoomMotor(int16_t data);

#endif 