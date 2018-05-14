#ifndef __TIMER_H
#define	__TIMER_H

#include "stm32f10x.h"

#define PWMoutput1 TIM3->CCR1
#define PWMoutput2 TIM3->CCR2
#define PWMoutput3 TIM8->CCR3
#define PWMoutput4 TIM8->CCR4		//PB1 is bad,,  use PA11
#define PWMoutput5 TIM2->CCR1
#define PWMoutput6 TIM2->CCR2
#define PWMoutput7 TIM2->CCR3
#define PWMoutput8 TIM2->CCR4

//#define Tailout1 TIM8->CCR2
//#define Tailout2 TIM8->CCR3
//#define Tailout3 TIM8->CCR4

#define Pectout1 TIM1->CCR1
#define Pectout2 TIM1->CCR3
/* ÎÐÂÖÍÆ½ø×é */
#define PWMturbineL Pectout1
#define PWMturbineR Pectout2

//#define PectoralPWMOutputLeft       PWMoutput5
//#define PectoralPWMOutputRight      PWMoutput6

void Timer6_Config(u16 Period,u16 Prescaler);
void Timer6_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);

void Timer4_Config(u16 Period,u16 Prescaler);
void Timer4_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);

void Timer5_Config(u16 Period,u16 Prescaler);
void Timer5_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);

//void TIM4_PWM_PB6_PB7_PB8_PB9_Config(unsigned int freq);
void TIM4_PWM_PB6_PB7_PB8_PB9_Config(unsigned int freq, unsigned int div);
//void TIM3_PWM_PA6_PA7_PB0_PB1_Config(unsigned int freq);
void TIM3_PWM_PA6_PA7_PB0_PB1_Config(unsigned int freq, unsigned int div);
//void TIM2_PWM_PA0_PA1_PA2_PA3_Config(unsigned int freq);
void TIM2_PWM_PA0_PA1_PA2_PA3_Config(unsigned int freq, unsigned int div);
//void TIM5_PWM_PA0_PA1_PA2_PA3_Config(unsigned int freq);
void TIM5_PWM_PA0_PA1_PA2_PA3_Config(unsigned int freq, unsigned int div);

void TIM8_PWM_PC6_PC7_PC8_PC9_Config(unsigned int freq, unsigned int div);
void TIM1_PWM_PA8_PA9_PA10_PA11_Config(unsigned int freq, unsigned int div);
#endif 
