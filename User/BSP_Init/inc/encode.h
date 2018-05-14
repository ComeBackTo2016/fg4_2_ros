#ifndef __ENCODE_H
#define	__ENCODE_H

#include "stm32f10x.h"

void Encoder_TIM_Init(void);
void TIM1_EncoderMode_PE9_PE11_Config(void);	//change
void TIM1_EncoderMode_PA8_PA9_Config(void);		//change
void TIM2_EncoderMode_PA0_PA1_Config(void);		//check
void TIM2_EncoderMode_PA15_PB3_Config(void);
void TIM3_EncoderMode_PA6_PA7_Config(void);		//check
void TIM3_EncoderMode_PC6_PC7_Config(void);
void TIM4_EncoderMode_PB6_PB7_Config(void);
void TIM4_EncoderMode_PD12_PD13_Config(void);	//check
void TIM5_EncoderMode_PA0_PA1_Config(void);
void TIM8_EncoderMode_PC6_PC7_Config(void);		//change

//void get_encoder_one(void);

#endif /* __INIT_H */
