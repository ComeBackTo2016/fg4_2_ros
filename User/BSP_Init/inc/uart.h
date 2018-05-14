#ifndef __UART_H
#define	__UART_H

#include "stm32f10x.h"

void UART1_PA09_PA10_Config(void);
void UART1_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
void UART2_PA02_PA03_Config(void);
void UART2_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
void UART3_PB10_PB11_Config(void);
void UART3_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
void UART4_PC10_PC11_Config(void);
void UART4_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
void UART5_PC12_PD02_Config(void);
void UART5_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);


void USART1_SendByte(u16 Data);
void USART1_SendString(uint8_t *ch);
void USART2_SendByte(u16 Data);
void USART2_SendString(uint8_t *ch);
void USART3_SendByte(u16 Data);
void USART3_SendString(uint8_t *ch);
void UART4_SendByte(u16 Data);
void UART4_SendString(uint8_t *ch);
void UART5_SendByte(u16 Data);
void UART5_SendString(uint8_t *ch);

#endif /* __INIT_H */