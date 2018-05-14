#ifndef __SERIALDMA_H
#define	__SERIALDMA_H

#include "stm32f10x.h"



#define RASPIBUFFERLEN	128
#define RS485BUFFERLEN	128

#define rs485set 				0xffffffff
#define rs485clr 				0x00000000
#define ecstartflag 		0x00000001
#define ecdataflag 	  	0x00000002
#define ecstopflag 			0x00000003
#define eccalbrflag			0x00000004
#define ecimuflag 			0x00000010

void Raspi_Uart_Init(void);
void Raspi_DMA_Init(void);
void Raspi_DMA_Send(DMA_Channel_TypeDef*DMA_CHx, void* buffer, int16_t buffersize);
void DMA1ch4_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
void RS485_Uart_Init(void);
void RS485_DMA_Init(void);
void DMA2ch5_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
void RS485_DMA_Send(DMA_Channel_TypeDef*DMA_CHx, void* buffer, int16_t buffersize);
void Raspi_Data_Process(void);
void RS485_IMU_Send(uint8_t *ch);
void RS485_Data_Send(uint32_t flag);
void RS485_Data_Process(uint8_t length);
void EC_TEData_Send(void);
void IMU_PRYData_Send(void);
#endif 