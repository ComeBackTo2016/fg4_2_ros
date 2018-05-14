#include "SerialDMA.h"
#include "string.h"

uint8_t RaspiRec[RASPIBUFFERLEN] = {0};
uint8_t RaspiSend[RASPIBUFFERLEN]= {0};

uint8_t ROSRec[RASPIBUFFERLEN] = {0};
uint8_t ROSSend[RASPIBUFFERLEN]= {0};

uint8_t RS485Rec[RS485BUFFERLEN] = {0};
uint8_t RS485Send[RS485BUFFERLEN]= {0};

void Raspi_Uart_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		 
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	
		USART_DeInit(USART1);  

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	

		USART_Init(USART1, &USART_InitStructure); 
		
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
		USART_ITConfig(USART1, USART_IT_TC, 	DISABLE);
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
		USART_Cmd(USART1, ENABLE);  
}

void Raspi_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel5);
	DMA_DeInit(DMA1_Channel4);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (int32_t)(&USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (int32_t)RaspiRec;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RASPIBUFFERLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* 设置STM32接受区域DMA */
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel5, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (int32_t)&(USART1->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (int32_t)RaspiSend;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = RASPIBUFFERLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* 设置STM32发送区域DMA */
	DMA_Init(DMA1_Channel4, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel4, ENABLE);
}

void DMA1ch4_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Raspi_DMA_Send(DMA_Channel_TypeDef*DMA_CHx, void* buffer, int16_t buffersize)
{ 
	while (DMA_GetCurrDataCounter(DMA1_Channel4));
	if(buffer) memcpy(RaspiSend, buffer, (buffersize > RASPIBUFFERLEN ? RASPIBUFFERLEN : buffersize));
	DMA_Cmd(DMA_CHx, DISABLE ); 
 	DMA_SetCurrDataCounter(DMA1_Channel4, buffersize);
 	DMA_Cmd(DMA_CHx, ENABLE);  
}

void ROS_Uart_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
		USART_DeInit(USART2);  

	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 

		USART_InitStructure.USART_BaudRate = 9600;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	

		USART_Init(USART2, &USART_InitStructure); 
		
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
		USART_ITConfig(USART2, USART_IT_TC, 	DISABLE);
		USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
		USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
		USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
		USART_Cmd(USART2, ENABLE);  	
}

/* register for ros is channel 7[4]tx, 6[4]rx */
void ROS_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel6);
	DMA_DeInit(DMA1_Channel7);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (int32_t)(&USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (int32_t)ROSRec;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RASPIBUFFERLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* 设置STM32接受区域DMA */
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (int32_t)&(USART2->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (int32_t)ROSSend;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = RASPIBUFFERLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* 设置STM32发送区域DMA */
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA1_Channel7, ENABLE);	
}

/* init channel 7 for tx */
void DMA1ch7_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void ROS_DMA_Send(DMA_Channel_TypeDef*DMA_CHx, void* buffer, int16_t buffersize)
{ 
	while (DMA_GetCurrDataCounter(DMA1_Channel7));
	if(buffer) memcpy(RaspiSend, buffer, (buffersize > RASPIBUFFERLEN ? RASPIBUFFERLEN : buffersize));
	DMA_Cmd(DMA_CHx, DISABLE ); 
 	DMA_SetCurrDataCounter(DMA1_Channel7, buffersize);
 	DMA_Cmd(DMA_CHx, ENABLE);  
}

void RS485_Uart_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		
		/* config UART4 clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
		
		/* Configure UART4 Tx (PC.10) as alternate function push-pull */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
    
  	/* Configure UART4 Rx (PC.11) as input floating */
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
		USART_InitStructure.USART_BaudRate            = 9600;//115200;
		USART_InitStructure.USART_WordLength          = USART_WordLength_8b;  
		USART_InitStructure.USART_StopBits            = USART_StopBits_1;     
		USART_InitStructure.USART_Parity              = USART_Parity_No ;     
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(UART4, &USART_InitStructure);
		USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);
		USART_ITConfig(UART4, USART_IT_TC, 	 DISABLE);
		USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);
		USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);
		USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
		USART_Cmd(UART4, ENABLE); 	
}
void RS485_DMA_Init(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA2_Channel5);
	DMA_DeInit(DMA2_Channel3);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (int32_t)(&UART4->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (int32_t)RS485Rec;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = RS485BUFFERLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* 设置STM32接受区域DMA */
	DMA_Init(DMA2_Channel3, &DMA_InitStructure);
	DMA_Cmd(DMA2_Channel3, ENABLE);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (int32_t)&(UART4->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (int32_t)RS485Send;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = RS485BUFFERLEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	/* 设置STM32发送区域DMA */
	DMA_Init(DMA2_Channel5, &DMA_InitStructure);
	DMA_ITConfig(DMA2_Channel5, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Channel5, ENABLE);	
}

void DMA2ch5_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Channel4_5_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void RS485_DMA_Send(DMA_Channel_TypeDef*DMA_CHx, void* buffer, int16_t buffersize)
{ 
	while (DMA_GetCurrDataCounter(DMA2_Channel5));
	if(buffer) memcpy(RS485Send, buffer, (buffersize > RS485BUFFERLEN ? RS485BUFFERLEN : buffersize));
	DMA_Cmd(DMA_CHx, DISABLE ); 
 	DMA_SetCurrDataCounter(DMA2_Channel5, buffersize);
 	DMA_Cmd(DMA_CHx, ENABLE);  
}

void RS485_IMU_Send(uint8_t *ch)
{
	while(*ch!=0)
	{		
		while(!USART_GetFlagStatus(UART4, USART_FLAG_TXE));
		USART_SendData(UART4, *ch);
		ch++;
	}
}

//void RS485_EC_Send(uint8_t *ch)
//{
//	while(*ch!=0)
//	{		
//		while(!USART_GetFlagStatus(UART4, USART_FLAG_TXE));
//		USART_SendData(UART4, *ch);
//		ch++;
//	}
//}

