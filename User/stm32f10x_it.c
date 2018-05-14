/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "init.h"
#include "stdio.h"
#include "cpg.h"
#include "conmunication.h"
#include "imu_data_decode.h"
#include "packet.h"
#include "StepMotor.h"
#include "DcMotor.h"
#include "Control.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern float theta[3];
extern float depth_ms;
extern float Angle[3];
extern float Pectoral_Theta[2];
extern void TimingDelay_Decrement(void);
extern signed short OutData[4]; 
extern unsigned char status_flag;
extern unsigned char StateData;
extern int32_t rcnt1;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	TimingDelay_Decrement();	
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void TIM6_IRQHandler(void)// 中断时间100ms
{	

		if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
		{	
/*****************MY_APP_CODE*******************/

/********************OVER***********************/
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update); 
		}
}

#define Period_CNT 10000

void TIM5_IRQHandler(void)// 中断时间20ms F = 50hz
{	
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)
		{	
/*****************MY_APP_CODE*******************/
			Glider4_Motion_Mode();
/********************OVER***********************/
			TIM_ClearITPendingBit(TIM5, TIM_IT_Update); 
		}
		
}


/**************************************************************************
函数功能:定时器1编码中断函数
入口参数:无
返 回 值:无
作    者:何宇帆(2016-01-25)
**************************************************************************/

void TIM1_IRQHandler()
{
	if(TIM1 -> CR1 & 0X0010)
	{
																				
	}				   
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}
void TIM1_UP_IRQHandler(void) 
{ 	    	  	     
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//TIM
	{

	}	     
} 

/**************************************************************************
函数功能:定时器4编码中断函数
入口参数:无
返 回 值:无
作    者:何宇帆(2016-01-25)
**************************************************************************/
void TIM4_IRQHandler()
{
	u16 tsr;
	tsr=TIM4->SR;	
	if(tsr&0X0001)
	{
		ChickStepMotorStatus();															
	}				   
	TIM4->SR&=~(1<<0);
}
/**************************************************************************
函数功能:定时器8编码中断函数
入口参数:无
返 回 值:无
作    者:何宇帆(2016-01-25)
**************************************************************************/
void TIM8_IRQHandler()
{
	u16 tsr;
	tsr=TIM8->SR;	
	if(tsr&0X0001)
	{
																				
	}				   
	TIM8->SR&=~(1<<0);
}

//void TIM2_IRQHandler()
//{
//	u16 tsr;
//	tsr=TIM2->SR;	
//	if(tsr&0X0001)
//	{

//	}				   
//	TIM2->SR&=~(1<<0);
//}

void TIM2_IRQHandler()
{
  if (TIM2->CR1 & 0X0010)		
   {
    rcnt1 -= 1;
   }
   else rcnt1 += 1;
   TIM_ClearITPendingBit(TIM2, TIM_IT_Update); 
} 
/**************************************************************************
函数功能:串口读取IMU数据包
入口参数:无
返 回 值:无
作    者:何宇帆(2016-01-25)
**************************************************************************/
extern uint8_t RaspiRec[64];
extern Fifo4Serial QueueOfUart1Rec;
extern uint8_t uart1sendflag;

void USART1_IRQHandler(void)
{
		uint16_t temp = 0;
		uint32_t IMASK = 0;
		uint8_t i = 0;

    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)  
    {  
				/* 清除串口1空闲中断 */
        IMASK = USART1->SR;  
        IMASK = USART1->DR;  
		DMA_Cmd(DMA1_Channel5,DISABLE);  
        temp = RASPIBUFFERLEN - DMA_GetCurrDataCounter(DMA1_Channel5);  
				/*----------User Code-------------*/
		for (i = 0; i < temp; i++)
		{
			QueueIn(&QueueOfUart1Rec, RaspiRec[i]);
		}
				/*--------------------------------*/
        DMA_SetCurrDataCounter(DMA1_Channel5, RASPIBUFFERLEN);  
        DMA_Cmd(DMA1_Channel5,ENABLE);  
    }
} 
void DMA1_Channel4_IRQHandler(void)
{    
   if(DMA_GetFlagStatus(DMA1_FLAG_TC4)==SET) 
   {  
			DMA_Cmd(DMA1_Channel4, DISABLE);
			DMA_SetCurrDataCounter(DMA1_Channel4, 0); 
			uart1sendflag = 0;
			DMA_ClearFlag(DMA1_FLAG_TC4); 
    }    
}
/**************************************************************************
函数功能:串口读取数据包
入口参数:无
返 回 值:无
作    者:何宇帆(2016-01-26)
**************************************************************************/
void USART2_IRQHandler(void)
{
	uint8_t ch;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
		ch = USART_ReceiveData(USART2);
		/* C++上位机\raspi通信通用协议代码段 */

	} 
}
extern uint8_t RS485Rec[RS485BUFFERLEN];
extern Fifo4Serial QueueOfUart4Rec;
uint16_t temp = 0;
void UART4_IRQHandler(void)
{
	uint32_t itmask = 0x00;
	uint16_t i = 0;
	
	if (USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)
	{
		itmask = UART4->SR;
		itmask = UART4->DR;
		DMA_Cmd(DMA2_Channel3,DISABLE);
		temp = RS485BUFFERLEN - DMA_GetCurrDataCounter(DMA2_Channel3);
		RS485_Data_Process((uint8_t)temp);
//		for (i = 0; i < temp; i++)
//		{
//			QueueIn(&QueueOfUart4Rec, RS485Rec[i]);
//		}
    DMA_SetCurrDataCounter(DMA2_Channel3, RS485BUFFERLEN);
    DMA_Cmd(DMA2_Channel3,ENABLE); 
	}
}
extern uint8_t uart4sendflag;
void DMA2_Channel4_5_IRQHandler(void)
{    
   if(DMA_GetFlagStatus(DMA2_FLAG_TC5)==SET) 
   {  
			DMA_Cmd(DMA2_Channel5, DISABLE);
			DMA_SetCurrDataCounter(DMA2_Channel5, 0); 
			uart4sendflag = 0;
			DMA_ClearFlag(DMA2_FLAG_TC5); 
    }    
}
extern Fifo4Serial QueueOfUart5Rec;
void UART5_IRQHandler(void)
{
	uint32_t IMASK = 0;
	uint8_t ch = 0;
	if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{ 	
		ch = USART_ReceiveData(UART5);
		Xbox360DataProcess(ch);
		FrameCharGet(ch);
	}
    if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)  
    {  
				/* 清除串口1空闲中断 */
        IMASK = UART5->SR;  
        IMASK = UART5->DR;
				/*----------User Code-------------*/

				/*--------------------------------*/
    }
}
/***************************can总线中断***************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	Host_Read();
}

void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line11) != RESET)
    {
        EXTI_ClearITPendingBit(EXTI_Line11); 
    }
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
