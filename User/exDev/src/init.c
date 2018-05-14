#include "init.h"
#include <stdio.h>
#include <string.h>

extern Fifo4Serial QueueOfUart1Rec;
extern Fifo4Serial QueueOfUart4Rec;
extern Fifo4Serial QueueOfUart2Rec;

static void IIC2_NVIC_Config(unsigned char Pre_EV, unsigned char Pre_ER);

void NVIC_of_All(void)
{
		/* ����NVIC������2,��ռ���ȼ�0-1 ��Ӧ���ȼ�0-7 */
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
		/*�ж����ȼ�������Ҫ���ݳ������������������*/
//	CANBUS 0 0 
		UART5_NVIC_Config(0, 1);
		UART1_NVIC_Config(0, 2);
		UART4_NVIC_Config(0, 3);
		Timer4_NVIC_Config(1, 4);
		DMA1ch7_NVIC_Config(1, 0);
		Timer5_NVIC_Config(1, 1);
		DMA1ch4_NVIC_Config(1, 2);
		DMA2ch5_NVIC_Config(1, 3);
		DMA1ch7_NVIC_Config(0, 0);
}
//void ROS_Uart_Init(void);
//void ROS_DMA_Init(void);
//void DMA1ch7_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority);
//void ROS_DMA_Send(DMA_Channel_TypeDef*DMA_CHx, void* buffer, int16_t buffersize);

void UART_INIT(void)				
{
//		/* 2.4G����ģ�� ������ 9600 */
		UART5_PC12_PD02_Config();
//		/* 485���� ������ 115200 */
		UART4_PC10_PC11_Config();
//		/* Raspiͨ�� ������ 115200 */
		UART1_PA09_PA10_Config();
		Raspi_DMA_Init();
		Raspi_Uart_Init();
		RS485_Uart_Init();
		RS485_DMA_Init();
		ROS_Uart_Init();
		ROS_DMA_Init();
		
		QueueInit(&QueueOfUart1Rec);
		QueueInit(&QueueOfUart4Rec);
		QueueInit(&QueueOfUart2Rec);
}

void PWM_INIT(void)
{
	  /* ˮ��ֱ����������ź� ch = 1,2 F = 8K PWM��0~1000 */
    TIM8_PWM_PC6_PC7_PC8_PC9_Config(1000, 9);
	/* ������������ź� ch = 1 F = 1K PWM: 500(50%) */
    TIM1_PWM_PA8_PA9_PA10_PA11_Config(1000, 72);
}

void ENCODE_INIT(void)		
{
		/* ��������ӿ� ch = 1,2 */
		TIM2_EncoderMode_PA0_PA1_Config();
		/* ����������巴������ ch = 1 */
		TIM4_EncoderMode_PB6_PB7_Config();

}

void TIMER_INIT(void)				
{
		/*��ʱʱ�䣺72M/72*20000us=20ms*/
		Timer5_Config(12500, 72);		//20
}

/* Time:2017-01-06 */
void IIC1_Config(void)
{
		I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
		RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C1, DISABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

		/*����I2C2�豸������Ϊ���ÿ�©���*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;    																							 //Ϊ������������п����𻵶˿ڣ�����
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		I2C_DeInit(I2C1);
		// I2C2 configuration ---------------------------------------------
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1 = 0x70;      
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStructure.I2C_ClockSpeed = 200000;//400000;

		I2C_Cmd(I2C1, ENABLE );

		I2C_Init(I2C1, &I2C_InitStructure);
		I2C_GeneralCallCmd(I2C1,ENABLE);

		I2C_ITConfig( I2C1, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE );
}

void IIC2_Config(void)
{
		I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
		// I2C���踴λ.
		RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB1PeriphResetCmd( RCC_APB1Periph_I2C2, DISABLE);
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

		/*����I2C2�豸������Ϊ���ÿ�©���*/
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;    																							 //Ϊ������������п����𻵶˿ڣ�����
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure); 

		// I2C����.
		I2C_DeInit( I2C2 );
		// I2C2 configuration ---------------------------------------------
		I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
		I2C_InitStructure.I2C_OwnAddress1 = 0x90;      
		I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
		I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	//	I2C_InitStructure.I2C_ClockSpeed = 200000;//400000;

		I2C_Cmd(I2C2, ENABLE );

		I2C_Init(I2C2, &I2C_InitStructure);
		I2C_GeneralCallCmd(I2C2,ENABLE);

		I2C_ITConfig( I2C2, I2C_IT_EVT | I2C_IT_BUF | I2C_IT_ERR, ENABLE );
}

/* TIME:2017-01-06 */
void IIC1_NVIC_Config(unsigned char Pre_EV, unsigned char Pre_ER)
{
		NVIC_InitTypeDef NVIC_InitStructure;
			
		NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Pre_EV;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Pre_ER;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}

//some question
void IIC2_NVIC_Config(unsigned char Pre_EV, unsigned char Pre_ER)
{
		NVIC_InitTypeDef NVIC_InitStructure;
			
		NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Pre_EV;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = I2C2_ER_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Pre_ER;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
}

/*IIC1��ʼ������*/
void IIC1_INIT(void)
{
		IIC1_Config();
    IIC1_NVIC_Config(0, 0);
}

/*IIC��ʼ������*/
void IIC2_INIT(void)
{
		IIC2_Config();
    IIC2_NVIC_Config(0, 0);
}

/* LED��ʼ������ */
void LED_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);					
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;       
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;                                        																								 //Ϊ������������п����𻵶˿ڣ�����
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
    /* ���İ����ʹ�� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Exit_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure; 
		EXTI_InitTypeDef EXTI_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		/* config the extiline() clock and AFIO clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
												  
    /* ����P[A|B|C|D|E]13Ϊ�ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* EXTI line gpio config() */	
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_15 | GPIO_Pin_14;       
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	 // ��������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* EXTI line() mode config */
    GPIO_EXTILineConfig(RCC_APB2Periph_GPIOB, GPIO_PinSource15); 
    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure); 
}

void Common_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure; 
    /* IO��ʱ�ӳ�ʼ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);    
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO , ENABLE);
    /* ����JTAG�ӿ�������ͨIO���� */        
    

    /* �������-1 IO��ʼ�� */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
		GPIO_Init(GPIOA, &GPIO_InitStructure); 
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
		GPIO_Init(GPIOC, &GPIO_InitStructure); 
		
		/* ���������س�ʼ�� */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
		GPIO_Init(GPIOB, &GPIO_InitStructure); 
		
		/* ���ʹ�ܶ˳�ʼ�� */
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
		/* ��ײ����IO��ʼ�� */
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_5;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
}
