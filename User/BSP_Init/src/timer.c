#include "timer.h"

/*
Name:
Function:2017-01-06
*/
void TIM1_PWM_PA8_PA9_PA10_PA11_Config(unsigned int freq, unsigned int div)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
					
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
    /* ���ú������õ���PB0���� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								// �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    /* ������ʱ������ */		 
    TIM_TimeBaseStructure.TIM_Period = freq - 1;       	//����ʱ����0������19999����Ϊ20000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = div -1;	    						//����Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;			//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* PWMģʽ���� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    				    //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	            //ʹ�����
    TIM_OCInitStructure.TIM_Pulse = 0;							                //���ó�ʼPWM������Ϊ0	
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	                //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ

    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
//    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
//    TIM_OC3Init(TIM1, &TIM_OCInitStructure);	 			                    //ʹ��ͨ��3
//    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
//    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
//    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);							//ʹ��Ԥװ��	
//    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);							//ʹ��Ԥװ��

    TIM_ARRPreloadConfig(TIM1, ENABLE);			 					        	//ʹ��TIM3���ؼĴ���ARR

    /* TIM3 enable counter */
    TIM_Cmd(TIM1, ENABLE);                   						        	//ʹ�ܶ�ʱ��3	
    TIM_CtrlPWMOutputs(TIM1, ENABLE); 
}

/*
Name:
Function:2017-01-06
*/
void TIM8_PWM_PC6_PC7_PC8_PC9_Config(unsigned int freq, unsigned int div)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
					
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	
    /* ���ú������õ���PB0���� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								// �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
    /* ������ʱ������ */		 
    TIM_TimeBaseStructure.TIM_Period = freq - 1;       	                        //����ʱ����0������19999����Ϊ20000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = div -1;	    						//����Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;			        //����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	            //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    /* PWMģʽ���� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    				    //����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	            //ʹ�����
    TIM_OCInitStructure.TIM_Pulse = 0;							                //���ó�ʼPWM������Ϊ0	
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	                //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ

    TIM_OC1Init(TIM8, &TIM_OCInitStructure);
    TIM_OC2Init(TIM8, &TIM_OCInitStructure);
//    TIM_OC3Init(TIM8, &TIM_OCInitStructure);	 			                    //ʹ��ͨ��3
//    TIM_OC4Init(TIM8, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
    TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
//    TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);							//ʹ��Ԥװ��	
//    TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);							//ʹ��Ԥװ��

    TIM_ARRPreloadConfig(TIM8, ENABLE);			 					        	//ʹ��TIM8���ؼĴ���ARR

    /* TIM3 enable counter */
    TIM_Cmd(TIM8, ENABLE);                   						        	//ʹ�ܶ�ʱ��3	
    TIM_CtrlPWMOutputs(TIM8, ENABLE); 
}
/***************************/



/*
Name:
Function:
*/
void TIM3_PWM_PA6_PA7_PB0_PB1_Config(unsigned int freq, unsigned int div)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);					
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	
    /* ���ú������õ���PB0���� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								// �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    /* ������ʱ������ */		 
    TIM_TimeBaseStructure.TIM_Period = freq - 1;       	//����ʱ����0������19999����Ϊ20000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = div -1;	    						//����Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;			//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* PWMģʽ���� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    				//����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
    TIM_OCInitStructure.TIM_Pulse = 0;										  			//���ó�ʼPWM������Ϊ0	
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
//    TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 										//ʹ��ͨ��3
//    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
//    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);							//ʹ��Ԥװ��	
//    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);							//ʹ��Ԥװ��

    TIM_ARRPreloadConfig(TIM3, ENABLE);			 											//ʹ��TIM3���ؼĴ���ARR


    /* TIM3 enable counter */
    TIM_Cmd(TIM3, ENABLE);                   											//ʹ�ܶ�ʱ��3	

}
/***************************/


void TIM5_PWM_PA0_PA1_PA2_PA3_Config(unsigned int freq, unsigned int div)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    /* ���ú������õ���PB0���� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    				// �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;				// | GPIO_Pin_2 | GPIO_Pin_3  
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ������ʱ������ */		 
    TIM_TimeBaseStructure.TIM_Period = freq - 1;       	//����ʱ����0������19999����Ϊ20000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = div -1;	    						//����Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;			//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ

    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    /* PWMģʽ���� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    				//����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
    TIM_OCInitStructure.TIM_Pulse = 0;										  			//���ó�ʼPWM������Ϊ0	
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ

    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2Init(TIM5, &TIM_OCInitStructure);											//ʹ��ͨ��2
    //  TIM_OC3Init(TIM5, &TIM_OCInitStructure);	 									//ʹ��ͨ��3
    //	TIM_OC4Init(TIM5, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);							//ʹ��Ԥװ��
    //  TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);						//ʹ��Ԥװ��	
    //  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);						//ʹ��Ԥװ��

    TIM_ARRPreloadConfig(TIM5, ENABLE);			 										//ʹ��TIM3���ؼĴ���ARR

    /* TIM5 enable counter */
    TIM_Cmd(TIM5, ENABLE);                   										//ʹ�ܶ�ʱ��3	
}

/*
 *���ܣ�PWMG ʹ��ͨ��3��4 ��Ӧ����ΪPA2 PA3
 *���룺PWM��Ƶ��
*/
void TIM2_PWM_PA0_PA1_PA2_PA3_Config(unsigned int freq, unsigned int div)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* GPIOA clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* enable the clock of tim2 & afio & remap*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /*�ض�������Ŷ�ӦΪ�PA15-PB3-PA2-PA3*/
    //GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); 

    /* ���ú������õ���PB0���� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;								// �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1;				// GPIO_Pin_0 | GPIO_Pin_1 | (A0��A1����TIM5��CH12ͨ������PWM3)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ������ʱ������ */		 
    TIM_TimeBaseStructure.TIM_Period = freq - 1;       					//����ʱ����0������19999����Ϊ20000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = div - 1;	    						//����Ԥ��Ƶ72/36=2m~0.5us ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;			//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    /* PWMģʽ���� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    				//����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
    TIM_OCInitStructure.TIM_Pulse = 0;										  			//���ó�ʼPWM������Ϊ0	
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
//    TIM_OC2Init(TIM2, &TIM_OCInitStructure);										//ʹ��ͨ��1
//    TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 									//ʹ��ͨ��2
//    TIM_OC4Init(TIM2, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);						//ʹ��Ԥװ��
//    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);						//ʹ��Ԥװ��
//    TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);						//ʹ��Ԥװ��	
//    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);						//ʹ��Ԥװ��

    TIM_ARRPreloadConfig(TIM2, ENABLE);			 										//ʹ��TIM2���ؼĴ���ARR

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);                   										//ʹ�ܶ�ʱ��3	
}



/***************************/

void TIM4_PWM_PB6_PB7_PB8_PB9_Config(unsigned int freq, unsigned int div)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    /* GPIOB clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);// ʹ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    /* ���ú������õ���PB0���� */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* ������ʱ������ */		 
    TIM_TimeBaseStructure.TIM_Period = freq - 1;       					//����ʱ����0������19999����Ϊ20000�Σ�Ϊһ����ʱ����
    TIM_TimeBaseStructure.TIM_Prescaler = div -1;	    						//����Ԥ��Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;			//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /* PWMģʽ���� */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    				//����ΪPWMģʽ1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//ʹ�����
    TIM_OCInitStructure.TIM_Pulse = 0;										  			//���ó�ʼPWM������Ϊ0	
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  	  //����ʱ������ֵС��CCR1_ValʱΪ�͵�ƽ

    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 									//ʹ��ͨ��3
    TIM_OC4Init(TIM4, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);						//ʹ��Ԥװ�أ���ͬ��ͨ����Ӧ��ͬ��IO��
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);						//ʹ��Ԥװ��
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);						//ʹ��Ԥװ��	
    TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);						//ʹ��Ԥװ��

    TIM_ARRPreloadConfig(TIM4, ENABLE);			 										//ʹ��TIM4���ؼĴ���ARR�Զ���װ�Ĵ��� 
    TIM_Cmd(TIM4, ENABLE); 
}

void Timer6_Config(u16 Period,u16 Prescaler)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//		NVIC_InitTypeDef NVIC_InitStructure; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
	
    TIM_TimeBaseStructure.TIM_Period= Period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler= Prescaler - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);	
    TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);		
    TIM_Cmd(TIM6, ENABLE);		
}

void Timer6_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority)
{
		NVIC_InitTypeDef NVIC_InitStructure; 
	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/************************************************************************/
void Timer4_Config(u16 Period,u16 Prescaler)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//		NVIC_InitTypeDef NVIC_InitStructure; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
	
    TIM_TimeBaseStructure.TIM_Period= Period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler= Prescaler - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);	
    TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);		
    TIM_Cmd(TIM4, ENABLE);		
}

void Timer4_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority)
{
		NVIC_InitTypeDef NVIC_InitStructure; 
	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
void Timer5_Config(u16 Period,u16 Prescaler)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//		NVIC_InitTypeDef NVIC_InitStructure; 
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);
	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
//    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;	  
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
	
    TIM_TimeBaseStructure.TIM_Period= Period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler= Prescaler - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);	
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);	
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);		
    TIM_Cmd(TIM5, ENABLE);		
}

void Timer5_NVIC_Config(unsigned char PreemptionPriority, unsigned char SubPriority)
{
		NVIC_InitTypeDef NVIC_InitStructure; 
	
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

