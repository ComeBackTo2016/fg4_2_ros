#include "encode.h"

#define ENCODER_PERIOD	10000  //ENCODER_PERIOD  ENCODER_PRES
#define ENCODER_PRES	0

/**************************************************************
以下代码均来自amobbs
**************************************************************/

/**************************************************************************
函数功能:定时器1编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM1_EncoderMode_PE9_PE11_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /*----------------------------------------------------------------*/

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PE.09,11 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 
    TIM_DeInit(TIM1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);    

    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;      
    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;           
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;       
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    /*-----------------------------------------------------------------*/
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);		

    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //TIM_ICPolarity_Rising?????
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;       
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
		TIM_ClearFlag(TIM1, TIM_FLAG_Update); 
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
    //Reset counter
    TIM1->CNT = 10000;
    TIM_Cmd(TIM1, ENABLE);   
}

/************************************/

/**************************************************************************
函数功能:定时器1编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM1_EncoderMode_PA8_PA9_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
    /*----------------------------------------------------------------*/

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //??TIM1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);  
		TIM_DeInit(TIM1);

    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;    
    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;          
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
    /*-----------------------------------------------------------------*/
    TIM_CounterModeConfig(TIM1, TIM_CounterMode_Up);
//    TIM_TIxExternalClockConfig(TIM1, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);
//    TIM_EncoderInterfaceConfig(TIM1, TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge); //TIM_ICPolarity_Rising?????
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;     
    TIM_ICInit(TIM1, &TIM_ICInitStructure);
		
		TIM_ClearFlag(TIM1, TIM_FLAG_Update); 
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); 
    //Reset counter
    TIM1->CNT = 0;
    TIM_Cmd(TIM1, ENABLE);  
}
/**************************************/
/**************************************************************************
函数功能:定时器2编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
/*定时器2配置为编码器模式，管教未印射*/
void TIM2_EncoderMode_PA0_PA1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*----------------------------------------------------------------*/
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA.0 1 encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //??
    TIM_DeInit(TIM2);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;      //
    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;           //?????:
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;       //????????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //??????

    /*???TIM2??? */
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
        
//    TIM_TIxExternalClockConfig(TIM2, TIM_TIxExternalCLK1Source_TI2, TIM_ICPolarity_Rising, 0);
		TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;         
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
		
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); 
		
    TIM2->CNT = 5000;
    TIM_Cmd(TIM2, ENABLE);   
}
/**************************************************************************
函数功能:定时器2编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM2_EncoderMode_PA15_PB3_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    //TIM_OCInitTypeDef  TIM_OCInitStructure;
    /*----------------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA15 encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PB3 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
//
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); //??
    TIM_DeInit(TIM2);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//??JTAG,??PA15?JTAG??
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = 0xffff;      //
    TIM_TimeBaseStructure.TIM_Prescaler = 0;           //?????:
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;       //????????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //??????
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    /*-----------------------------------------------------------------*/
    //????                        ????
    TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //TIM_ICPolarity_Rising?????
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;         //?????
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);   //?????2

}
/**************************************************************************
函数功能:定时器3编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM3_EncoderMode_PA6_PA7_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*----------------------------------------------------------------*/

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA6,7 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //TIM3
    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;      //
    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;           //?????:
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;       //????????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //??????
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
    /*-----------------------------------------------------------------*/
    //????                        ????
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //TIM_ICPolarity_Rising?????
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;         //?????
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
		
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); 
		
    //Reset counter
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);   //?????3
}
/**************************************************************************
函数功能:定时器3编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM3_EncoderMode_PC6_PC7_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    /*----------------------------------------------------------------*/

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA.06,07 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //??TIM3
    TIM_DeInit(TIM3);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xffff;      //
    TIM_TimeBaseStructure.TIM_Prescaler = 0;           //?????:
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;       //????????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //??????
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    /*???TIM3??? */
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /*-----------------------------------------------------------------*/
    //????                        ????
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //TIM_ICPolarity_Rising?????
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;         //?????
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    //Reset counter
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);   //?????3
}
/**************************************************************************
函数功能:定时器4编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM4_EncoderMode_PB6_PB7_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    /*----------------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PB6,7 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); //??TIM4
    TIM_DeInit(TIM4);  

		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;     
    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;        
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;   
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    /*-----------------------------------------------------------------*/
//    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); 
		TIM_TIxExternalClockConfig(TIM4, TIM_TIxExternalCLK1Source_TI1, TIM_ICPolarity_Rising, 0);
		
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;         
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);   
    //Reset counter
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);   
}
/**************************************************************************
函数功能:定时器4编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
/*********************************/
void TIM4_EncoderMode_PD12_PD13_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PB6,7 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
    TIM_DeInit(TIM4);
	
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 
	
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	
    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;     
    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;    //ENCODER_PERIOD  ENCODER_PRES
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;     
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //TIM_ICPolarity_Rising?????

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;        
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);   
		
    //Reset counter
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);  
}
/*********************************/
/**************************************************************************
函数功能:定时器5编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM5_EncoderMode_PA0_PA1_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    //TIM_OCInitTypeDef  TIM_OCInitStructure;
    /*----------------------------------------------------------------*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA.0.1 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    /*----------------------------------------------------------------*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //??tim5
    TIM_DeInit(TIM5);
    //  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_TimeBaseStructure.TIM_Period = 0xffff;      //
    TIM_TimeBaseStructure.TIM_Prescaler = 0;           //?????:
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;       //????????:???
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //??????
    //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    /*???TIM5??? */
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    /*-----------------------------------------------------------------*/
    //????                        ????
    TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //TIM_ICPolarity_Rising?????
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;         //?????
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    //Reset counter
    TIM5->CNT = 0;
    TIM_Cmd(TIM5, ENABLE);   //?????3
}
/**************************************************************************
函数功能:定时器8编码器功能初始化
入口参数:无
返 回 值:无
作    者:何宇帆(2015-12-12)
**************************************************************************/
void TIM8_EncoderMode_PC6_PC7_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PC.06,07 as encoder input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
    TIM_DeInit(TIM8);

    TIM_TimeBaseStructure.TIM_Prescaler = ENCODER_PRES;  // No prescaling
    TIM_TimeBaseStructure.TIM_Period = ENCODER_PERIOD;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
		
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

    TIM_EncoderInterfaceConfig(TIM8, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIM8, &TIM_ICInitStructure);
		
    TIM_ClearFlag(TIM8, TIM_FLAG_Update);  //???????
    TIM_ITConfig(TIM8, TIM_IT_Update, ENABLE); //??????
		
    TIM8->CNT = 0;
    TIM_Cmd(TIM8, ENABLE);
}

//#define prd    5000
//#define Vbreak 4000
//uint16_t cnt1;
//int32_t CNT1, V1, rcnt1;

//void get_encoder_one(void)
//{
//  s32 CNT1_temp,CNT1_last;
//  
//  cnt1 = TIM2->CNT;
//  CNT1_last = CNT1;
//  CNT1_temp = rcnt1 * prd + cnt1;  
//  V1 = CNT1_temp - CNT1_last;		
//  
//  while (V1>Vbreak)				 
//  {							      
//		 rcnt1--;					      
//		 CNT1_temp = rcnt1 * prd + cnt1;
//		 V1 = CNT1_temp - CNT1_last;		 
//  }							     
//  while (V1<-Vbreak)			   
//  {							      
//		 rcnt1++;					      
//		 CNT1_temp = rcnt1 * prd + cnt1;
//		 V1 = CNT1_temp - CNT1_last;		 
//  }
//  CNT1 = CNT1_temp;						 
//}
