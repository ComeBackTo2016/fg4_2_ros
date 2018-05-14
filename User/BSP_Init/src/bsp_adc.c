 #include "bsp_adc.h"
  #include "init.h"

#define ADC1_DR_Address    ((u32)0x40012400+0x4c)

#define N_sample 1
#define N_channel 5

__IO uint16_t sampleValue[N_sample][N_channel];
__IO uint16_t fliterValue[N_channel];

__IO uint16_t ADC_ConvertedValue;

/**
  * @brief  ʹ��ADC1��DMA1��ʱ�ӣ���ʼ���ɼ��˿�
  * @param  ��
  * @retval ��
  */
static void ADC1_GPIO_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		
		/* Enable DMA clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		/* Enable ADC1 and GPIOC clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOA, ENABLE);
		
		/*Configure port  as analog input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/**
  * @brief  ����ADC1�Ĺ���ģʽΪMDAģʽ
  * @param  ��
  * @retval ��
  */
static void ADC1_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	/* DMA channel1 configuration */
	DMA_DeInit(DMA1_Channel1);
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;	 	//ADC��ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&fliterValue;	        //�ڴ��ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;					//set the periphera add as the source add
	DMA_InitStructure.DMA_BufferSize = N_channel;					//����DMA��������С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�����ַ�̶�
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  					//�ڴ��ַ�̶�
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//�����������ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;					//�����ڴ���Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;															//ѭ�����䣨���λ��壩
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;													//DMA���ȼ���
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;																//��ֹDMAͨ���洢�����洢������
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);																//
	
	/* ADC1 configuration */

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;			//����ADCģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE ; 	 				//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			//��������ת��ģʽ������ͣ�ؽ���ADCת��
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//��ʹ���ⲿ����ת��
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 	//�ɼ������Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 5;	 								//Ҫת����ͨ����Ŀ8
	ADC_Init(ADC1, &ADC_InitStructure);
	
	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9MHz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
	/*����ADC1��ͨ��11Ϊ55.	5���������ڣ�����Ϊ1 */ 
   	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 3, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 4, ADC_SampleTime_55Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_4 , 5, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_5 , 6, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 7, ADC_SampleTime_55Cycles5);
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 8, ADC_SampleTime_55Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_0 , 1, ADC_SampleTime_55Cycles5);
//   	ADC_RegularChannelConfig(ADC1, ADC_Channel_4 , 2, ADC_SampleTime_55Cycles5);
    /* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);
	
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	
	/*��λУ׼�Ĵ��� */   
	ADC_ResetCalibration(ADC1);
	/*�ȴ�У׼�Ĵ�����λ��� */
	while(ADC_GetResetCalibrationStatus(ADC1));
	
	/* ADCУ׼ */
	ADC_StartCalibration(ADC1);
	/* �ȴ�У׼���*/
	while(ADC_GetCalibrationStatus(ADC1));
	
	/* Enable DMA channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);	
	/* ��ͨ��ADA����ʱ��Ҫ��dmaʹ�ܷŵ�adc��У׼֮��ԭ����adc��У׼
	   �ᴥ��dmaͨ����û������������ǻᵼ��dmaͨ����ַ�Լӣ����ᵼ
		 �����ݴ�λ */
	
	/* ����û�в����ⲿ����������ʹ���������ADCת�� */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

/**
  * @brief  ADC1��ʼ��
  * @param  ��
  * @retval ��
  */
void ADC1_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_Mode_Config();
}
/*********************************************END OF FILE**********************/
void ADC_Fliter(void)
{

}

