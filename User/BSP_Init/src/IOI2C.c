#include "IOI2C.h"
#include "stdio.h"
#include "string.h"
#define IICus 5
  
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Init(void)
*��������:		��ʼ��I2C��Ӧ�Ľӿ����š�
*******************************************************************************/

void Delay(u32 count)//���ڲ���400KHzIIC�ź�����Ҫ����ʱ
{
	while (count--);
}
void IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_InitStructure;
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);			     
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	SDA_OUT();
	IIC_SDA=1;	  	  
	IIC_SCL=1;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Start(void)
*��������:		����IIC��ʼ�ź�
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	
	Delay(IICus);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	
	Delay(IICus);
	IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Stop(void)
*��������:	    //����IICֹͣ�ź�
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	
		Delay(IICus);
	IIC_SCL=1; 
	IIC_SDA=1;//����I2C���߽����ź�
	
		Delay(IICus);							   	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Wait_Ack(void)
*��������:	    �ȴ�Ӧ���źŵ��� 
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA=1;
		Delay(IICus);	  
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
		Delay(IICus);
	}  
	IIC_SCL=1;
	Delay(IICus); 
	IIC_SCL=0;//ʱ�����0  
	return 0;  
} 

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Ack(void)
*��������:	    ����ACKӦ��
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
		Delay(IICus);
	IIC_SCL=1;
		Delay(IICus);
	IIC_SCL=0;
}
	
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_NAck(void)
*��������:	    ����NACKӦ��
*******************************************************************************/	    
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	
		Delay(IICus);
	IIC_SCL=1;
		Delay(IICus);
	IIC_SCL=0;
}					 				     

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void IIC_Send_Byte(u8 txd)
*��������:	    IIC����һ���ֽ�
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
		SDA_OUT(); 	    
    IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
      IIC_SDA=(txd&0x80)>>7;
      txd<<=1; 	  

			Delay(IICus);   
			IIC_SCL=1;
			Delay(IICus);
			IIC_SCL=0;
			Delay(IICus);
    }
} 	 
   
/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IIC_Read_Byte(unsigned char ack)
*��������:	    //��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        
		Delay(IICus);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		
		Delay(IICus); 
    }					 
    if (ack)
        IIC_Ack(); //����ACK 
    else
        IIC_NAck();//����nACK  
    return receive;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*��������:	    ��ȡָ���豸 ָ���Ĵ����� length��ֵ
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫ�����ֽ���
		*data  ���������ݽ�Ҫ��ŵ�ָ��
����   ���������ֽ�����
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //�������ģʽ	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
		 	else  data[count]=IIC_Read_Byte(0);	 //���һ���ֽ�NACK
	}
    IIC_Stop();//����һ��ֹͣ����
    return count;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*��������:	    ������ֽ�д��ָ���豸 ָ���Ĵ���
����	dev  Ŀ���豸��ַ
		reg	  �Ĵ�����ַ
		length Ҫд���ֽ���
		*data  ��Ҫд�����ݵ��׵�ַ
����   �����Ƿ�ɹ�
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //����д����
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //���͵�ַ
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++)
	{
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
  }
	IIC_Stop();//����һ��ֹͣ����

    return 1; //status == 0;
	
}
#include "Control.h"
FrameROMMotionData rom_motion_data;
uint8_t ROM_UPDATE_FLAG = 0;
void Rom_Motion_Data_Init(void)
{
	rom_motion_data.rom_motor_l_a_offest = 0;
	rom_motion_data.rom_motor_l_b_offest = 0;
	rom_motion_data.rom_motor_r_a_offest = 0;
	rom_motion_data.rom_motor_r_b_offest = 0;
	rom_motion_data.rom_tail_freq = 0;
	rom_motion_data.rom_tail_amp = 0;
	rom_motion_data.rom_tail_dyn = 0;
	rom_motion_data.rom_st_ratio = 0;
	rom_motion_data.rom_boom_init_posi = 0;
}
//0xA0
void ROM_MOTION_DATA_WRITE(void)
{
	uint16_t struct_len = 0;
	uint8_t buffer[128] = {0};
	uint8_t i = 0;
	struct_len = sizeof(rom_motion_data);
	memcpy(&buffer, &rom_motion_data, struct_len);
	while(!IICwriteBytes(0xa0, 0x00, struct_len, buffer));
}

void ROM_MOTION_DATA_READ(void)
{
	uint16_t struct_len = 0;
	uint8_t buffer[128] = {0};
	uint8_t i = 0;
	struct_len = sizeof(rom_motion_data);
	while(!IICwriteBytes(0xa0, 0x00, struct_len, buffer));
	memcpy(&rom_motion_data, &buffer, struct_len);
}