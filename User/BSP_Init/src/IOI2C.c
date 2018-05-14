#include "IOI2C.h"
#include "stdio.h"
#include "string.h"
#define IICus 5
  
/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/

void Delay(u32 count)//用于产生400KHzIIC信号所需要的延时
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

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	
	Delay(IICus);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	
	Delay(IICus);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	
		Delay(IICus);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	
		Delay(IICus);							   	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0; 
	SDA_IN();      //SDA设置为输入  
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
	IIC_SCL=0;//时钟输出0  
	return 0;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
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
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
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

/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t; 
		SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
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
   
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
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
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
}

/**************************实现函数********************************************
*函数原型:		u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
*功　　能:	    读取指定设备 指定寄存器的 length个值
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要读的字节数
		*data  读出的数据将要存放的指针
返回   读出来的字节数量
*******************************************************************************/ 
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data){
    u8 count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //进入接收模式	
	IIC_Wait_Ack();
	
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    IIC_Stop();//产生一个停止条件
    return count;
}

/**************************实现函数********************************************
*函数原型:		u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
*功　　能:	    将多个字节写入指定设备 指定寄存器
输入	dev  目标设备地址
		reg	  寄存器地址
		length 要写的字节数
		*data  将要写的数据的首地址
返回   返回是否成功
*******************************************************************************/ 
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data){
  
 	u8 count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++)
	{
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
  }
	IIC_Stop();//产生一个停止条件

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