#include "ads1115.h"
#include "init.h"

//#define  WR_REG 0x90       //写寄存器   ADDR_pin--GND  1001000 0 0x90  1001001 0 0x92
//#define  RE_REG 0x91       //读寄存器		ADDR_pin--GND  1001000 1 0x91  1001001 1 0x93

unsigned char WR_REG = 0x92;
unsigned char RE_REG = 0x93;

static void ADS1115_delay(u16 D)
{
	while(--D);
}

void delay_nms(u16 ms)
{
	u16 i;
	u32 M = 0;//720W
	for(i = 0;i < ms; i++)
	for(M=12000;M > 0;M--);
}

void delay_nus(u16 us)
{
	u16 i;
	u16 M = 0;//720W
	for(i = 0;i < us; i++)
	for(M=72;M > 0;M--);
}



/////////////////PA8 SDA////PA9 SCL///////////////////////////////////
void ADS1115_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB ,ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;//A SCL SDA
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

  SDA_A1;
  SCL_A1;
	delay_nms(5);

}

//I2C总线启动
//void I2CStart_A(void)
//{
//   SDA_A1;
//   ADS1115_delay(5);
//   SCL_A1;
//   ADS1115_delay(5);
//   SDA_A0;
//   ADS1115_delay(5);//MIN 160ns
//   SCL_A0;
//   ADS1115_delay(5);
//}




//I2C停止总线
//void I2CStop_A(void)
//{
//   SDA_A0;
//   ADS1115_delay(5);
//   SCL_A1;
//   ADS1115_delay(5);
//   SDA_A1;
//   ADS1115_delay(5);//MIN 160ns
//}



//I2C 写一字节
void I2CWriteByte_A(u8 DATA)
{			
		u8 t; 
		SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
			IIC_SDA=(DATA&0x80)>>7;
			DATA<<=1; 	  
			
			Delay_us(2);   
			IIC_SCL=1;
			Delay_us(5);
			IIC_SCL=0;	
			Delay_us(3);
    }	 
}

/*********************************************************************
*函数名称:  ADS1115Config
*描	   述： 设置ADS1115包括通道配置，采样时间等等
*参	   数： HCMD ：命令字高8位(通道，量程，转换模式)
			LCMD : 命令字低8位(采样率设置 比较模式 有效电平 信号输出锁存)
*返	   回； 无
********************************************************************/
void ADS1115Config_A(u8 LCMD,u8 HCMD)
{
    u8 i=0;
    u8 Initdata[4];

    Initdata[0] = WR_REG;  // 地址0x90  器件ADR接地 写寄存器
    Initdata[1] = CONF_REG;// 配置寄存器
    Initdata[2] = HCMD;    // 配置字高字节
    Initdata[3] = LCMD;    // 配置字低字节
    
		IIC_Start();						//start
    for(i=0;i<4;i++)
    {
    	IIC_Send_Byte(Initdata[i]);
			IIC_Wait_Ack();
    	Delay_us(5);
    }
   IIC_Stop();         			//start
}
/*
void SetThresHold_A(u16 L_TH,u16 H_TH)        //高低阀门设置
{
   SCL_A1;
   I2CStart_A();      // 开启
   I2CWriteByte_A(WR_REG);
   I2CWriteByte_A(LOTH_REG);//最低阀值寄存器
   I2CWriteByte_A((L_TH>>8));
   I2CWriteByte_A(L_TH);
   I2CStop_A();       //关闭

   I2CStart_A();     //开启
   I2CWriteByte_A(WR_REG);
   I2CWriteByte_A(HITH_REG);//最高阀值寄存器
   I2CWriteByte_A((H_TH>>8));
   I2CWriteByte_A(H_TH);
   I2CStop_A();      //关闭
}
*/

/*******************************************************************
*函数名称:  ReadAD_A
*描	   述： 获取AD转换的值
*参	   数： 获取的值为在前面设置的那个通道
*返	   回； 无
********************************************************************/
u16 ReadAD_A(void)
{
   u16 Data[2]={0,0};

   IIC_Start();						//start
   IIC_Send_Byte(WR_REG);
	 IIC_Wait_Ack();
   IIC_Send_Byte(DATA_REG);
	 IIC_Wait_Ack();
   IIC_Stop();
   
   IIC_Start();
   IIC_Send_Byte(RE_REG);
	 IIC_Wait_Ack();
	 
   Data[0] = IIC_Read_Byte(1);			
   Data[1] = IIC_Read_Byte(1);
   IIC_Stop();
   
   Data[0] = Data[0]<<8 | Data[1];
   return  (Data[0]);//&0x7fff
}
//return (float)(getad(0xc3, ch) * cal_temp);
extern float IRData[3];	
int16_t getad(u8 LCMD,u8 HCMD)
{
		int16_t value=0;
		static uint16_t cnt = 0;
		cnt++;
		if (cnt == 1)
		{
			ADS1115Config_A(LCMD,HCMD);		  //AINP = AIN0 and AINN = AIN1 (default)
		}
    else if (cnt == 50)
		{
			value=ReadAD_A();
		}
		else if (cnt == 100)
		{
			cnt = 0;
		}
//    delay_nms(4); 
    return value;
}
//int16_t getad(u8 LCMD,u8 HCMD)
//{
//   int16_t value=0;
//		static uint16_t cnt = 0;
//		cnt++;
//		if (cnt == 1)
//		{
//			ADS1115Config_A(LCMD,HCMD);		  //AINP = AIN0 and AINN = AIN1 (default)
//		}
//    else if (cnt == 50)
//		{
//			value=ReadAD_A();
//		}
//		else if (cnt == 100)
//		{
//			cnt = 0;
//		}
////    delay_nms(4); 
//    return value;
//}

u16 lvbo(u8 LCMD,u8 HCMD)        //求平均值
{
	int k;
	float U=0,temp;
	
	 
	for(k=0;k<500;k++)
	{
		U+=getad(LCMD,HCMD);	
	}
	temp=U;
		U=0;
	return (temp/500);	
}






