#ifndef  __ADS115_H_
#define  __ADS115_H_
#include "IOI2C.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x.h"


/***************************************************************************************
*说明：当端输入时候正输入为输入信号，负输入为地但是输入信号不能为负电压(不能比地电位低)
*	   双端输入时候正输入为输入信号，负输入为负输入输入信号的差值可以为负电压
****************************************************************************************/
#define	 SDA_A1     PBout(11)=1   //SDA输出
#define	 SDA_A0     PBout(11)=0
#define	 SCL_A1	    PBout(10)=1    //SCL
#define	 SCL_A0	    PBout(10)=0
#define	 SDA_AI	    PBin(11)   //SDA读入

//I2C地址以及读写设置
//#define  WR_REG 0x92       //写寄存器   ADDR_pin--VCC  1001001 0
//#define  RE_REG 0x93       //读寄存器		ADDR_pin--VCC  1001001 1


/***********************************寄存器控制字**********************************************/
#define  DATA_REG  0x00		//转换数据寄存器
#define  CONF_REG  0x01     //控制字设置寄存器
#define  LOTH_REG  0x02		//最低阀值寄存器
#define  HITH_REG  0x03		//最高阀值寄存器

#define  ch0  0xc0       //通道0
#define  ch1  0xd0       //通道1
#define  ch2  0xe0       //通道2
#define  ch3  0xf0       //通道3

/***********************控制字申明*************************************************************
*|  OS | MUX2 | MUX1 | MUX0 | PGA2 | PGA1 | PGA0 | MODE  |------HCMD
*|  DR2| DR1  | DR0  | COMP_MODE | COMP_POL | COMP_LAT |  COMP_QUE1 | COMP_QUE0 |-----LCMD
***********************************************************************************************/
#define  HCMD0    0xc2   //单端输入  AIN0 +/-6.144量程  连续模式  1 100 000 0b
#define  HCMD1    0xd2   //AIN1                             1101 000 0b
#define  HCMD2   	0xe2  //AIN2                             	1110 000 0b
#define  HCMD3    0xf2   //AIN3                             1111 000 0b
#define  LCMD1	  0xf0	 //860sps 窗口比较器模式 输出低有效  不锁存信号至读 每周期检测阀值 11110000b

/************************函数申明****************************/
static void ADS1115_delay(u16 D);
void delay_nms(u16 ms);
void delay_nus(u16 us);
void ADS1115_Init(void);
void I2CStart_A(void);
void I2CStop_A(void);
void I2CWriteByte_A(u8 DATA);
u8 I2CReadByte_A(void);
void ADS1115Config_A(u8 LCMD,u8 HCMD);
void SetThresHold_A(u16 L_TH,u16 H_TH);        //高低阀门设置
u16 ReadAD_A(void);
int16_t getad(u8 LCMD,u8 HCMD);
u16 lvbo(u8 LCMD,u8 HCMD);





#endif		
