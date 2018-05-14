#ifndef  __ADS115_H_
#define  __ADS115_H_
#include "IOI2C.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x.h"


/***************************************************************************************
*˵������������ʱ��������Ϊ�����źţ�������Ϊ�ص��������źŲ���Ϊ����ѹ(���ܱȵص�λ��)
*	   ˫������ʱ��������Ϊ�����źţ�������Ϊ�����������źŵĲ�ֵ����Ϊ����ѹ
****************************************************************************************/
#define	 SDA_A1     PBout(11)=1   //SDA���
#define	 SDA_A0     PBout(11)=0
#define	 SCL_A1	    PBout(10)=1    //SCL
#define	 SCL_A0	    PBout(10)=0
#define	 SDA_AI	    PBin(11)   //SDA����

//I2C��ַ�Լ���д����
//#define  WR_REG 0x92       //д�Ĵ���   ADDR_pin--VCC  1001001 0
//#define  RE_REG 0x93       //���Ĵ���		ADDR_pin--VCC  1001001 1


/***********************************�Ĵ���������**********************************************/
#define  DATA_REG  0x00		//ת�����ݼĴ���
#define  CONF_REG  0x01     //���������üĴ���
#define  LOTH_REG  0x02		//��ͷ�ֵ�Ĵ���
#define  HITH_REG  0x03		//��߷�ֵ�Ĵ���

#define  ch0  0xc0       //ͨ��0
#define  ch1  0xd0       //ͨ��1
#define  ch2  0xe0       //ͨ��2
#define  ch3  0xf0       //ͨ��3

/***********************����������*************************************************************
*|  OS | MUX2 | MUX1 | MUX0 | PGA2 | PGA1 | PGA0 | MODE  |------HCMD
*|  DR2| DR1  | DR0  | COMP_MODE | COMP_POL | COMP_LAT |  COMP_QUE1 | COMP_QUE0 |-----LCMD
***********************************************************************************************/
#define  HCMD0    0xc2   //��������  AIN0 +/-6.144����  ����ģʽ  1 100 000 0b
#define  HCMD1    0xd2   //AIN1                             1101 000 0b
#define  HCMD2   	0xe2  //AIN2                             	1110 000 0b
#define  HCMD3    0xf2   //AIN3                             1111 000 0b
#define  LCMD1	  0xf0	 //860sps ���ڱȽ���ģʽ �������Ч  �������ź����� ÿ���ڼ�ֵⷧ 11110000b

/************************��������****************************/
static void ADS1115_delay(u16 D);
void delay_nms(u16 ms);
void delay_nus(u16 us);
void ADS1115_Init(void);
void I2CStart_A(void);
void I2CStop_A(void);
void I2CWriteByte_A(u8 DATA);
u8 I2CReadByte_A(void);
void ADS1115Config_A(u8 LCMD,u8 HCMD);
void SetThresHold_A(u16 L_TH,u16 H_TH);        //�ߵͷ�������
u16 ReadAD_A(void);
int16_t getad(u8 LCMD,u8 HCMD);
u16 lvbo(u8 LCMD,u8 HCMD);





#endif		
