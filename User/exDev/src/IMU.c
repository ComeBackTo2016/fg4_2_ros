#include "IMU.h"
#include "stm32f10x.h"

//uart reicer flag
#define b_uart_head  0x80  //�յ�A5 ͷ ��־λ
#define b_rx_over    0x40  //�յ�������֡��־

volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; //�������ݻ�����
volatile unsigned char rx_wr_index; //����дָ��
volatile unsigned char RC_Flag;  //����״̬��־�ֽ�
float 	yaw,  //ƫ����
		pitch,//����
		roll, //��ת
		alt,  //�߶�
		tempr,//�¶�
		press;//��ѹ

int32_t lon;
int32_t lat;
int16_t hight;
int8_t  STnum;
int16_t heading;
int16_t	speed;

/********�ڽ�����һ֡IMU��̬����󣬵�������ӳ�����ȡ����̬����******/
void Get_POSE(void)
{
	int16_t temp;
	
	temp = 0;
	temp = rx_buffer[2];
	temp <<= 8;
	temp |= rx_buffer[3];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	yaw=(float)temp / 10.0f; //ƫ����
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	pitch=(float)temp / 10.0f;//����
	
	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	roll=(float)temp / 10.0f;//��ת

	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	alt=(float)temp / 10.0f;//�߶�
	
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	tempr=(float)temp / 10.0f;//�¶�
	
	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	press=(float)temp * 10.0f;//��ѹ

}

//ADCֵ
int16_t ax, ay, az;//���ٶȼ�
int16_t gx, gy, gz;//������
int16_t hx, hy, hz;//������
//�ڽ���һ֡ReportMotion ���������ӳ�����ȡ��ADC����
void Get_Motion(void)
{
	int16_t temp;
	
	temp = 0;
	temp = rx_buffer[2];
	temp <<= 8;
	temp |= rx_buffer[3];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	ax=temp;//���ٶȼ� X���ADCֵ
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	ay=temp;//���ٶȼ� Y���ADCֵ
	
	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	az=temp;//���ٶȼ� Z���ADCֵ
	
	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gx=temp;//������ X���ADCֵ
	
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gy=temp;//������ Y���ADCֵ
	
	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gz=temp;//������ Z���ADCֵ
	
	temp = 0;
	temp = rx_buffer[14];
	temp <<= 8;
	temp |= rx_buffer[15];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hx=temp;//������ X���ADCֵ
	
	temp = 0;
	temp = rx_buffer[16];
	temp <<= 8;
	temp |= rx_buffer[17];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hy=temp;//������ Y���ADCֵ
	
	temp = 0;
	temp = rx_buffer[18];
	temp <<= 8;
	temp |= rx_buffer[19];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hz=temp;//������ Z���ADCֵ
}

//����һ֡���յ���GPS����
void Get_GPS(void)
{
	int16_t temp;
        int32_t ltemp;
	
	ltemp = 0;
        ltemp |= (rx_buffer[2] << 24);
        ltemp |= (rx_buffer[3] << 16);
        ltemp |= (rx_buffer[4] <<  8);
	ltemp |= rx_buffer[5];
	if(ltemp&0x80000000){
	ltemp = 0-(ltemp&0x7fffffff);
	}else ltemp = (ltemp&0x7fffffff);
	lon=ltemp;//GPS��lon
        
	ltemp = 0;
        ltemp |= (rx_buffer[6] << 24);
        ltemp |= (rx_buffer[7] << 16);
        ltemp |= (rx_buffer[8] <<  8);
	ltemp |= rx_buffer[9];
	if(ltemp&0x80000000){
	ltemp = 0-(ltemp&0x7fffffff);
	}else ltemp = (ltemp&0x7fffffff);
	lat=ltemp;//GPS��lat
        
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hight=temp;//GPS�ĸ߶�ֻ
        
	temp = 0;
	temp = rx_buffer[12];
	if(temp&0x80){
	temp = 0-(temp&0x7f);
	}else temp = (temp&0x7f);
	STnum=temp;//���ǵ�����
        
	temp = 0;
	temp = rx_buffer[13];
	temp <<= 8;
	temp |= rx_buffer[14];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	heading=temp;//GPS����ֵ
        
	temp = 0;
	temp = rx_buffer[15];
	temp <<= 8;
	temp |= rx_buffer[16];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	speed=temp;//GPS�ٶ�ֵ
}

//--У�鵱ǰ���յ���һ֡�����Ƿ� ��֡У���ֽ�һ��
unsigned char Sum_check(void)
{ 
  unsigned char i;
  unsigned int checksum=0; 
  for(i=0;i<rx_buffer[0]-2;i++)
   checksum+=rx_buffer[i];
  if((checksum%256)==rx_buffer[rx_buffer[0]-2])
   return(0x01); //Checksum successful
  else
   return(0x00); //Checksum error
}

//-----------�ж��е���-------------
void IMU_UART_GET(unsigned char data)
{
  if (data == 0xa5)
  {
    RC_Flag|=b_uart_head; //������յ�A5 ��λ֡ͷ��רλ
    rx_buffer[rx_wr_index++]=data; //��������ֽ�.
  }
  else if (data == 0x5a)
  {
    if(RC_Flag&b_uart_head) //�����һ���ֽ���A5 ��ô�϶� �����֡��ʼ�ֽ�
    {
      rx_wr_index=0;  //���� ������ָ��
      RC_Flag&=~b_rx_over; //���֡�Ÿոտ�ʼ��
    }
    else
    {
      rx_buffer[rx_wr_index++]=data;
    }
    RC_Flag&=~b_uart_head; //��֡ͷ��־
  }
  else
  {
    rx_buffer[rx_wr_index++]=data;
    RC_Flag&=~b_uart_head;
    if(rx_wr_index==rx_buffer[0]) //�չ����ֽ���.(����˵������volatile���εı���һ�㲻ֱ�Ӳ������㣬volatile����Ϊ���������������������п����Ѿ��ı���)
    {
      RC_Flag|=b_rx_over; //��λ ����������һ֡����
    }
  }
  
  if(rx_wr_index==RX_BUFFER_SIZE) //��ֹ���������
  {
    rx_wr_index--;
  } 
}

//�ڶ�ʱ���ж�ʱ���´˺���
void IMU_Data_Get(void)
{
  if(RC_Flag&b_rx_over)
  {
    RC_Flag&=~b_rx_over; //���־��
    if(Sum_check() == 1)
    {
      //У��ͨ��
      if(rx_buffer[1]==0xA1)//UART2_ReportIMU ������
      {
        Get_POSE();//ȡ����
      }
      if(rx_buffer[1]==0xA2)//UART2_ReportMotion ������
      {
        Get_Motion();//ȡ����
      }
      if(rx_buffer[1]==0xA3)//UART2_P ������
      {
        Get_GPS();//ȡ����
      }  
    }
  }
}


