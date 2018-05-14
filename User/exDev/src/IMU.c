#include "IMU.h"
#include "stm32f10x.h"

//uart reicer flag
#define b_uart_head  0x80  //收到A5 头 标志位
#define b_rx_over    0x40  //收到完整的帧标志

volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; //接收数据缓冲区
volatile unsigned char rx_wr_index; //缓冲写指针
volatile unsigned char RC_Flag;  //接收状态标志字节
float 	yaw,  //偏航角
		pitch,//俯仰
		roll, //滚转
		alt,  //高度
		tempr,//温度
		press;//气压

int32_t lon;
int32_t lat;
int16_t hight;
int8_t  STnum;
int16_t heading;
int16_t	speed;

/********在接收完一帧IMU姿态报告后，调用这个子程序来取出姿态数据******/
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
	yaw=(float)temp / 10.0f; //偏航角
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	pitch=(float)temp / 10.0f;//俯仰
	
	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	roll=(float)temp / 10.0f;//滚转

	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	alt=(float)temp / 10.0f;//高度
	
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	tempr=(float)temp / 10.0f;//温度
	
	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	press=(float)temp * 10.0f;//气压

}

//ADC值
int16_t ax, ay, az;//加速度计
int16_t gx, gy, gz;//陀螺仪
int16_t hx, hy, hz;//磁力计
//在接收一帧ReportMotion 后调用这个子程序来取出ADC数据
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
	ax=temp;//加速度计 X轴的ADC值
	
	temp = 0;
	temp = rx_buffer[4];
	temp <<= 8;
	temp |= rx_buffer[5];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	ay=temp;//加速度计 Y轴的ADC值
	
	temp = 0;
	temp = rx_buffer[6];
	temp <<= 8;
	temp |= rx_buffer[7];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	az=temp;//加速度计 Z轴的ADC值
	
	temp = 0;
	temp = rx_buffer[8];
	temp <<= 8;
	temp |= rx_buffer[9];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gx=temp;//陀螺仪 X轴的ADC值
	
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gy=temp;//陀螺仪 Y轴的ADC值
	
	temp = 0;
	temp = rx_buffer[12];
	temp <<= 8;
	temp |= rx_buffer[13];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	gz=temp;//陀螺仪 Z轴的ADC值
	
	temp = 0;
	temp = rx_buffer[14];
	temp <<= 8;
	temp |= rx_buffer[15];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hx=temp;//磁力计 X轴的ADC值
	
	temp = 0;
	temp = rx_buffer[16];
	temp <<= 8;
	temp |= rx_buffer[17];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hy=temp;//磁力计 Y轴的ADC值
	
	temp = 0;
	temp = rx_buffer[18];
	temp <<= 8;
	temp |= rx_buffer[19];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hz=temp;//磁力计 Z轴的ADC值
}

//处理一帧接收到的GPS数据
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
	lon=ltemp;//GPS的lon
        
	ltemp = 0;
        ltemp |= (rx_buffer[6] << 24);
        ltemp |= (rx_buffer[7] << 16);
        ltemp |= (rx_buffer[8] <<  8);
	ltemp |= rx_buffer[9];
	if(ltemp&0x80000000){
	ltemp = 0-(ltemp&0x7fffffff);
	}else ltemp = (ltemp&0x7fffffff);
	lat=ltemp;//GPS的lat
        
	temp = 0;
	temp = rx_buffer[10];
	temp <<= 8;
	temp |= rx_buffer[11];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	hight=temp;//GPS的高度只
        
	temp = 0;
	temp = rx_buffer[12];
	if(temp&0x80){
	temp = 0-(temp&0x7f);
	}else temp = (temp&0x7f);
	STnum=temp;//卫星的数量
        
	temp = 0;
	temp = rx_buffer[13];
	temp <<= 8;
	temp |= rx_buffer[14];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	heading=temp;//GPS航向值
        
	temp = 0;
	temp = rx_buffer[15];
	temp <<= 8;
	temp |= rx_buffer[16];
	if(temp&0x8000){
	temp = 0-(temp&0x7fff);
	}else temp = (temp&0x7fff);
	speed=temp;//GPS速度值
}

//--校验当前接收到的一帧数据是否 与帧校验字节一致
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

//-----------中断中调用-------------
void IMU_UART_GET(unsigned char data)
{
  if (data == 0xa5)
  {
    RC_Flag|=b_uart_head; //如果接收到A5 置位帧头标专位
    rx_buffer[rx_wr_index++]=data; //保存这个字节.
  }
  else if (data == 0x5a)
  {
    if(RC_Flag&b_uart_head) //如果上一个字节是A5 那么认定 这个是帧起始字节
    {
      rx_wr_index=0;  //重置 缓冲区指针
      RC_Flag&=~b_rx_over; //这个帧才刚刚开始收
    }
    else
    {
      rx_buffer[rx_wr_index++]=data;
    }
    RC_Flag&=~b_uart_head; //清帧头标志
  }
  else
  {
    rx_buffer[rx_wr_index++]=data;
    RC_Flag&=~b_uart_head;
    if(rx_wr_index==rx_buffer[0]) //收够了字节数.(警告说明：用volatile修饰的变量一般不直接参与运算，volatile就以为着这个变量在运算过程中有可能已经改变了)
    {
      RC_Flag|=b_rx_over; //置位 接收完整的一帧数据
    }
  }
  
  if(rx_wr_index==RX_BUFFER_SIZE) //防止缓冲区溢出
  {
    rx_wr_index--;
  } 
}

//在定时器中定时更新此函数
void IMU_Data_Get(void)
{
  if(RC_Flag&b_rx_over)
  {
    RC_Flag&=~b_rx_over; //清标志先
    if(Sum_check() == 1)
    {
      //校验通过
      if(rx_buffer[1]==0xA1)//UART2_ReportIMU 的数据
      {
        Get_POSE();//取数据
      }
      if(rx_buffer[1]==0xA2)//UART2_ReportMotion 的数据
      {
        Get_Motion();//取数据
      }
      if(rx_buffer[1]==0xA3)//UART2_P 的数据
      {
        Get_GPS();//取数据
      }  
    }
  }
}


