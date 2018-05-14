#ifndef _IMU_H__
#define _IMU_H__

//include your Headerfiles
#include "stm32f10x.h"

//include your Macro definitions

#define RX_BUFFER_SIZE 100 //接收缓冲区字节数   // USART Receiver buffer

//include your externfun
void IMU_UART_GET(unsigned char data);
void IMU_Data_Get(void);
void Get_POSE(void);
//include your externvariable
extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; //接收数据缓冲区
extern float 	        yaw,  //偏航角
                        pitch,//俯仰
                        roll, //滚转
                        alt,  //高度
                        tempr,//温度
                        press;//气压

extern int32_t lon;
extern int32_t lat;
extern int16_t hight;
extern int8_t  STnum;
extern int16_t heading;
extern int16_t	speed;

struct SLonLat
{
	long lLon;
	long lLat;
};

struct SGPSV
{
	short sGPSHeight;
	short sGPSYaw;
	long lGPSVelocity;
};

#endif
