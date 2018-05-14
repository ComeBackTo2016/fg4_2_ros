#ifndef _IMU_H__
#define _IMU_H__

//include your Headerfiles
#include "stm32f10x.h"

//include your Macro definitions

#define RX_BUFFER_SIZE 100 //���ջ������ֽ���   // USART Receiver buffer

//include your externfun
void IMU_UART_GET(unsigned char data);
void IMU_Data_Get(void);
void Get_POSE(void);
//include your externvariable
extern volatile unsigned char rx_buffer[RX_BUFFER_SIZE]; //�������ݻ�����
extern float 	        yaw,  //ƫ����
                        pitch,//����
                        roll, //��ת
                        alt,  //�߶�
                        tempr,//�¶�
                        press;//��ѹ

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
