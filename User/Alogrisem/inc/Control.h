#ifndef _CONTROL_H__
#define _CONTROL_H__

//include your Headerfiles
#include "stm32f10x.h"

struct Sensor_Parameter{
    /* 传感器的IIC地址 */
    unsigned char devID;
    /* 要读取的数据寄存器地址 */
    unsigned char index;
    /* 要读取的数据长度 */
    unsigned char num;
};

struct SerCommunication_Protocol{
    char headFrame;
    char secFrame;
    unsigned char dataLength;
    char index;
    char dataSum;
    char tailFrame;
};

union FLOAT2CHAR
{
	float num;
	unsigned char byte[4];
};

union SHORT2CHAR
{
	short int num;
	unsigned char byte[2];
};

typedef struct {
	int8_t dj_amp;
	int8_t dj_dseta;
	float dj_vol;
} FrameXbox360DJData;
typedef struct {
	uint16_t motor_l;
	uint16_t motor_r;
} FrameXbox360MTData;

typedef struct {
	uint8_t rom_motor_l_a_offest;
	uint8_t rom_motor_l_b_offest;
	uint8_t rom_motor_r_a_offest;
	uint8_t rom_motor_r_b_offest;
	uint8_t rom_tail_freq;
	uint8_t rom_tail_amp;
	 int8_t rom_tail_dyn;
	uint8_t rom_st_ratio;
	uint8_t rom_boom_init_posi;
} FrameROMMotionData;

void GetUart5Frame(unsigned char data);
void Glider4_Motion_Mode(void);
void PythonDataStructInit(void);
void GetPoseJY901(void);
void FrameCharGet(unsigned char data);
uint8_t FrameGet_360(uint8_t data);
void GetPoseJY901Init(float times);
void Xbox360DataProcess(uint8_t data);
void OutPut_Data(void);
#endif
