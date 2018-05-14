#ifndef _ms5837_H
#define _ms5837_H

#include "IOI2C.h"



void setFluidDensity(float); 
void ResetForMs5837();
unsigned int MS5837_init();
uint8_t crc4(uint16_t *n_prom);
void calculat();
void MS_WriteOneByte(u8,u8);
float pressure(float);
float	temperature();
float depth(); 
void MS5837_read();

void MS5837DeviceInit(void);

int filter_Volt();


#endif