#ifndef __INIT_H
#define	__INIT_H

#include <stdio.h>
#include <string.h>
#include "stm32f10x.h"
#include "bsp_SysTick.h"
#include "bsp_spi_flash.h"
#include "bsp_adc.h"
#include "bsp_led.h" 
#include "timer.h"
#include "uart.h"
#include "encode.h"
#include "IOI2C.h"
#include "CPG.h"
#include "IMU.h"
#include "Control.h"
#include "AHRSREG.h"
#include "SerialDMA.h"
#include "Fifo4serial.h" 


//#define LED0	4
//#define LED1	5
//#define LED2	6
//#define LED3	7

#define LED_OFF(n)   BIT_ADDR(GPIOA_ODR_Addr,n)  // ‰≥ˆ 
#define LED_ON(n)    BIT_ADDR(GPIOA_IDR_Addr,n)  // ‰»Î

#define     FSTEP_DIR(n)        PCout(12)  = n
#define     FSTEP_MS3(n)        PBout(5)  = n
#define     FSTEP_MS2(n)        PBout(4)  = n
#define     FSTEP_MS1(n)        PBout(3)  = n
#define     FSTEP_EN(n)         PDout(2)  = n

#define AIN1(n) PBout(13) = n
#define AIN2(n) PBout(12) = n
#define BIN1(n) PBout(14) = n
#define BIN2(n) PBout(15) = n
#define STBY(n) PAout(15) = n


extern float IMUData[];

void UART_INIT(void);

void NVIC_of_All(void);

void PWM_INIT(void);
void ENCODE_INIT(void);
void TIMER_INIT(void);

void IIC2_INIT(void);
void IIC2_Config(void);
void IIC2_NVIC_Config(unsigned char Pre_EV, unsigned char Pre_ER);
void IIC1_INIT(void);
void IIC1_Config(void);
void IIC1_NVIC_Config(unsigned char Pre_EV, unsigned char Pre_ER);

void LED_Config(void);
void Common_GPIO_Config(void);
void Exit_Config(void);







#endif /* __INIT_H */
