#include "stm32f10x_i2c.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void I2C_StartTransmission(I2C_TypeDef* I2Cx, uint8_t transmissionDirection,  uint8_t slaveAddress);

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);

void I2C_Write_Data(I2C_TypeDef* I2Cx, uint8_t data);

uint8_t I2C_Read_ack(I2C_TypeDef* I2Cx);

uint8_t I2C_Read_nack(I2C_TypeDef* I2Cx);
