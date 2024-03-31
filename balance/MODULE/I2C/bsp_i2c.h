#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f10x.h"

void Software_I2C_GPIO_Config(void);	

int Sensors_I2C_WriteRegister(uint8_t slave_address,uint8_t register_address,uint8_t length,const uint8_t *data);
int Sensors_I2C_ReadRegister(uint8_t slave_address,uint8_t register_address,uint8_t length, uint8_t *data);

#endif
