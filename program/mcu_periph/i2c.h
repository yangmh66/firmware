/* #include "stm32f4_i2c.h" */

#ifndef FILE_I2C_H
#define FILE_I2C_H

#include "stm32f4xx.h"

#define EEPROM_A2_PIN_GROUP GPIOB
#define EEPROM_A2_PIN GPIO_Pin_7

void i2c_Init(void);
void enable_i2c1(void);
void enable_i2c2(void);
void i2c1_reinit(void);

int i2c_flag_loop_check(I2C_TypeDef* i2c_channel, uint32_t flag, int retry_count);
void i2c1_send(uint8_t *data, int data_count);

#endif
