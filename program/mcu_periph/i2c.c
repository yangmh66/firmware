#include "stm32f4xx_conf.h"	
#include "i2c.h"
#include "AT24C04C.h"

#include "global.h"

void enable_i2c1()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	/* SCL = PB8, SDA = PB9 */
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_PuPd = GPIO_PuPd_NOPULL
	};
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = EEPROM_A2_PIN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(EEPROM_A2_PIN_GROUP, &GPIO_InitStruct);
	GPIO_SetBits(EEPROM_A2_PIN_GROUP, EEPROM_A2_PIN);

	I2C_InitTypeDef I2C_InitStruct = {
		.I2C_ClockSpeed = 400000,
		.I2C_Mode = I2C_Mode_I2C,
		.I2C_DutyCycle = I2C_DutyCycle_2,
		.I2C_OwnAddress1 = 0x0A,
		.I2C_Ack = I2C_Ack_Enable,
		.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
	};
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_Cmd(I2C1, ENABLE);
} 

void enable_i2c2()
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	/* SCL I2Cx_SCL_PIN */  /* SDA I2Cx_SDA_PIN*/
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_Cmd(I2C2, ENABLE);
	I2C_Init(I2C2, &I2C_InitStruct);

}

void i2c1_reinit()
{
	EEPROM_DEBUG_PRINT("[I2C reinitialize]\n\r");

	I2C_DeInit(I2C1);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);

	/* Prepare I2C's GPIO configuration */
	GPIO_InitTypeDef GPIO_InitStruct = {
		.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_PuPd = GPIO_PuPd_NOPULL
	};
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/* Prepare GPIO configuration to operate the EEPROM address pin */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = EEPROM_A2_PIN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(EEPROM_A2_PIN_GROUP, &GPIO_InitStruct);
	GPIO_SetBits(EEPROM_A2_PIN_GROUP, EEPROM_A2_PIN);

	I2C_InitTypeDef I2C_InitStruct = {
		.I2C_ClockSpeed = 400000,
		.I2C_Mode = I2C_Mode_I2C,
		.I2C_DutyCycle = I2C_DutyCycle_2,
		.I2C_OwnAddress1 = 0x0A,
		.I2C_Ack = I2C_Ack_Enable,
		.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit
	};
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_AcknowledgeConfig(I2C1,ENABLE);
	I2C_Cmd(I2C1, ENABLE);
}

void i2c_Init()
{
	enable_i2c1();
	enable_i2c2();
}
