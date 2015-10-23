#ifndef FILE_AT24C04C_H
#define FILE_AT24C04C_H

#include <stdint.h>

/* Hardware abstraction */
#define EEPROM_I2C_IRQ_HANDLER I2C1_EV_IRQHandler

/* EEPROM device information */
#define EEPROM_DEVICE_BASE_ADDRESS 0xA8
#define EEPROM_WORD_BASE_ADDRESS 0x00

#define EEPROM_PAGE_SIZE 16
#define EEPROM_MAX_SIZE 1024

/* EEPROM I2C DMA configurations */
#define EEPROM_DMA_TX_CHANNEL DMA_Channel_1
#define EEPROM_DMA_TX_STREAM DMA1_Stream7
#define EEPROM_DMA_RX_CHANNEL DMA_Channel_1
#define EEPROM_DMA_RX_STREAM DMA1_Stream0
#define EEPROM_I2C_DR_ADDR (uint32_t)(&(I2C1->DR))
#define EEPROM_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF7
#define EEPROM_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF0

enum {
	EEPROM_I2C_SUCCESS,
	EEPROM_I2C_BUSY_FAILED,
	EEPROM_I2C_RXNE_FAILED,
	EEPROM_I2C_DMA_FAILED,
	EEPROM_I2C_TIMEOUT_FAILED
} EEPROM_Low_Level_Result;

enum {
	EEPROM_SUCCESS,
	EEPROM_INVALID_ADDRESS,
	EEPROM_BUFFER_OVERFLOW,
	EEPROM_FAILED, //Check "EEPROM_Low_Level_Result" for failed reason
} EEPROM_High_Level_Result;

enum {
	EEPROM_DEVICE_IDLE,
	EEPROM_DEVICE_WRITE,
	EEPROM_DEVICE_READ
} EEPROM_Mode;

enum {
	GENERATE_START_CONDITION,
	SEND_DEVICE_ADDRESS,
	SEND_WORD_ADDRESS,
	GENERATE_START_CONDITION_AGAIN,
	SEND_DEVICE_ADDRESS_AGAIN,
	RECEIVE_ONE_BYTE_DATA,
	RECEIVE_N_BYTE_DATA,
	GENERATE_STOP_CONDITION
} EEPROM_I2C_Event;

typedef struct {
	int operating_type; //Check ENUM "EEPROM_Operating_Type"
	int state; //Check ENUM "EEPROM_Write_Read_State"
	int device_address;
	int word_address;
	uint8_t *buffer;
	int buffer_count;
	int sent_count;
	int received_count;
	int exit_status; //Check ENUM "EEPROM_Exit_Status"
} eeprom_device_info_t;

typedef struct {
	int (*write)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	int (*read)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	void (*clear)(void);
} eeprom_t;

extern eeprom_t eeprom;

void EEPROM_I2C_IRQ_HANDLER(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream7_IRQHandler(void);

#endif
