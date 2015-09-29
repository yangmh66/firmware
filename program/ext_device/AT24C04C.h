#ifndef FILE_AT24C04C_H
#define FILE_AT24C04C_H

#include <stdint.h>

/* EEPROM */
typedef struct {
	int (*write)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	int (*read)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	void (*clear)(void);
} eeprom_t;

extern eeprom_t eeprom;

enum {
	EEPROM_SUCCESS,
	EEPROM_INVALID_ADDRESS,
	EEPROM_BUFFER_OVERFLOW
} EEPROM_Exit_Status;

#define EEPROM_DMA_TX_CHANNEL DMA_Channel_1
#define EEPROM_DMA_TX_STREAM DMA1_Stream7
#define EEPROM_DMA_RX_CHANNEL DMA_Channel_1
#define EEPROM_DMA_RX_STREAM DMA1_Stream0
#define EEPROM_I2C_DR_ADDR (uint32_t)(&(I2C1->DR))
#define EEPROM_TX_DMA_FLAG_TCIF DMA_FLAG_TCIF7
#define EEPROM_RX_DMA_FLAG_TCIF DMA_FLAG_TCIF0
#endif
