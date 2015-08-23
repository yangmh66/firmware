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

void I2C1_EV_IRQHandler(void);

#endif
