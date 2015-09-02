#ifndef FILE_AT24C04C_H
#define FILE_AT24C04C_H

#include <stdint.h>

/* EEPROM device information */
#define EEPROM_DEVICE_BASE_ADDRESS 0xA8
#define EEPROM_WORD_BASE_ADDRESS 0x00

#define EEPROM_PAGE_SIZE 16
#define EEPROM_MAX_SIZE 1024

enum {
	EEPROM_SUCCESS,
	EEPROM_INVALID_ADDRESS,
	EEPROM_BUFFER_OVERFLOW
} EEPROM_Exit_Status;

enum {
	EEPROM_DEVICE_IDLE,
	EEPROM_DEVICE_WRITE,
	EEPROM_DEVICE_READ
} EEPROM_Device_State;

typedef struct {
	int state; //Check ENUM "EEPROM_Device_State"
	int address;
	uint8_t *buffer;
	int buffer_count;
	int exit_status; //Check ENUM "EEPROM_Exit_Status"
} eeprom_device_info_t;

typedef struct {
	int (*write)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	int (*read)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	void (*clear)(void);
} eeprom_t;

extern eeprom_t eeprom;

void I2C1_EV_IRQHandler(void);

#endif
