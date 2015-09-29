#ifndef FILE_AT24C04C_H
#define FILE_AT24C04C_H

#include <stdint.h>

//Hardware abstraction
#define EEPROM_I2C_IRQ_HANDLER I2C1_EV_IRQHandler

/* EEPROM device information */
#define EEPROM_DEVICE_BASE_ADDRESS 0xA8
#define EEPROM_WORD_BASE_ADDRESS 0x00

#define EEPROM_PAGE_SIZE 16
#define EEPROM_MAX_SIZE 1024

/* I2C event timeout */
#define I2C_TIMEOUT_COUNT_MAX 10

enum {
	I2C_SUCCESS,
	I2C_BUSY_FAILED,
	I2C_TIMEOUT_FAILED,
	I2C_WRONG_EVENT_FAILED
} I2C_Exit_Status;

enum {
	EEPROM_SUCCESS,
	EEPROM_I2C_FAILED, //Refer to ENUM "I2C_Exit_Status"
	EEPROM_INVALID_ADDRESS,
	EEPROM_BUFFER_OVERFLOW
} EEPROM_Exit_Status;

enum {
	EEPROM_DEVICE_IDLE,
	EEPROM_DEVICE_WRITE,
	EEPROM_DEVICE_READ
} EEPROM_Operating_Type;

enum {
	GENERATE_START_CONDITION, //For both read and write
	SEND_DEVICE_ADDRESS, //For both read and write
	SEND_WORD_ADDRESS, //For both read and write
	GENERATE_START_CONDITION_AGAIN, //For read only
	SEND_DEVICE_ADDRESS_AGAIN, //For read only
	RECEIVE_DATA, //For read only
	RECEIVE_LAST_DATA, //FOr read only
	SEND_DATA //For write only
} EEPROM_Write_Read_Event;

typedef struct {
	int operating_type; //Check ENUM "EEPROM_Operating_Type"
	int state; //Check ENUM "EEPROM_Write_Read_State"
	int device_address;
	int word_address;
	uint8_t *buffer;
	int buffer_count;
	int sent_count;
	int received_count;
	int timeout_counter;
	int exit_status; //Check ENUM "EEPROM_Exit_Status"
} eeprom_device_info_t;

typedef struct {
	int (*write)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	int (*read)(uint8_t *data, uint16_t eeprom_address, uint16_t count);
	void (*clear)(void);
} eeprom_t;

extern eeprom_t eeprom;

void EEPROM_I2C_IRQ_HANDLER(void);

#endif
