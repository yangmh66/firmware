#ifndef FILE_AT24C04C_H
#define FILE_AT24C04C_H

void eeprom_read(uint8_t *data, uint16_t eeprom_address, int count);
void eeprom_write(uint8_t *data, uint16_t eeprom_address, int count);

/* EEPROM */
typedef struct {
	void (*write)(uint8_t *data, uint16_t eeprom_address, int count);
	void (*read)(uint8_t *data, uint16_t eeprom_address, int count);
} eeprom_t;

#endif
