#ifndef __CHECKSUM_H
#define __CHECKSUM_H

#include <stdint.h>

uint8_t checksum_generate(uint8_t *data, int data_size);
int checksum_test(uint8_t *data, int data_size, uint8_t checksum);

#endif
