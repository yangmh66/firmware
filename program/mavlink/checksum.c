#include <stdint.h>

#include "checksum.h"

uint8_t checksum_generate(uint8_t *data, int data_size)
{
	uint8_t result = 0;

	int i;
	for(i = 0; i < data_size; i++) {
		result ^= data[i];
	}

	return result;
}
