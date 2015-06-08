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

int checksum_test(uint8_t *data, int data_size, uint8_t checksum)
{
	uint8_t checksum_verify = 0;

	int i;
	for(i = 0; i < data_size; i++) {
		checksum_verify ^= data[i];
	}

	if(checksum == checksum_verify) return 0;

	return 1;
}
