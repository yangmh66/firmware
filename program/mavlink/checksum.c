#include <stdint.h>

#include "global.h"
#include "checksum.h"

uint8_t checksum_generate(uint8_t *data, int data_size)
{
	//Se the init value as the global data count
	uint8_t result = get_global_data_count();

	int i;
	for(i = 0; i < data_size; i++) {
		result ^= data[i];
	}

	return result;
}
