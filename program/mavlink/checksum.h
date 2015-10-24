#ifndef __CHECKSUM_H
#define __CHECKSUM_H

#include <stdint.h>

uint8_t checksum_generate(uint8_t *data, int data_size);

#endif
