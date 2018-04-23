#pragma once

#include <stdint.h>

#define BROADCAST_ADDRESS	0

int Protocol_Read(uint8_t* data, int max_len);
int Protocol_Write(uint8_t* data, int size);