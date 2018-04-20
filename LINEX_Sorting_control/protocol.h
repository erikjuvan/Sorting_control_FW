#pragma once

#include <stdint.h>

int Protocol_Read(uint8_t* data, int max_len);
int Protocol_Write(uint8_t* data, int size);