#pragma once

#include <stdint.h>

int Read(uint8_t* buffer, int max_size, int ascii);
int Write(const uint8_t* buffer, int size, int ascii);

void Communication_Set_USB();
void Communication_Set_UART();
int Communication_Get_USB();