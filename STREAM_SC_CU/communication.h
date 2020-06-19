#pragma once

#include <stdint.h>

void COM_UART_RX_Complete_Callback(uint8_t* buf, int size);
int  UARTWrite(const uint8_t* buffer, int size);

int USBRead(uint8_t* buffer, int max_size);
int USBWrite(const uint8_t* buffer, int size);