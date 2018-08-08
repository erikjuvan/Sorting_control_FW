#pragma once

#include <stdint.h>

typedef enum { ASCII, BINARY } CommunicationMode;
typedef enum { UART, USB} CommunicationInterface;

int Read(uint8_t* buffer, int max_size);
int Write(const uint8_t* buffer, int size);