#pragma once

#include <stdint.h>

typedef int (*write_func)(const uint8_t*, int);

void Parse(char*, write_func);