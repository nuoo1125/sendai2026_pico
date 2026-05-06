#pragma once

#include "pico/stdlib.h"
#include "../config.h"
#include "hardware/spi.h"
#ifdef __cplusplus
extern "C" {
#endif
void mcp3208_init();
uint16_t mcp3208_read(uint8_t ch);
#ifdef __cplusplus
}
#endif