#pragma once

#include "pico/stdlib.h"
#include "../config.h"
#ifdef __cplusplus
extern "C" {
#endif
void buzzer();
void ssd1306_send_cmd(uint8_t cmd);
void ssd1306_init();
void oled_clear();
void oled_putchar(char c);
void oled_print(const char *str, uint8_t page, uint8_t col);
#ifdef __cplusplus
}
#endif