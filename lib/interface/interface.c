#include "interface.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "../config.h"

#define i2c_port i2c1
#define oled_addr 0x3C

void buzzer(){
    gpio_init(buzzer_pin);
    gpio_set_dir(buzzer_pin,GPIO_OUT);
    gpio_put(buzzer_pin,true);
    sleep_ms(500);
    gpio_put(buzzer_pin,false);
}
uint8_t init_cmds[] = {
    0x00,          // これ以降はコマンドであることを示す
    0xAE,          // Display OFF
    0xD5, 0x80,    // Set Display Clock Divide Ratio
    0xA8, 0x3F,    // Set Multiplex Ratio (128x64)
    0xD3, 0x00,    // Set Display Offset
    0x40,          // Set Display Start Line
    0x8D, 0x14,    // Charge Pump (重要: これを忘れると表示されない)
    0xAF           // Display ON  
};
void oled_cmd(uint8_t cmd){
    uint8_t buf[2] = {0x00, cmd}; // 0x00はコマンドであることを示す
    i2c_write_blocking(i2c_port, oled_addr, buf, 2, false);
}