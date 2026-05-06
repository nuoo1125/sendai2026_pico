#pragma once

#include "pico/stdlib.h"
#include "../config.h"
#include "hardware/uart.h"
#ifdef __cplusplus
extern "C" {
#endif
void motor_setup(int id);
void motor_mode(int id,int mode);
void motor_run(int id,int speed);
void motor_angle(int16_t id,uint8_t angle10);
void motor_lock(uint8_t id);
void motor_unlock(uint8_t id);

#ifdef __cplusplus
}
#endif