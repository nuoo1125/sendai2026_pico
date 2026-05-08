#ifndef SERVO_H
#define SERVO_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class SERVO {
private:
    uint pin_in1;
    float current_angle = 0.0f;
public:
    SERVO(int in1);
    float value();
    void run(float angle);              
};

#endif