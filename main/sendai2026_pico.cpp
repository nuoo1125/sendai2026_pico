#include <stdio.h>
    #include <stdlib.h>
    #include "pico/stdlib.h"
    #include "hardware/spi.h"
    #include "hardware/uart.h"
    #include "ws2812.pio.h"
    #include "hardware/adc.h"
    #include "config.h"

    #include "gyro/gyro.h"
    #include "servo/servo.h"
    #include "interface/interface.h"
    #include "motor/motor.h"
    #include "mcp3208/mcp3208.h"
    #include "VL53L0X/VL53L0X.h"
#define motor_l 1
#define motor_r 2
#define motor_arm 3
#define wheel_mode 1
#define angle_mode 0
int shiki = 0;//白いほうが高い
int photo_data[5];
int base_speed = 1000;
int phase = 0;
const int xshut_pins[4] = {xshut1,xshut2,xshut3,xshut4};
const int new_addr[4] = {0x30,0x31,0x32,0x33};
SERVO servo_left(servo1);
SERVO servo_right(servo2);
SERVO servo_rotation(servo3);
SERVO servo_elbow1(servo4);
SERVO servo_elbow2(servo5);
SERVO servo_red(servo6);
SERVO servo_yellow(servo7);
SERVO servo_blue(servo8);
VL53L0X sensors[4] ={
    VL53L0X(i2c0,VL53L0X_DEFAULT_ADDRESS),
    VL53L0X(i2c0,VL53L0X_DEFAULT_ADDRESS),
    VL53L0X(i2c0,VL53L0X_DEFAULT_ADDRESS),
    VL53L0X(i2c0,VL53L0X_DEFAULT_ADDRESS)
};

void init_tof(){
    i2c_init(i2c0, 400000);
    gpio_set_function(tof_sda, GPIO_FUNC_I2C);
    gpio_set_function(tof_scl, GPIO_FUNC_I2C);
    gpio_pull_up(tof_sda);
    gpio_pull_up(tof_scl);
    for(int i=0;i<4;i++){
        gpio_init(xshut_pins[i]);
        gpio_set_dir(xshut_pins[i], GPIO_OUT);
        gpio_put(xshut_pins[i], 0);
    }
    sleep_ms(10);
    for(int i=0;i<4;i++){
        gpio_put(xshut_pins[i], 1);
        sleep_ms(10);
        sensors[i].setAddress(new_addr[i]);
        if (!sensors[i].init()) {
            printf("Sensor %d failed to initialize!\n", i);
        }
        sensors[i].setTimeout(500);
        sensors[i].startContinuous();
    }
}
void linetrace(){
    motor_mode(motor_l,wheel_mode);
    motor_mode(motor_r,wheel_mode);
    for(int i=0;i<5;i++){
        photo_data[i] = mcp3208_read(i);
        printf("photo_data[%d]: %d\n", i, photo_data[i]);
    }
    if(photo_data[1] <= shiki && photo_data[3] <= shiki){
        motor_run(motor_l, base_speed);
        motor_run(motor_r, base_speed);
        buzzer();
    }
    else if(photo_data[1] <= shiki){
        motor_run(motor_l, -base_speed);
        motor_run(motor_r, base_speed);
        printf("Turning left\n");
    }
    else if(photo_data[3] <= shiki){
        motor_run(motor_l, base_speed);
        motor_run(motor_r, -base_speed);
        printf("Turning right\n");
    }
    else{
        motor_run(motor_l, base_speed);
        motor_run(motor_r, base_speed);
        printf("Moving forward\n");
    }
}
void display(){
    char buffer[32];
    oled_clear();
    snprintf(buffer, sizeof(buffer), "L:%.1f", servo_left.value());
    oled_print(buffer,1,0);
    snprintf(buffer, sizeof(buffer), "R:%.1f", servo_right.value());
    oled_print(buffer,2,0);
    snprintf(buffer, sizeof(buffer), "Rot:%.1f", servo_rotation.value());
    oled_print(buffer,3,0);
    snprintf(buffer, sizeof(buffer), "Elbow1:%.1f",servo_elbow1.value());
    oled_print(buffer,4,0);
    snprintf(buffer, sizeof(buffer), "Elbow2:%.1f", servo_elbow2.value());
    oled_print(buffer,5,0);
    snprintf(buffer, sizeof(buffer), "Arm:%.1f", get_motor_angle(motor_arm));
    oled_print(buffer,6,0);
}
int main() {
    stdio_init_all();
    sleep_ms(2000);     
    motor_setup(motor_l);
    motor_setup(motor_r);
    motor_setup(motor_arm);
    motor_mode(motor_arm, angle_mode);
    init_bno055();
    ws2812_program_init(ws2812_pin, 800000, IS_RGBW);
    mcp3208_init();
    init_tof();
    ssd1306_init();
    oled_clear();
    oled_print("hello world",1,0);
    blue_led();//初期化ok
    while(phase <= 4){
        display();
        if(phase == 2){
            motor_lock(motor_l);
            motor_lock(motor_r);
            //throw free ball
            motor_unlock(motor_l);
            motor_unlock(motor_r);
            linetrace();
        }
        linetrace();
    }
    // Stop the motors
    //camera
    sleep_ms(100);
    return 0;
}