#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "motor.h"
#include "../config.h"
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define _USE_MATH_DEFINES
#include <stdio.h>
#include <math.h>
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define UART0_TX    (0)
#define UART0_RX    (1)
#define UART_ID uart0
#define UART_BAUD 1000000

uint8_t TxData[20];

void motor_setup(int id){
    uart_init(UART_ID, UART_BAUD);
    gpio_set_function(UART0_TX, GPIO_FUNC_UART);
    gpio_set_function(UART0_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID,8,1,UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, false);
    printf("motor set up!\n");
    TxData[0] = 0xFF;//ヘッダー
    TxData[1] = 0xFF;//ヘッダー
    TxData[2] = id;//motorID
    TxData[3] = 4;//length
    TxData[4] = 3;//書き込むよ！
    TxData[5] = 33;//mode設定
    TxData[6] = 1;//wheel mode
    uint8_t checksum = 0;
    for (int i = 2; i < 7; i++) {
        checksum += TxData[i];
    }
    TxData[7] = ~checksum;
    uart_write_blocking(UART_ID,TxData,8);
}
void motor_mode(int id,int mode){//1:wheel mode 0:joint mode
    uint8_t txData[20];
    txData[0] = 0xFF;//ヘッダー
    txData[1] = 0xFF;//ヘッダー
    txData[2] = id;//ID
    txData[3] = 4;//length
    txData[4] = 3;//書き込むよ！
    txData[5] = 33;//mode設定
    txData[6] = mode;//mode
    uint8_t checksum = 0;
    for (int i = 2; i < 7; i++) {
        checksum += txData[i];
    }
    txData[7] = ~checksum;
    uart_write_blocking(UART_ID,txData,8);
}
void motor_run(int id,int speed){
    uint8_t txData[20];
    if(speed < 0){
        speed = -speed | 0x0400;
    }
    txData[0] = 0xFF;//ヘッダー
    txData[1] = 0xFF;//ヘッダー
    txData[2] = id;//ID
    txData[3] = 5;//length
    txData[4] = 3;//書き込むよ！
    txData[5] = 46;//速度を書き込む
    txData[6] = speed & 0xFF;//speedの下位8ビット
    txData[7] = (speed >> 8) & 0x7F;//speedの上位7ビットと回転方向を組み合わせる
    uint8_t checksum = 0;
    for (int i = 2; i < 8; i++) {
        checksum += txData[i];
    }
    txData[8] = ~checksum;
    uart_write_blocking(UART_ID,txData,9);
}
void motor_angle(int16_t id,uint8_t angle10) {
    uint8_t txData[20];
    txData[0] = 0xFF; // Header
    txData[1] = 0xFF; // Header
    txData[2] = id;   // サーボID
    txData[3] = 5;    // 長さ (Flags+Addr+Data2byte)
    txData[4] = 3;    // Flags (0: 常時書き込み)
    txData[5] = 42;   // Address 30: 目標位置

    // 角度データを下位・上位に分ける
    txData[6] = angle10 & 0xFF;        // 低位バイト
    txData[7] = (angle10 >> 8) & 0xFF; // 高位バイト

    // チェックサム計算
    uint8_t checksum = 0;
    for (int i = 2; i < 8; i++) {
        checksum += txData[i];
    }
    txData[8] = ~checksum;
    uart_write_blocking(UART_ID,txData,9);
}
void motor_lock(uint8_t id) {
    uint8_t txData[20];
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = id;
    txData[3] = 4;    // Length: Flags(1) + Addr(1) + Data(2) + Checksum(1)
    txData[4] = 3;
    txData[5] = 40;   // Address: トルク
    txData[6] = 1;    // ON (トルク有効)
    uint8_t checksum = 0;
    for (int i = 2; i < 6; i++) {
        checksum += txData[i];
    }
    txData[7] = ~checksum;
    uart_write_blocking(UART_ID,txData,8);
}
void motor_unlock(uint8_t id) {
    uint8_t txData[8];
    txData[0] = 0xFF; 
    txData[1] = 0xFF;
    txData[2] = id;
    txData[3] = 4;
    txData[4] = 3;
    txData[5] = 40;   // Torque Enable Address
    txData[6] = 0;    // OFF
    uint8_t checksum = 0;
    for (int i = 2; i < 6; i++) {
        checksum += txData[i];
    }
    txData[7] = ~checksum;
    uart_write_blocking(UART_ID, txData, 8);
}