#ifndef PINS_H
#define PINS_H

#include "driver/gpio.h"

// ===== 四路红外传感器IR =====
//看向前进方向，从左往右编号
#define IR1 GPIO_NUM_x
#define IR2 GPIO_NUM_x
#define IR3 GPIO_NUM_x
#define IR4 GPIO_NUM_x

// ===== LED（WS2812）=====
#define LED_GPIO        GPIO_NUM_38
#define LED_NUM         1

// ===== 电机驱动（TB6612）=====
#define STBY  GPIO_NUM_4
#define PWMA  GPIO_NUM_5
#define AIN1  GPIO_NUM_7
#define AIN2  GPIO_NUM_6
#define PWMB  GPIO_NUM_17
#define BIN1  GPIO_NUM_8
#define BIN2  GPIO_NUM_18

// ===== 电机 D（引脚待定，暂时留空）=====
// TODO: 填好后取消注释，例如：
// #define PWMD  GPIO_NUM_XX
// #define DIN1  GPIO_NUM_XX
// #define DIN2  GPIO_NUM_XX

#endif // PINS_H
