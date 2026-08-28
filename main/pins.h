#ifndef PINS_H
#define PINS_H

#include "driver/gpio.h"

// ===== 四路红外传感器IR =====
//看向前进方向，从左往右编号
#define IR1 GPIO_NUM_9
#define IR2 GPIO_NUM_10
#define IR3 GPIO_NUM_11
#define IR4 GPIO_NUM_12

// ===== LED（WS2812）=====
#define LED_GPIO        GPIO_NUM_38
#define LED_NUM         1

// ===== 按键 =====
#define BOOT_GPIO       GPIO_NUM_0   // BOOT 按键（按下接 GND，低电平）

// ===== 电机驱动（TB6612）=====
#define STBY  GPIO_NUM_4
#define PWMA  GPIO_NUM_5
#define AIN1  GPIO_NUM_7
#define AIN2  GPIO_NUM_6
#define PWMB  GPIO_NUM_17
#define BIN1  GPIO_NUM_8
#define BIN2  GPIO_NUM_18
#define PWMD  GPIO_NUM_40
#define DIN1  GPIO_NUM_42
#define DIN2  GPIO_NUM_41

// A电机
#define E1A   GPIO_NUM_x
#define E1B   GPIO_NUM_x
// B电机
#define E2A   GPIO_NUM_x
#define E2B   GPIO_NUM_x
// D电机
#define E4A   GPIO_NUM_x
#define E4B   GPIO_NUM_x
// ===== 舵机（MG90S，两个）=====
// servo0控制俯仰角，servo1控制水平旋转
#define SERVO1_GPIO     GPIO_NUM_3
#define SERVO2_GPIO     GPIO_NUM_46

// ===== 液晶屏（ST7735S，SPI 串口 7Pin）=====
#define LCD_DC  GPIO_NUM_15
#define LCD_SDI GPIO_NUM_16
#define LCD_SCK GPIO_NUM_13

// ===== 超声波测距传感器（HC-SR04）=====
#define TRIG GPIO_NUM_1
#define ECHO GPIO_NUM_2

#endif // PINS_H
