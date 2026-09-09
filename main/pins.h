#ifndef PINS_H
#define PINS_H

#include "driver/gpio.h"

// ===== 四路红外传感器IR =====
//看向前进方向，从左往右编号
//#define IR1 GPIO_NUM_19
//#define IR2 GPIO_NUM_20
//#define IR3 GPIO_NUM_21
//#define IR4 GPIO_NUM_41

// ===== LED（WS2812）=====
#define LED_GPIO        GPIO_NUM_38
#define LED_NUM         1

// ===== 按键 =====
#define BOOT_GPIO       GPIO_NUM_0   // BOOT 按键（按下接 GND，低电平）

// ===== 电机驱动（TB6612）=====
#define PWMA  GPIO_NUM_6
#define AIN1  GPIO_NUM_15
#define AIN2  GPIO_NUM_7
#define PWMB  GPIO_NUM_18
#define BIN1  GPIO_NUM_3
#define BIN2  GPIO_NUM_8
#define PWMD  GPIO_NUM_14
#define DIN1  GPIO_NUM_12
#define DIN2  GPIO_NUM_13

// A电机
#define E1A   GPIO_NUM_16
#define E1B   GPIO_NUM_17

// B电机
#define E2A   GPIO_NUM_46
#define E2B   GPIO_NUM_9

// D电机
#define E4A   GPIO_NUM_11
#define E4B   GPIO_NUM_10

// ===== 舵机（MG90S，两个）=====
// servo2控制俯仰角，servo1控制水平旋转
#define SERVO1_GPIO     GPIO_NUM_21
#define SERVO2_GPIO     GPIO_NUM_45

// ===== 液晶屏（ST7735S，SPI 串口 7Pin）=====
#define LCD_RST GPIO_NUM_39
#define LCD_DC  GPIO_NUM_40
#define LCD_SDI GPIO_NUM_2
#define LCD_SCK GPIO_NUM_1

// ===== 超声波测距传感器（HC-SR04）=====
#define TRIG GPIO_NUM_5
#define ECHO GPIO_NUM_4

#endif // PINS_H