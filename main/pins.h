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

// ===== 1.8寸 TFT 液晶屏（ST7735S，SPI 串口 7Pin）=====
// 引脚：RST / D/C / SDI(MOSI) / SCK / CS
#define LCD_RST GPIO_NUM_21
#define LCD_DC  GPIO_NUM_15
#define LCD_SDI GPIO_NUM_16
#define LCD_SCK GPIO_NUM_13
#define LCD_CS  GPIO_NUM_14

#endif // PINS_H
