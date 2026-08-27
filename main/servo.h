#ifndef SERVO_H
#define SERVO_H

#include "driver/gpio.h"

// 初始化两个 MG90S 舵机（50Hz PWM），上电后调用一次即可
void servo_init(void);

// 设置指定舵机的角度（0°~180°），超出范围会自动限幅
// gpio_num: SERVO1_GPIO 或 SERVO2_GPIO（见 pins.h）
// SERVO1控制水平，SERVO2控制竖直，具体见小车上的标记
void servo_set_angle(gpio_num_t gpio_num, int angle);

#endif // SERVO_H
