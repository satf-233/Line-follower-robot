#ifndef UTILS_H
#define UTILS_H

#include "driver/gpio.h"

// 在指定 GPIO 上输出指定占空比的 PWM 信号
// gpio_num: 输出引脚
// duty: 占空比，0.0（0%）~ 1.0（100%）
void pwm_set_duty(gpio_num_t gpio_num, float duty);

#endif // UTILS_H
