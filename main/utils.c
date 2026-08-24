#include "utils.h"

void pwm_set_duty(gpio_num_t gpio_num, float duty)
{
    // TODO: 使用 LEDC 在 gpio_num 上输出占空比为 duty 的 PWM
    // 现有的占位版本为，不论占空比传入多少，都会输出占空比1，即电机全速
    gpio_set_level(gpio_num, 1);
}
