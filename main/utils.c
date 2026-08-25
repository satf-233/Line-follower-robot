#include "utils.h"
#include "driver/ledc.h"
#include <stdbool.h>

#define PWM_FREQ_HZ     10000              // PWM 频率（Hz），可调
#define PWM_DUTY_RES    LEDC_TIMER_10_BIT  // 分辨率：10 位，占空比 0~1023
#define PWM_MAX_DUTY    ((1U << PWM_DUTY_RES) - 1)

// 记录每个 LEDC 通道当前绑定的 GPIO，-1 表示该通道空闲
static int s_channel_gpio[LEDC_CHANNEL_MAX];
static bool s_pwm_ready = false;

// 首次使用时：初始化共享定时器 + 通道映射表
static void pwm_ensure_ready(void)
{
    if (s_pwm_ready) {
        return;
    }
    for (int i = 0; i < LEDC_CHANNEL_MAX; i++) {
        s_channel_gpio[i] = -1;
    }

    // 所有 PWM 引脚共用一个定时器（频率、分辨率相同）
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_DUTY_RES,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    s_pwm_ready = true;
}

void pwm_set_duty(gpio_num_t gpio_num, float duty)
{
    pwm_ensure_ready();

    // 占空比限幅到 [0, 1]
    if (duty < 0.0f) {
        duty = 0.0f;
    } else if (duty > 1.0f) {
        duty = 1.0f;
    }

    // 查找该 GPIO 是否已绑定过通道
    ledc_channel_t ch = LEDC_CHANNEL_MAX;
    for (int i = 0; i < LEDC_CHANNEL_MAX; i++) {
        if (s_channel_gpio[i] == (int)gpio_num) {
            ch = (ledc_channel_t)i;
            break;
        }
    }

    // 未绑定则分配一个空闲通道并绑定该 GPIO
    if (ch == LEDC_CHANNEL_MAX) {
        for (int i = 0; i < LEDC_CHANNEL_MAX; i++) {
            if (s_channel_gpio[i] == -1) {
                s_channel_gpio[i] = (int)gpio_num;
                ch = (ledc_channel_t)i;

                ledc_channel_config_t chan = {
                    .speed_mode = LEDC_LOW_SPEED_MODE,
                    .channel    = ch,
                    .timer_sel  = LEDC_TIMER_0,
                    .intr_type  = LEDC_INTR_DISABLE,
                    .gpio_num   = gpio_num,
                    .duty       = 0,
                    .hpoint     = 0,
                };
                ledc_channel_config(&chan);
                break;
            }
        }
    }

    // 通道用尽（超过 LEDC_CHANNEL_MAX 个 PWM 引脚）则忽略
    if (ch == LEDC_CHANNEL_MAX) {
        return;
    }

    // 换算占空比并输出
    uint32_t val = (uint32_t)(duty * (float)PWM_MAX_DUTY + 0.5f);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, val);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}
