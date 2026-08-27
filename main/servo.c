#include "servo.h"
#include "pins.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdbool.h>

// ================================================================
// MG90S 舵机控制
//   控制信号：50Hz PWM（周期 20ms），高电平脉宽 0.5ms~2.5ms 对应 0°~180°
//   与 utils.c 的电机 PWM（TIMER_0，10kHz）分开，单独用 TIMER_1
// ================================================================

#define SERVO_FREQ_HZ       50                 // 舵机标准频率
#define SERVO_DUTY_RES      LEDC_TIMER_14_BIT  // 14 位分辨率，角度更精细
#define SERVO_TIMER         LEDC_TIMER_1       // 用 TIMER_1，避开电机 PWM 的 TIMER_0
#define SERVO_SPEED_MODE    LEDC_LOW_SPEED_MODE

// 脉宽范围（us），MG90S 常见参数，可按实际舵机微调
#define SERVO_PULSE_MIN_US  500    // 0°   对应脉宽
#define SERVO_PULSE_MAX_US  2500   // 180° 对应脉宽

// 角度范围。最小角度都是 0°，但两个舵机的最大角度不同：
// 舵机2（SERVO2）因安装空间受限，实际最多只能转到 100°
#define SERVO_ANGLE_MIN     0
#define SERVO_FULL_ANGLE    180   // 满行程角度，用于脉宽换算（不是限幅上限）
#define SERVO1_ANGLE_MAX    180   // 舵机1 最大角度
#define SERVO2_ANGLE_MAX    100   // 舵机2 最大角度（受空间限制）

// 两个舵机使用的 LEDC 通道（utils.c 的电机 PWM 会从 0 开始占用通道，这里用高位通道避开）
#define SERVO1_CHANNEL      LEDC_CHANNEL_6
#define SERVO2_CHANNEL      LEDC_CHANNEL_7

static const char *TAG = "servo";

// 根据 GPIO 找到对应通道，找不到返回 false
static bool servo_get_channel(gpio_num_t gpio_num, ledc_channel_t *ch)
{
    if (gpio_num == SERVO1_GPIO) {
        *ch = SERVO1_CHANNEL;
        return true;
    }
    if (gpio_num == SERVO2_GPIO) {
        *ch = SERVO2_CHANNEL;
        return true;
    }
    return false;
}

void servo_init(void)
{
    // 1. 配置 LEDC 定时器：50Hz，14 位分辨率
    ledc_timer_config_t timer = {
        .speed_mode      = SERVO_SPEED_MODE,
        .timer_num       = SERVO_TIMER,
        .duty_resolution = SERVO_DUTY_RES,
        .freq_hz         = SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    // 2. 配置两个舵机通道
    ledc_channel_config_t ch1 = {
        .speed_mode = SERVO_SPEED_MODE,
        .channel    = SERVO1_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO1_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch1));

    ledc_channel_config_t ch2 = {
        .speed_mode = SERVO_SPEED_MODE,
        .channel    = SERVO2_CHANNEL,
        .timer_sel  = SERVO_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = SERVO2_GPIO,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch2));

    // 3. 初始放到 90°（中间位置）
    servo_set_angle(SERVO1_GPIO, 90);
    servo_set_angle(SERVO2_GPIO, 90);

    // ESP_LOGI(TAG, "舵机初始化完成 (GPIO%d / GPIO%d)", SERVO1_GPIO, SERVO2_GPIO);
}

void servo_set_angle(gpio_num_t gpio_num, int angle)
{
    ledc_channel_t ch;
    if (!servo_get_channel(gpio_num, &ch)) {
        ESP_LOGE(TAG, "未配置的舵机 GPIO: %d", gpio_num);
        return;
    }

    // 每个舵机的最大角度不同：舵机2 因空间限制最大只能到 100°
    int angle_max = (gpio_num == SERVO2_GPIO) ? SERVO2_ANGLE_MAX : SERVO1_ANGLE_MAX;

    // 角度限幅到 [0°, 最大角度]
    if (angle < SERVO_ANGLE_MIN) {
        angle = SERVO_ANGLE_MIN;
    } else if (angle > angle_max) {
        angle = angle_max;
    }

    // 按角度换算脉宽（us）
    uint32_t pulse_us = SERVO_PULSE_MIN_US +
        (uint32_t)((SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) * angle /
                   (SERVO_FULL_ANGLE - SERVO_ANGLE_MIN));

    // 换算成 LEDC duty：duty = 脉宽 / 周期 * 最大duty
    // 周期 = 1 / 50Hz = 20ms = 20000us
    const uint32_t period_us = 1000000U / SERVO_FREQ_HZ;
    const uint32_t max_duty  = (1U << SERVO_DUTY_RES) - 1;
    uint32_t duty = (uint32_t)((uint64_t)pulse_us * max_duty / period_us);

    ledc_set_duty(SERVO_SPEED_MODE, ch, duty);
    ledc_update_duty(SERVO_SPEED_MODE, ch);
}
