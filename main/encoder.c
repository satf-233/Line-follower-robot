#include "encoder.h"
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// ================================================================
// 霍尔编码器测速模块
//
//   每台电机有两路霍尔信号 A / B（相位相差 90°，即正交编码）：
//     - A 相：每个上升/下降沿计一个脉冲，用于测速
//     - B 相：边沿时刻的电平决定旋转方向（正转 / 反转）
//   由 esp_timer 定时器周期性结算转速（RPM），存入 s_rpm 供外部读取。
//
//   ⚠ 接线前注意：
//     1. pins.h 中 E1A/E1B/E2A/E2B/E4A/E4B 目前为 GPIO_NUM_x 占位符，
//        填上真实引脚后才能编译使用；选择引脚时避开 pins.h 已占用的脚。
//     2. ESP32-S3 的 GPIO46 只能输出、不能输入，不能用作编码器输入。
//     3. 若读到的转速正负与转向相反，交换该电机 A/B 两根线即可。
// ================================================================

// 输出轴每转一圈，A 相产生的边沿数。
// 需按实际电机修改：霍尔盘每转脉冲数 × 减速比 × 2（双边沿计数）。
#define ENCODER_PPR          205    // 示例：霍尔盘 10 脉冲/转 × 减速比 1:1
#define ENCODER_UPDATE_MS    100   // 转速更新周期（ms），RPM 每 100ms 刷新一次

typedef struct {
    gpio_num_t pin_a;
    gpio_num_t pin_b;
} encoder_channel_t;

// 与 pins.h 中 E1/E2/E4 对应
static const encoder_channel_t s_channels[MOTOR_MAX] = {
    [MOTOR_A] = { E1A, E1B },
    [MOTOR_B] = { E2A, E2B },
    [MOTOR_D] = { E4A, E4B },
};

// 累计脉冲数（带方向）。ISR 中写、定时器/主循环中读，访问需临界区保护
static volatile int32_t s_count[MOTOR_MAX];
static int32_t s_last_count[MOTOR_MAX];   // 上次结算时的计数，用于算差值
static float s_rpm[MOTOR_MAX];
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

// ---- A 相边沿中断：计数 + 判向 ----
static void encoder_isr(void *arg)
{
    uint32_t motor = (uint32_t)arg;
    int a = gpio_get_level(s_channels[motor].pin_a);
    int b = gpio_get_level(s_channels[motor].pin_b);
    int dir = (a ^ b) ? 1 : -1; // 正交解码：A 与 B 异相为正转，同相为反转

    portENTER_CRITICAL_ISR(&s_mux);
    s_count[motor] += dir;
    portEXIT_CRITICAL_ISR(&s_mux);
}

// ---- 定时器回调：按周期差值计算 RPM ----
static void update_speed(void *arg)
{
    for (uint32_t motor = 0; motor < MOTOR_MAX; motor++)
    {
        portENTER_CRITICAL(&s_mux);
        int32_t count = s_count[motor];
        portEXIT_CRITICAL(&s_mux);

        int32_t delta = count - s_last_count[motor];
        s_last_count[motor] = count;
        // 脉冲差 / 每转脉冲数 = 转数，再换算成"每分钟多少转"
        s_rpm[motor] = (float)delta / ENCODER_PPR * (60.0f * 1000.0f / ENCODER_UPDATE_MS);
    }
}

void encoder_init(void)
{
    // 6 根霍尔线全部配置为输入（霍尔模块多为开漏输出，需要上拉）
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << E1A) | (1ULL << E1B)
                      | (1ULL << E2A) | (1ULL << E2B)
                      | (1ULL << E4A) | (1ULL << E4B),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_ANYEDGE, // A 相上升/下降沿都计数
    };
    gpio_config(&io);

    // 安装 GPIO 中断服务（不加 ESP_INTR_FLAG_IRAM，中断里可以直接调用 gpio_get_level）
    static bool isr_installed = false;
    if (!isr_installed)
    {
        gpio_install_isr_service(0);
        isr_installed = true;
    }

    // 每台电机只在 A 相注册中断，B 相只做判向
    for (uint32_t motor = 0; motor < MOTOR_MAX; motor++)
    {
        gpio_isr_handler_add(s_channels[motor].pin_a, encoder_isr, (void *)motor);
        s_count[motor] = 0;
        s_last_count[motor] = 0;
        s_rpm[motor] = 0.0f;
    }

    // 周期结算转速
    const esp_timer_create_args_t timer_args = {
        .callback = update_speed,
        .name = "encoder_speed",
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, ENCODER_UPDATE_MS * 1000));
}

float encoder_get_rpm(motor_id_t motor)
{
    return s_rpm[motor];
}

// 获取并显示脉冲数量
int32_t encoder_get_pulses(motor_id_t motor)
{
    int32_t count;
    portENTER_CRITICAL(&s_mux);
    count = s_count[motor];
    portEXIT_CRITICAL(&s_mux);
    return count;
}

void encoder_reset(motor_id_t motor)
{
    portENTER_CRITICAL(&s_mux);
    s_count[motor] = 0;
    portEXIT_CRITICAL(&s_mux);
    s_last_count[motor] = 0;
}
