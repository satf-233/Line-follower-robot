#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#define LED_GPIO        GPIO_NUM_38
#define LED_NUM         1

// 电机测试用
#define STBY  GPIO_NUM_4
#define PWMA  GPIO_NUM_5
#define AIN1  GPIO_NUM_7
#define AIN2  GPIO_NUM_6
#define PWMB  GPIO_NUM_17
#define BIN1  GPIO_NUM_8
#define BIN2  GPIO_NUM_18

// 电机测试用
static void motor_gpio_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL<<STBY)|(1ULL<<PWMA)|(1ULL<<AIN1)|(1ULL<<AIN2)
                      | (1ULL<<PWMB)|(1ULL<<BIN1)|(1ULL<<BIN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(STBY, 1);   // 使能（不拉高电机不动）
}
// 电机测试用
// dir=1 正转，dir=0 反转（全速）
static void motor_a(int dir)
{
    gpio_set_level(PWMA, 1);
    gpio_set_level(AIN1, dir);
    gpio_set_level(AIN2, !dir);
}
static void motor_b(int dir)
{
    gpio_set_level(PWMB, 1);
    gpio_set_level(BIN1, dir);
    gpio_set_level(BIN2, !dir);
}
static void motor_stop(void)
{
    gpio_set_level(AIN1, 0); gpio_set_level(AIN2, 0);  // 滑行停止
    gpio_set_level(BIN1, 0); gpio_set_level(BIN2, 0);
}



void app_main(void)
{
    led_strip_handle_t led_strip;

    // v3.x API：字段名变了
    // led配置，告诉系统led的结构，以便能被正确调用
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // v3.x 用这个 // r在第1位 g在第0位 b在第2位 
        .flags = {
            .invert_out = false,
        }
    };

    // rmt配置
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        }
    };

    // 创建灯带设备，并生成一个可以控制灯的句柄
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    // 参数1用来传入灯带配置，参数2用来传入rmt配置，参数3用来存放创建好的灯带设备


    // 电机测试用
    motor_gpio_init();

    // 需要执行的程序
    // while (1) {
    //     // led测试用
    //     // ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 0)); // 绿色
    //     // ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    //     // vTaskDelay(pdMS_TO_TICKS(500));// 单位ms

    //     // ESP_ERROR_CHECK(led_strip_clear(led_strip));
    //     // ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    //     // vTaskDelay(pdMS_TO_TICKS(500));
    // }
    // 电机测试用
    motor_a(1);
    // motor_b(1);
    vTaskDelay(pdMS_TO_TICKS(10000));
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_a(0);
    vTaskDelay(pdMS_TO_TICKS(2000));  // A 反转 2s
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));


    vTaskDelay(pdMS_TO_TICKS(2000));  // B 正转 2s
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));
    motor_b(0);
    vTaskDelay(pdMS_TO_TICKS(2000));  // B 反转 2s
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));
}