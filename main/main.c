#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

#include "pins.h"
#include "motor.h"

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

    motor_init();

    // 需要执行的程序
    for (int i = 0; i < 2; i++)
    {
        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 0)); // 绿色
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(500));// 单位ms

        ESP_ERROR_CHECK(led_strip_clear(led_strip));
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_forward(0.2);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_stop_drive();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // motor_backward(0.2);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_stop_drive();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // motor_turn_left(0.2);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_stop_turn();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_turn_right(0.2);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_stop_turn();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // motor_forward(0.2);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_turn_left(0.2);
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // motor_stop();
}
