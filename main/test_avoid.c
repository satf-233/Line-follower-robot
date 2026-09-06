#include <stdio.h>
#include "motor.h"
#include "lcd.h"
#include "avoid.h"
#include "image.h"
#include "camera_audio.h"
#include "follow_brain.h"
#include "pins.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "TEST_AVOID";
// 等待按一下 BOOT 按键（低电平）后再启动，带简单消抖    
static void wait_boot_press(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BOOT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,        // 上拉：未按下读到 1
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    int stable = 0;
    while (stable < 3)          // 连续 3 次读到低电平（约 30ms）才算按下
    {
        if (gpio_get_level(BOOT_GPIO) == 0)
            stable++;
        else
            stable = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI("BOOT", "Main program boot!");
}

void app_main()
{
    image_init();
    motor_init();
    avoid_init();
    /* 1. 挂载存储区（用于保存抓到的图像） */
    storage_load();

    /* 2. 初始化摄像头（NULL 使用默认 640x480 @ 15fps） */
    if (cam_init(NULL) != CAM_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }

    /* 3. 启动视频流 */
    if (cam_start_stream() != CAM_OK) {
        ESP_LOGE(TAG, "视频流启动失败");
        cam_cleanup();
        return;
    }
    wait_boot_press();

    float dir = 114514;
    while (dir > 10 || dir < 0)
    {
        image_follow();
        dir = avoid_measure_cm();
    }
    avoid_run_plus();
}