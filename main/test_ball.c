#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "camera_audio.h"
#include "ball.h"
#include "mask_stream.h"
#include "WIFI.h"
#include "lcd.h"
#include "start.h"

static const char *TAG = "TEST_BALL";

#if 0  /* 使用本文件作为入口时置 1，并同时把 test_ball_noWIFI.c 的入口置 0，避免重复 app_main */

void app_main(void)
{
    char ip[16] = {0};

    /* 1. 挂载存储区 */
    storage_load();
    lcd_init();
    motor_init();
    image_init();
    avoid_init();

    /* 2. 初始化摄像头（NULL 使用默认 480*320 @ 15fps） */
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

    int w = 0, h = 0;
    if (cam_get_resolution(&w, &h) == CAM_OK) {
        ESP_LOGI(TAG, "分辨率: %d x %d", w, h);
    }

    /* 4. 启动 Wi-Fi AP（仅此步与 noWIFI 版不同） */
    if (wifi_ap_init() != WIFI_OK) {
        ESP_LOGE(TAG, "Wi-Fi AP 初始化失败");
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    /* 5. 启动 HTTP 服务器（提供 /mask 端点，电脑端可查看掩码） */
    if (wifi_http_start() != WIFI_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败");
        wifi_ap_deinit();
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    wifi_ap_get_ip_str(ip, sizeof(ip));
    ESP_LOGI(TAG, "AP IP=%s，电脑端运行 wifi_receiver_bw.py 查看掩码", ip);

    /* 6. 启动后台取图任务（push_wifi=true 持续推送 /mask；noWIFI 版为 false） */
    if (!mask_stream_start(REDBALL_MODE, true)) {
        ESP_LOGE(TAG, "后台取图任务启动失败");
        wifi_ap_deinit();
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    /* 7. 主循环：与 test_ball_noWIFI 完全一致，主程序只取最新掩码，取图与推流由后台任务负责 */
    start();
    BWImage mask = {0};
    aim(&mask, REDBALL_MODE);
    push(&mask);
    free_bw_image(&mask);   /* 失败时掩码也可能已分配，统一释放 */

    /* 正常情况下不会走到这里 */
    mask_stream_stop();
    wifi_ap_deinit();
    cam_stop_stream();
    cam_cleanup();
    ESP_LOGI(TAG, "测试结束");
}

#endif
