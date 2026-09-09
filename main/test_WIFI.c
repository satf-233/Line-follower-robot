/**
 * test_WIFI.c - WIFI 二值图 (BWImage) 推流测试入口
 *
 * 只发送滤波后的二值图（掩码），不发送原始 MJPEG，节省计算。
 *
 * 用法:
 *   1. 编译烧录后，ESP32-S3 以 AP 模式启动（SSID=ESP32S3-CAM，密码 12345678）
 *   2. 电脑连上热点后运行: python wifi_receiver_bw.py
 *      （默认拉取 http://192.168.4.1/mask）
 */

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "camera_audio.h"
#include "WIFI.h"


#define STREAM_DELAY_MS 100    /* 两帧之间的间隔（100ms ≈ 10fps） */

#if 0  /* 使用 test_WIFI.c 作为入口时保持 1，其它入口文件请置 0 避免重复 app_main */
#define FILTER_MODE LINE_MODE2
static const char *TAG = "TEST_WIFI";

void app_main(void)
{
    char ip[16] = {0};

    /* 1. 挂载存储区 */
    storage_load();

    /* 2. 初始化摄像头（NULL 使用默认 320x240 @ 15fps，单帧小，适合推流） */
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

    /* 4. 启动 Wi-Fi AP */
    if (wifi_ap_init() != WIFI_OK) {
        ESP_LOGE(TAG, "Wi-Fi AP 初始化失败");
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    /* 5. 启动 HTTP 服务器 */
    if (wifi_http_start() != WIFI_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败");
        wifi_ap_deinit();
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    wifi_ap_get_ip_str(ip, sizeof(ip));
    ESP_LOGI(TAG, "AP IP=%s，电脑端运行 wifi_receiver_bw.py 查看掩码", ip);

    /* 6. 主循环：取滤波后的二值图 -> 推送到 /mask（不再发送原始 MJPEG） */
    while (1) {
        int64_t t0 = esp_timer_get_time();
        int64_t t1;
        int64_t t2;
        BWImage mask = {0};

        /* get_mask 内部自带摄像头连接重试与取图/滤波 */
        if (get_mask_pro(&mask, 143, 0, FILTER_MODE))
        {
            t1 = esp_timer_get_time();
            wifi_stream_push_bw(&mask);
        } 
        else 
        {
            t1 = esp_timer_get_time();
            ESP_LOGW(TAG, "获取掩码失败");
        }
        free_bw_image(&mask);   /* 失败时掩码也可能已分配，统一释放 */

        t2 = esp_timer_get_time();
        ESP_LOGI(TAG, "取图+滤波: %lld ms, 推流: %lld ms, 总计: %lld ms", 
            (long long)(t1 - t0) / 1000, 
            (long long)(t2 - t1) / 1000, 
            (long long)(t2 - t0) / 1000);

        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS));
    }

    /* 正常情况下不会走到这里 */
    wifi_ap_deinit();
    cam_stop_stream();
    cam_cleanup();
    ESP_LOGI(TAG, "测试结束");
}

#endif
