#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "camera_audio.h"
#include "BLE.h"
#include "esp_heap_caps.h"

static const char *TAG = "TEST_BLE_CAM";

#define CONNECT_RETRY   50    /* 等待摄像头连接的次数（每次 100ms，共约 5s） */
#define STREAM_DELAY_MS 20    /* 两帧之间的间隔，约 15~30fps */

/* 发送前拷贝帧用的独立缓冲（帧缓冲是单缓冲，发送慢时会被下一帧覆盖） */
static uint8_t *s_frame_buf = NULL;
static size_t   s_frame_buf_cap = 0;

#if 1  /* 使用 test_ble_cam.c 作为入口时，此处注释掉，避免重复 app_main */

void app_main(void) {
    /* 1. 挂载存储区 */
    storage_load();

    /* 2. 初始化摄像头（NULL 使用默认 320x240 @ 15fps，单帧小，适合 BLE 推流） */
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

    /* 4. 初始化 BLE 并开始广播 */
    if (ble_init() != BLE_OK) {
        ESP_LOGE(TAG, "BLE 初始化失败");
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    /* 5. 等待电脑连接（0 = 一直等待） */
    ESP_LOGI(TAG, "等待电脑连接...");
    if (ble_wait_connected(0) != BLE_OK) {
        ESP_LOGE(TAG, "等待连接异常");
        ble_deinit();
        cam_stop_stream();
        cam_cleanup();
        return;
    }
    ESP_LOGI(TAG, "已连接，等待电脑订阅通知...");

    /* 6. 主循环：抓一帧 MJPEG -> BLE 分片发送 */
    while (1) {
        /* 已连接但还没订阅通知（电脑没跑接收脚本）时，先等待 */
        if (!ble_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        unsigned char *data = NULL;
        size_t size = 0;

        /* 摄像头刚启动需要一点时间，超时则重试 */
        int ret = CAM_ERR_TIMEOUT;
        for (int t = 0; t < CONNECT_RETRY; t++) {
            ret = cam_capture_frame(&data, &size, 100);
            if (ret == CAM_OK) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (ret != CAM_OK) {
            ESP_LOGE(TAG, "抓帧失败: %d", ret);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* data 指向内部帧缓冲（单缓冲），BLE 发送慢、期间会被下一帧覆盖；
         * 先拷贝到独立 PSRAM 缓冲再发送 */
        if (s_frame_buf_cap < size) {
            uint8_t *p = heap_caps_realloc(s_frame_buf, size, MALLOC_CAP_SPIRAM);
            if (!p) {
                ESP_LOGE(TAG, "帧拷贝缓冲分配失败");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
            s_frame_buf = p;
            s_frame_buf_cap = size;
        }
        memcpy(s_frame_buf, data, size);

        if (ble_send_frame(s_frame_buf, size) != BLE_OK) {
            ESP_LOGW(TAG, "发送失败（未连接或未订阅）");
        }

        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS));
    }

    /* 正常情况下不会走到这里 */
    ble_deinit();
    cam_stop_stream();
    cam_cleanup();
    ESP_LOGI(TAG, "测试结束");
}
#endif