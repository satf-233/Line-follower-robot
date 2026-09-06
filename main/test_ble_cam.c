#include <stdio.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_audio.h"
#include "BLE.h"

static const char *TAG = "TEST_BLE_CAM";

#define STREAM_DELAY_MS 20    /* 两帧之间的间隔，约 15~30fps */

/* 发送哪种滤波结果：LINE_MODE 黑线 / REDBALL_MODE 红球 / BLUEBALL_MODE 蓝球 */
#define FILTER_MODE     BLUEBALL_MODE

#if 0  /* 使用 test_ble_cam.c 作为入口时，此处注释掉，避免重复 app_main */

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

    /* 6. 主循环：取滤波后的二值图 -> 转 RGB 可视化 -> 编码 JPEG -> BLE 分片发送 */
    while (1) {
        /* 已连接但还没订阅通知（电脑没跑接收脚本）时，先等待 */
        if (!ble_is_ready()) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        int64_t t0 = esp_timer_get_time();

        /* get_mask 内部已完成：抓帧 + 阈值过滤 + 开闭运算 + 连通域滤噪 */
        BWImage mask = {0};
        if (!get_mask(&mask, FILTER_MODE)) {
            ESP_LOGW(TAG, "取图失败，重试");
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        int64_t t1 = esp_timer_get_time();

        /* 二值图 -> RGB 可视化（黑线=白，场地=黑），便于电脑端看清滤波效果。
         * 注意：mask.data 是单通道二值图原始字节，电脑端按 JPEG 解码，
         *       不能直接发送，必须先编码成 JPEG。 */
        RGBImage vis = {0};
        if (bw_to_rgb(&mask, &vis) != CAM_OK) {
            free_bw_image(&mask);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* RGB -> JPEG（MJPEG） */
        unsigned char *jpg = NULL;
        size_t jpg_size = 0;
        if (rgb_to_jpeg(&vis, &jpg, &jpg_size) == CAM_OK) {
            int64_t t2 = esp_timer_get_time();
            if (ble_send_frame(jpg, jpg_size) != BLE_OK) {
                ESP_LOGW(TAG, "发送失败（未连接或未订阅）");
            }
            free(jpg);
            int64_t t3 = esp_timer_get_time();
            ESP_LOGI(TAG, "计时: 抓取+解码+裁剪+滤波=%lld ms, 可视化+编码=%lld ms, 发送=%lld ms, 合计=%lld ms",
                     (long long)(t1 - t0) / 1000,
                     (long long)(t2 - t1) / 1000,
                     (long long)(t3 - t2) / 1000,
                     (long long)(t3 - t0) / 1000);
        } else {
            ESP_LOGW(TAG, "JPEG 编码失败");
        }

        free_rgb_image(&vis);
        free_bw_image(&mask);

        vTaskDelay(pdMS_TO_TICKS(STREAM_DELAY_MS));
    }

    /* 正常情况下不会走到这里 */
    ble_deinit();
    cam_stop_stream();
    cam_cleanup();
    ESP_LOGI(TAG, "测试结束");
}
#endif
