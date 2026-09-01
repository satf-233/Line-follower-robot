#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "camera_audio.h"

static const char *TAG = "TEST_CAM";

#define CAPTURE_COUNT   3     /* 抓取并保存的帧数 */
#define CONNECT_RETRY   50    /* 等待摄像头连接的次数（每次 100ms，共约 5s） */

#if 0  /* 使用 test_cam.c 作为入口时，此处注释掉，避免重复 app_main */

void app_main(void) {
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

    /* 打印当前分辨率 */
    int w = 0, h = 0;
    if (cam_get_resolution(&w, &h) == CAM_OK) {
        ESP_LOGI(TAG, "分辨率: %d x %d", w, h);
    }
    BWImage obj = {.width = 1280, .height = 720, .channels = 1};
    for (int i = 0; i < CAPTURE_COUNT; i++) {
        RGBImage rgb;
        int ret = CAM_ERR_TIMEOUT;

        /* 摄像头刚启动需要一点时间连接，超时则重试 */
        for (int t = 0; t < CONNECT_RETRY; t++) {
            ret = cam_capture_rgb(&rgb, 100);
            if (ret == CAM_OK) break;
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (ret != CAM_OK) {
            ESP_LOGE(TAG, "第 %d 帧抓取失败: %d", i + 1, ret);
            continue;
        }

        char name[32];
        snprintf(name, sizeof(name), "/storage/frame_%d.ppm", i + 1);

        if (save_rgb_as_ppm(name, &rgb) == CAM_OK) {
            ESP_LOGI(TAG, "保存成功: %s (%dx%d, %zu 字节)", name, rgb.width, rgb.height, rgb.total_size);
        } else {
            ESP_LOGE(TAG, "保存失败: %s", name);
        }

        free_rgb_image(&rgb);
    }

    cam_stop_stream();
    cam_cleanup();
    ESP_LOGI(TAG, "摄像头测试结束");
}
#endif
