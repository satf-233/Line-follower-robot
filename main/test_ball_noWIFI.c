#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#include "motor.h"
#include "avoid.h"
#include "image.h"
#include "camera_audio.h"
#include "ball.h"
#include "mask_stream.h"
#include "lcd.h"
#include "start.h"
#include "pins.h"

static const char *TAG = "TEST_BALL_NOWIFI";

#if 1  /* 使用本文件作为入口时置 1，并同时把 test_ball.c 的入口置 0，避免重复 app_main */

void app_main(void)
{
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

    /* 4. 启动后台取图任务（push_wifi=false，不推送 /mask，也不启动 Wi-Fi） */
    if (!mask_stream_start(REDBALL_MODE, false)) {
        ESP_LOGE(TAG, "后台取图任务启动失败");
        cam_stop_stream();
        cam_cleanup();
        return;
    }

    /* 5. 主循环：主程序只取最新掩码，取图由后台任务负责，不再被取图卡住 */
    //约定红球顺时针，蓝球逆时针，小车偏右向停在线终点，先找右侧球，先采用顺时针转动。所以要右侧放红球，左侧放蓝球

    // start();
    // BWImage mask = {0};
    // //找红球并推入洞
    // aim_cam(&mask, REDBALL_MODE);
    // push(&mask);

    // motor_backward(0.2);
    // vTaskDelay(pdMS_TO_TICKS(2000));//后退两秒
    // motor_stop();

    // free_bw_image(&mask);   /* 失败时掩码也可能已分配，统一释放 */
    // /* 正常情况下不会走到这里 */
    // mask_stream_stop();
    // cam_stop_stream();
    // cam_cleanup();
    // ESP_LOGI(TAG, "测试结束");



    start();

    float dir = 114514;
    while (dir > 10 || dir < 0)
    {
        mask_stream_set_mode(LINE_MODE2);
        image_follow();
        dir = avoid_measure_cm();
        lcd_show_dist(dir);
    }
    avoid_run_plus();
    int state = IMAGE_FOLLOW_OK;
    while(state != IMAGE_FOLLOW_STOP){
        lcd_show_dist(avoid_measure_cm());
        mask_stream_set_mode(LINE_MODE2);
        state = image_follow_stop();
    }
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));//后退四秒
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    lcd_init();
    BWImage mask = {0};
    //找红球并推入洞
    aim_cam(&mask, REDBALL_MODE);
    push(&mask);

    motor_backward(0.2);
    vTaskDelay(pdMS_TO_TICKS(2000));//后退四秒

    lcd_show_tri_error(0.0, 1);
    lcd_show_tri_error(0.0, 2);
    lcd_show_tri_error(0.0, 3);

    //找篮球并推入洞
    aim_cam(&mask, BLUEBALL_MODE);
    push(&mask);

    motor_stop();
    free_bw_image(&mask);   /* 失败时掩码也可能已分配，统一释放 */
    /* 正常情况下不会走到这里 */
    mask_stream_stop();
    cam_stop_stream();
    cam_cleanup();
    ESP_LOGI(TAG, "测试结束");
}

#endif
