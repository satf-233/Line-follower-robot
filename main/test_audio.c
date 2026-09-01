#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "camera_audio.h"

static const char *TAG = "MAIN";

#if 0  /* 使用 test_audio.c 作为入口时，此处注释掉，避免重复 app_main */

void app_main(void) {
    // 1. 挂载存储区（内部 flash 的 storage 分区 → /storage）
    storage_load();

    // 2. 播放 WAV
    ESP_LOGI(TAG, "开始播放: /storage/music.wav");
    audio_play_wav("/storage/music.wav", 30);
}
#endif