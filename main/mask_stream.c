/**
 * mask_stream.c - 后台取图 + 非阻塞取帧实现
 *
 * 单生产者 / 单消费者模型：
 *   - 生产者后台任务：get_mask_pro -> 可选 wifi_stream_push_bw -> 发布最新帧
 *   - 消费者主循环：mask_stream_get_latest（非阻塞）/ mask_stream_wait_new（等新帧）
 *
 * 内存说明：
 *   - 生产者自持的 frame 由 get_mask_pro 内部 malloc（内部 RAM）
 *   - 发布给消费者的 s_latest 放在 PSRAM（heap_caps_malloc），避免占用内部 SRAM
 *   - 消费者拿到的 out 为内部 RAM（malloc），与 free_bw_image 兼容
 */
#include "mask_stream.h"
#include "WIFI.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include <string.h>
#include <stdlib.h>

static const char *TAG = "MASK_STREAM";

/* 取图起始行：与 ball.c 里的 get_mask_pro(mask, 143, 0, mode) 保持一致 */
#define MASK_ROW_START  143

/* 摄像头 USB 枚举 + 首帧稳定需要的预热时间(ms)。
 * 若在枚举期就抓帧，可能拿到损坏的首帧让 JPEG 解码器长时间占 CPU，
 * 进而饿死 IDLE 任务触发任务看门狗（日志里表现为「等待相机，-4」后小车卡死）。 */
#define CAM_WARMUP_MS   2000

/* 每处理完一帧后主动阻塞的时长(ms)。
 * 解码 ~85ms > 帧间隔 66.7ms，生产者会因二值信号量一直“满”而永不阻塞，
 * 持续解码把同核 idle 饿死。这里强制让出，保证 idle 能喂狗。
 * 注意：CONFIG_FREERTOS_HZ=100 时 1 tick=10ms，pdMS_TO_TICKS(1)=0 会退化成
 * taskYIELD（只让同级，不让 idle），所以本值必须 >=10 才能真正阻塞。 */
#define MASK_DECODE_YIELD_MS   10

static volatile int s_mode      = REDBALL_MODE;  // 取图模式，set_mode 可运行时切换
static bool         s_push_wifi = false;         // 是否同时推送到 /mask

static TaskHandle_t      s_task      = NULL;
static SemaphoreHandle_t s_data_lock = NULL;   // 保护 s_latest
static SemaphoreHandle_t s_frame_sem = NULL;   // 每成功出一帧给一次，唤醒 wait_new

static BWImage s_latest = {0};   // 最新已发布帧（PSRAM）
static size_t  s_latest_cap = 0; // s_latest.data 当前容量（字节），复用缓冲避免每帧 malloc/free

/* 把 src 深拷贝到 s_latest（PSRAM），复用已有缓冲避免每帧 malloc/free；调用前需持有 s_data_lock */
static void publish_latest(const BWImage *src)
{
    s_latest.width    = src->width;
    s_latest.height   = src->height;
    s_latest.channels = src->channels;

    size_t n = (size_t)src->width * src->height * src->channels;
    if (n == 0 || !src->data) {
        return;   // 无效帧，保留旧缓冲不动
    }

    if (n > s_latest_cap) {
        // 只有尺寸变大才重新分配；尺寸稳定后每帧只 memcpy，不再 malloc/free
        if (s_latest.data) {
            heap_caps_free(s_latest.data);
        }
        s_latest.data = (unsigned char *)heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
        s_latest_cap  = s_latest.data ? n : 0;
    }

    if (s_latest.data) {
        memcpy(s_latest.data, src->data, n);
    }
}

/* 把 s_latest 深拷贝到 out（内部 RAM，free_bw_image 可释放），调用前需持有 s_data_lock */
static bool copy_latest_to(BWImage *out)
{
    if (out->data) {
        free(out->data);
        out->data = NULL;
    }

    size_t n = (size_t)s_latest.width * s_latest.height * s_latest.channels;
    if (n == 0 || !s_latest.data) {
        out->width = out->height = out->channels = 0;
        return false;
    }

    out->width    = s_latest.width;
    out->height   = s_latest.height;
    out->channels = s_latest.channels;
    out->data = (unsigned char *)malloc(n);
    if (out->data) {
        memcpy(out->data, s_latest.data, n);
        return true;
    }
    return false;
}

static void producer_task(void *arg)
{
    (void)arg;
    BWImage frame = {0};

    // 等摄像头完成 USB 枚举并稳定出流，避免抓到损坏首帧（见 CAM_WARMUP_MS 注释）
    vTaskDelay(pdMS_TO_TICKS(CAM_WARMUP_MS));

    while (1) {
        if (!get_mask_pro(&frame, MASK_ROW_START, 0, s_mode)) {
            // 取图失败：小睡后重试，避免忙等
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (s_push_wifi) {
            wifi_stream_push_bw(&frame);
        }

        if (xSemaphoreTake(s_data_lock, portMAX_DELAY) == pdTRUE) {
            publish_latest(&frame);
            xSemaphoreGive(s_data_lock);
        }
        xSemaphoreGive(s_frame_sem);   // 通知消费者有新帧

        // 解码是 CPU 大户且快于出帧节奏时生产者会永不阻塞（见 MASK_DECODE_YIELD_MS 注释），
        // 每帧主动阻塞一小段，让同核 idle 任务有机会运行喂狗，避免饿死触发看门狗。
        vTaskDelay(pdMS_TO_TICKS(MASK_DECODE_YIELD_MS));
    }
}

bool mask_stream_start(int mode, bool push_wifi)
{
    if (s_task) {
        return true;   // 已启动
    }

    s_mode      = mode;
    s_push_wifi = push_wifi;

    s_data_lock = xSemaphoreCreateMutex();
    s_frame_sem = xSemaphoreCreateBinary();
    if (!s_data_lock || !s_frame_sem) {
        if (s_data_lock) { vSemaphoreDelete(s_data_lock); s_data_lock = NULL; }
        if (s_frame_sem) { vSemaphoreDelete(s_frame_sem); s_frame_sem = NULL; }
        return false;
    }

    // 优先级 1（与 app_main 同级）+ 钉死在核 1（APP_CPU）：
    // JPEG 解码是 CPU 大户，放核 1 与核 0 上的 main、USB sample_proc 分摊负载。
    // 否则核 0 被解码+球识别+USB 采样挤满到 100%，IDLE0 饿死，看门狗每 5 秒报一次。
    if (xTaskCreatePinnedToCore(producer_task, "mask_prod", 8192, NULL, 1, &s_task, 1) != pdPASS) {
        vSemaphoreDelete(s_data_lock); s_data_lock = NULL;
        vSemaphoreDelete(s_frame_sem); s_frame_sem = NULL;
        return false;
    }

    ESP_LOGI(TAG, "后台取图任务已启动 mode=%d push_wifi=%d", mode, push_wifi);
    return true;
}

void mask_stream_set_mode(int mode)
{
    s_mode = mode;
}

bool mask_stream_get_latest(BWImage *out)
{
    if (!out || !s_data_lock) {
        return false;
    }

    bool ok = false;
    if (xSemaphoreTake(s_data_lock, portMAX_DELAY) == pdTRUE) {
        ok = copy_latest_to(out);
        xSemaphoreGive(s_data_lock);
    }
    return ok;
}

bool mask_stream_wait_new(BWImage *out, uint32_t timeout_ms)
{
    if (!out || !s_frame_sem) {
        return false;
    }

    // 等一帧新帧（超时则失败）；有积压帧时立刻返回最新帧
    if (xSemaphoreTake(s_frame_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return false;
    }
    return mask_stream_get_latest(out);
}

void mask_stream_stop(void)
{
    if (s_task) {
        vTaskDelete(s_task);
        s_task = NULL;
    }

    if (s_data_lock) { vSemaphoreDelete(s_data_lock); s_data_lock = NULL; }
    if (s_frame_sem) { vSemaphoreDelete(s_frame_sem); s_frame_sem = NULL; }

    if (s_latest.data) {
        heap_caps_free(s_latest.data);
        s_latest.data = NULL;
    }
    s_latest_cap = 0;
}
