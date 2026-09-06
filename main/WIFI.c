/**
 * WIFI.c - ESP32-S3 SoftAP + HTTP MJPEG 推流实现
 *
 * 核心流程:
 *   1. wifi_ap_init()  启动 AP (默认 SSID=ESP32S3-CAM，密码见 WIFI.h)
 *   2. wifi_http_start() 注册 /、/stream、/snapshot
 *   3. 主循环抓帧后 wifi_stream_push_frame() 推流；
 *      /stream 处理器把帧以 multipart/x-mixed-replace 分块发给每个客户端
 */

#include "WIFI.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

#include "nvs_flash.h"

static const char *TAG = "WIFI";

/* multipart 边界，需与 /stream 返回的 Content-Type 一致 */
#define STREAM_BOUNDARY     "frame"
#define STREAM_PART_HDR     \
    "\r\n--" STREAM_BOUNDARY "\r\n" \
    "Content-Type: image/jpeg\r\n" \
    "Content-Length: %u\r\n\r\n"

/* /stream 处理器轮询新帧的间隔；摄像头约 15fps，10ms 轮询足够且不空转 */
#define STREAM_POLL_MS      10

/* ==================== 状态 ==================== */

/* 最新一帧的共享缓冲：主循环 push 写入，/stream 客户端读取 */
static uint8_t     *s_frame_buf = NULL;
static size_t       s_frame_size = 0;
static uint32_t     s_frame_seq = 0;         /* 每推一帧 +1，客户端据此判断是否有新帧 */
static SemaphoreHandle_t s_frame_lock = NULL;

/* 最新二值/灰度图 (BWImage) 的共享缓冲：主循环 push 写入，/mask 客户端读取 */
static uint8_t     *s_bw_buf = NULL;
static size_t       s_bw_size = 0;
static int          s_bw_w = 0;
static int          s_bw_h = 0;
static uint32_t     s_bw_seq = 0;

/* HTTP 服务器句柄 */
static httpd_handle_t s_httpd = NULL;

/* 当前 /stream 客户端数量（stream 处理器进入/退出时增减） */
static volatile int s_stream_clients = 0;

static bool s_wifi_started = false;

/* ==================== 内部: 帧缓冲 ==================== */

/* 拷贝一帧到调用者提供的缓冲（在锁内完成，返回新 seq；无帧返回 0） */
static uint32_t frame_lock_get(uint8_t **out_data, size_t *out_size)//帧读取
{
    uint32_t seq = 0;
    if (xSemaphoreTake(s_frame_lock, portMAX_DELAY) == pdTRUE) {
        if (s_frame_buf && s_frame_size > 0) {
            *out_data = heap_caps_malloc(s_frame_size, MALLOC_CAP_SPIRAM);
            if (*out_data) {
                memcpy(*out_data, s_frame_buf, s_frame_size);
                *out_size = s_frame_size;
                seq = s_frame_seq;
            }
        }
        xSemaphoreGive(s_frame_lock);
    }
    return seq;
}

/* ==================== 内部: HTTP 处理器 ==================== */

/* GET / : 返回内嵌 <img src="/stream"> 的网页 */
static esp_err_t index_handler(httpd_req_t *req)
{
    static const char html[] =
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<title>ESP32S3-CAM</title></head><body>"
        "<h1>ESP32S3-CAM MJPEG 推流</h1>"
        "<img src=\"/stream\" style=\"max-width:100%\">"
        "</body></html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, html, strlen(html));
}

/* GET /stream : multipart/x-mixed-replace 连续推流 */
static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = ESP_OK;

    res = httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=" STREAM_BOUNDARY);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    s_stream_clients++;
    ESP_LOGI(TAG, "/stream 客户端接入，当前 %d 个", s_stream_clients);

    uint32_t last_seq = 0;

    while (1) {
        uint8_t *data = NULL;
        size_t size = 0;
        uint32_t seq = frame_lock_get(&data, &size);

        if (seq != 0 && seq != last_seq) {//ccx
            last_seq = seq;

            char part_hdr[96];
            int hdr_len = snprintf(part_hdr, sizeof(part_hdr),//ccx
                                   STREAM_PART_HDR, (unsigned)size);

            /* 分块发送：先发 multipart 头，再发 JPEG 数据 */
            if (httpd_resp_send_chunk(req, part_hdr, hdr_len) != ESP_OK) {
                free(data);
                break;
            }
            if (httpd_resp_send_chunk(req, (const char *)data, size) != ESP_OK) {
                free(data);
                break;
            }
            free(data);
        } else {
            /* 尚未收到第一帧，或该帧已被读走，稍等后重试 */
            vTaskDelay(pdMS_TO_TICKS(STREAM_POLL_MS));
        }
    }

    s_stream_clients--;
    ESP_LOGI(TAG, "/stream 客户端断开，当前 %d 个", s_stream_clients);
    return ESP_OK;
}

/* GET /snapshot : 返回最新一帧的单张 JPEG */
static esp_err_t snapshot_handler(httpd_req_t *req)
{
    uint8_t *data = NULL;
    size_t size = 0;
    uint32_t seq = frame_lock_get(&data, &size);

    if (seq == 0) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "no frame yet", 12);
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t res = httpd_resp_send(req, (const char *)data, size);
    free(data);
    return res;
}

/* GET /mask : 返回最新 BWImage，以灰度 JPEG 发送（二值图压缩率高，比 PGM 原始字节小得多） */
static esp_err_t mask_handler(httpd_req_t *req)
{
    uint8_t *data = NULL;
    size_t size = 0;
    int w = 0, h = 0;
    int64_t t0 = esp_timer_get_time();
    int64_t t_copy, t_enc, t_send;

    /* 加锁拷贝出最新掩码，避免读取时被主循环覆盖 */
    if (xSemaphoreTake(s_frame_lock, portMAX_DELAY) == pdTRUE) {
        if (s_bw_buf && s_bw_w > 0 && s_bw_h > 0) {
            size = (size_t)s_bw_w * s_bw_h;
            data = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
            if (data) {
                memcpy(data, s_bw_buf, size);
                w = s_bw_w;
                h = s_bw_h;
            }
        }
        xSemaphoreGive(s_frame_lock);
    }
    t_copy = esp_timer_get_time();

    if (!data) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "no mask yet", 11);
    }

    /* 掩码 -> 灰度 JPEG。二值图压缩率极高（83KB 原始 -> 十几 KB），
     * 大幅降低 WiFi 吞吐压力，让端到端帧率接近生产速率。 */
    BWImage bw = { .data = data, .width = w, .height = h, .channels = 1 };
    unsigned char *jpg = NULL;
    size_t jpg_size = 0;
    int ret = gray_to_jpeg(&bw, &jpg, &jpg_size);
    heap_caps_free(data);   /* 原始掩码用完后立即释放 */
    t_enc = esp_timer_get_time();

    if (ret != CAM_OK || !jpg) {
        httpd_resp_set_status(req, "500 Internal Server Error");
        httpd_resp_set_type(req, "text/plain");
        return httpd_resp_send(req, "encode failed", 13);
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    esp_err_t res = httpd_resp_send(req, (const char *)jpg, jpg_size);
    t_send = esp_timer_get_time();

    ESP_LOGI(TAG, "/mask 计时: 拷贝=%lld ms, 压缩=%lld ms, 发送=%lld ms, JPEG=%zu B",
             (long long)(t_copy - t0) / 1000,
             (long long)(t_enc - t_copy) / 1000,
             (long long)(t_send - t_enc) / 1000,
             jpg_size);

    free(jpg);
    return res;
}

/* ==================== 内部: 注册路由 ==================== */

static esp_err_t register_handlers(void)
{
    static const httpd_uri_t uri_root = {
        .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL
    };
    static const httpd_uri_t uri_stream = {
        .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL
    };
    static const httpd_uri_t uri_snapshot = {
        .uri = "/snapshot", .method = HTTP_GET, .handler = snapshot_handler, .user_ctx = NULL
    };
    static const httpd_uri_t uri_mask = {
        .uri = "/mask", .method = HTTP_GET, .handler = mask_handler, .user_ctx = NULL
    };

    esp_err_t res;
    res = httpd_register_uri_handler(s_httpd, &uri_root);
    if (res != ESP_OK) return res;
    res = httpd_register_uri_handler(s_httpd, &uri_stream);
    if (res != ESP_OK) return res;
    res = httpd_register_uri_handler(s_httpd, &uri_snapshot);
    if (res != ESP_OK) return res;
    res = httpd_register_uri_handler(s_httpd, &uri_mask);
    if (res != ESP_OK) return res;
    return ESP_OK;
}

/* ==================== 内部: Wi-Fi 事件回调 ==================== */

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *evt = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(TAG, "站点接入: " MACSTR, MAC2STR(evt->mac));
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *evt = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(TAG, "站点断开: " MACSTR, MAC2STR(evt->mac));
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_AP_STAIPASSIGNED) {
        ip_event_ap_staipassigned_t *evt = (ip_event_ap_staipassigned_t *)event_data;
        ESP_LOGI(TAG, "分配 IP: " IPSTR, IP2STR(&evt->ip));
    }
}

/* ==================== 公共函数 ==================== */

int wifi_ap_init(void)
{
    esp_err_t ret;

    /* NVS：Wi-Fi 校准数据等需要 */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init 失败: %d", ret);
        return WIFI_ERR;
    }

    /* 网络接口与事件循环 */
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_netif_init 失败: %d", ret);
        return WIFI_ERR;
    }
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "事件循环创建失败: %d", ret);
        return WIFI_ERR;
    }
    esp_netif_create_default_wifi_ap();

    /* Wi-Fi 驱动 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_init 失败: %d", ret);
        return WIFI_ERR;
    }

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED,
                                        &wifi_event_handler, NULL, NULL);

    /* AP 配置 */
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_AP_SSID,
            .ssid_len = strlen(WIFI_AP_SSID),
            .channel = WIFI_AP_CHANNEL,
            .password = WIFI_AP_PASS,
            .max_connection = WIFI_AP_MAX_CONN,
            .authmode = (strlen(WIFI_AP_PASS) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK,
        },
    };

    ret = esp_wifi_set_mode(WIFI_MODE_AP);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_mode 失败: %d", ret);
        return WIFI_ERR;
    }
    ret = esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_config 失败: %d", ret);
        return WIFI_ERR;
    }
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_start 失败: %d", ret);
        return WIFI_ERR;
    }
    s_wifi_started = true;

    ESP_LOGI(TAG, "AP 已启动: SSID=%s, 密码=%s",
             WIFI_AP_SSID, (strlen(WIFI_AP_PASS) == 0) ? "(开放)" : WIFI_AP_PASS);
    return WIFI_OK;
}

int wifi_http_start(void)
{
    esp_err_t ret;

    if (s_httpd) {
        return WIFI_OK;   /* 已启动 */
    }

    if (!s_frame_lock) {
        s_frame_lock = xSemaphoreCreateMutex();
        if (!s_frame_lock) {
            ESP_LOGE(TAG, "创建帧互斥锁失败");
            return WIFI_ERR;
        }
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = WIFI_HTTP_PORT;
    config.max_uri_handlers = 8;
    config.stack_size = 8192;             /* stream 处理器需较大栈 */
    config.lru_purge_enable = true;       /* 断开异常连接时回收，避免资源耗尽 */

    ret = httpd_start(&s_httpd, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败: %d", ret);
        return WIFI_ERR;
    }

    ret = register_handlers();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "注册路由失败: %d", ret);
        httpd_stop(s_httpd);
        s_httpd = NULL;
        return WIFI_ERR;
    }

    char ip[16] = {0};
    wifi_ap_get_ip_str(ip, sizeof(ip));
    ESP_LOGI(TAG, "HTTP 服务器已启动: http://%s/", ip);
    return WIFI_OK;
}

void wifi_http_stop(void)
{
    if (s_httpd) {
        httpd_stop(s_httpd);
        s_httpd = NULL;
        ESP_LOGI(TAG, "HTTP 服务器已停止");
    }
}

int wifi_stream_push_frame(const uint8_t *data, size_t size)
{
    if (!data || size == 0) {
        return WIFI_ERR_PARAM;
    }
    if (!s_frame_lock) {
        return WIFI_ERR;
    }

    if (xSemaphoreTake(s_frame_lock, portMAX_DELAY) != pdTRUE) {
        return WIFI_ERR;
    }

    if (s_frame_buf == NULL || s_frame_size < size) {
        uint8_t *p = heap_caps_realloc(s_frame_buf, size, MALLOC_CAP_SPIRAM);
        if (!p) {
            xSemaphoreGive(s_frame_lock);
            ESP_LOGE(TAG, "帧缓冲扩容失败: %zu 字节", size);
            return WIFI_ERR;
        }
        s_frame_buf = p;
        s_frame_size = size;
    }
    memcpy(s_frame_buf, data, size);//帧写入
    s_frame_seq++;

    xSemaphoreGive(s_frame_lock);
    return WIFI_OK;
}

int wifi_stream_push_bw(const BWImage *bw)
{
    if (!bw || !bw->data || bw->width <= 0 || bw->height <= 0) {
        return WIFI_ERR_PARAM;
    }
    if (!s_frame_lock) {
        return WIFI_ERR;
    }

    size_t size = (size_t)bw->width * bw->height;

    if (xSemaphoreTake(s_frame_lock, portMAX_DELAY) != pdTRUE) {
        return WIFI_ERR;
    }

    /* 掩码放到 PSRAM，避免占内部 SRAM（内部 RAM 已被摄像头/WiFi 占用） */
    if (s_bw_buf == NULL || s_bw_size < size) {
        uint8_t *p = heap_caps_realloc(s_bw_buf, size, MALLOC_CAP_SPIRAM);
        if (!p) {
            xSemaphoreGive(s_frame_lock);
            ESP_LOGE(TAG, "掩码缓冲扩容失败: %zu 字节", size);
            return WIFI_ERR;
        }
        s_bw_buf = p;
        s_bw_size = size;
    }
    memcpy(s_bw_buf, bw->data, size);
    s_bw_w = bw->width;
    s_bw_h = bw->height;
    s_bw_seq++;

    xSemaphoreGive(s_frame_lock);
    return WIFI_OK;
}

int wifi_stream_client_count(void)
{
    return s_stream_clients;
}

void wifi_ap_get_ip_str(char *out, size_t out_len)
{
    if (!out || out_len == 0) {
        return;
    }
    esp_netif_ip_info_t ip;
    esp_netif_t *ap = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap && esp_netif_get_ip_info(ap, &ip) == ESP_OK) {
        snprintf(out, out_len, IPSTR, IP2STR(&(ip.ip)));//在缓冲区out压入数据
    } else {
        snprintf(out, out_len, "192.168.4.1");
    }
}

void wifi_ap_deinit(void)
{
    wifi_http_stop();

    if (s_frame_lock) {
        xSemaphoreTake(s_frame_lock, portMAX_DELAY);
        if (s_frame_buf) {
            heap_caps_free(s_frame_buf);
            s_frame_buf = NULL;
            s_frame_size = 0;
        }
        if (s_bw_buf) {
            heap_caps_free(s_bw_buf);
            s_bw_buf = NULL;
            s_bw_size = 0;
            s_bw_w = 0;
            s_bw_h = 0;
        }
        s_frame_seq = 0;
        s_bw_seq = 0;
        xSemaphoreGive(s_frame_lock);
        vSemaphoreDelete(s_frame_lock);
        s_frame_lock = NULL;
    }

    if (s_wifi_started) {
        esp_wifi_stop();
        esp_wifi_deinit();
        s_wifi_started = false;
    }

    ESP_LOGI(TAG, "Wi-Fi 资源已释放");
}
