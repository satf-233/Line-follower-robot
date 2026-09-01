<<<<<<< HEAD
/**
 * camera_audio_esp32.c - ESP32-S3 USB摄像头驱动实现
 * 基于 espressif/usb_stream 组件
 * 
 * 编译: 使用 ESP-IDF 构建系统，需在 idf_component.yml 中添加依赖
 *   dependencies:
 *     espressif/usb_stream: "^1.5.2"
 */

#include "camera_audio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "usb_stream.h"
#include "esp_vfs_fat.h"
#include <dirent.h>

/* ==================== 日志标签 ==================== */
static const char *TAG = "CAM_AUDIO";

/* ==================== 内部数据结构 ==================== */

/**
 * WAV文件头结构体
 */
typedef struct {
    /* RIFF头 */
    char     chunk_id[4];      /* "RIFF" */
    uint32_t chunk_size;       /* 文件大小 - 8 */
    char     format[4];        /* "WAVE" */
    
    /* fmt块 */
    char     subchunk1_id[4];  /* "fmt " */
    uint32_t subchunk1_size;   /* 16 (PCM格式) */
    uint16_t audio_format;     /* 1 = PCM */
    uint16_t num_channels;     /* 声道数 */
    uint32_t sample_rate;      /* 采样率 */
    uint32_t byte_rate;        /* sample_rate * num_channels * bits_per_sample/8 */
    uint16_t block_align;      /* num_channels * bits_per_sample/8 */
    uint16_t bits_per_sample;  /* 位深 */
    
    /* data块 */
    char     subchunk2_id[4];  /* "data" */
    uint32_t subchunk2_size;   /* 音频数据大小 */
} WAVHeader;

/**
 * 视频结构体
 */
typedef struct {
    bool initialized;
    bool stream_running;
    uint16_t width;
    uint16_t height;
    uint32_t fps;
    
    /* 缓冲区 */
    uint8_t *xfer_buffer_a;
    uint8_t *xfer_buffer_b;
    uint8_t *frame_buffer;
    size_t xfer_buf_size;
    size_t frame_buf_size;
    
    /* 同步 */
    SemaphoreHandle_t frame_sem;
    unsigned char *last_frame_data;
    size_t last_frame_size;
} CameraContext;

static CameraContext g_cam = {0};
static bool g_audio_init = false;
static int  g_spk_volume = 100;   /* 播放时应用的扬声器音量，默认 100 */
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;   /* storage 分区 wear-leveling 句柄 */

/* ==================== 内部函数 ==================== */

/**
 * 存储区挂载，运行后才允许调用存储区文件
 */
void storage_load(void)
{
        // 1. 挂载 FATFS（storage 分区）
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
        "/storage",          // 挂载路径
        "storage",           // 分区名，必须和 partitions.csv 一致
        &mount_config,
        &s_wl_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FATFS 挂载失败 (%s)", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "FATFS 挂载成功");

    // 2. 打印文件列表（调试用）
    DIR *dir = opendir("/storage");
    if (dir == NULL) {
        ESP_LOGE(TAG, "无法打开 /storage 目录");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        // 跳过 . 和 ..
        if (entry->d_name[0] == '.') continue;
        ESP_LOGI(TAG, "文件: %s", entry->d_name);
    }
    closedir(dir);
}

/**
 * 摄像头帧回调函数 - 由 usb_stream 组件调用
 */
static void camera_frame_cb(uvc_frame_t *frame, void *arg) {
    (void)arg;

    if (!frame || frame->data_bytes == 0) return;

    /* 打印实际 MJPEG 帧大小，用于确认 frame_buffer_size 是否够用 */
    
    //ESP_LOGI(TAG, "帧大小: %u 字节", (unsigned)frame->data_bytes);

    /* 保存帧数据指针，供 cam_capture_frame 使用 */
    g_cam.last_frame_data = frame->data;
    g_cam.last_frame_size = frame->data_bytes;

    /* 通知有新的帧到达 */
    if (g_cam.frame_sem) {
        xSemaphoreGive(g_cam.frame_sem);
    }
}

/**
 * 获取默认摄像头配置
 */
static CameraConfig get_default_camera_config(void) {
    CameraConfig cfg = {
        .width = 640,
        .height = 480,
        .fps = 15,                       /* 默认 3fps；640x480 支持 25/15fps，15fps 稳定 */
        .xfer_buffer_size = 128 * 1024,  /* 传输缓冲：480p MJPEG 单帧约 20~40KB，128KB 足够 */
        .frame_buffer_size = 256 * 1024, /* 帧缓冲，需容纳整帧 MJPEG，留余量 */
    };
    return cfg;
}

/**
 * 获取默认音频配置
 */
static AudioConfig get_default_audio_config(void) {
    AudioConfig cfg = {
        .sample_rate = 16000,
        .bit_resolution = 16,
        .channels = 1,
    };
    return cfg;
}

/* ==================== 公共摄像头函数 ==================== */

int cam_init(const CameraConfig *config) {
    CameraConfig cfg;
    esp_err_t ret;
    
    if (g_cam.initialized) {
        cam_cleanup();
    }
    
    /* 使用默认配置或用户配置 */
    if (config == NULL) {
        cfg = get_default_camera_config();
    } else {
        cfg = *config;
    }
    
    /* 分配缓冲区 */
    g_cam.xfer_buf_size = cfg.xfer_buffer_size;
    g_cam.frame_buf_size = cfg.frame_buffer_size;
    
    g_cam.xfer_buffer_a = heap_caps_malloc(g_cam.xfer_buf_size, MALLOC_CAP_DEFAULT);
    g_cam.xfer_buffer_b = heap_caps_malloc(g_cam.xfer_buf_size, MALLOC_CAP_DEFAULT);
    g_cam.frame_buffer = heap_caps_malloc(g_cam.frame_buf_size, MALLOC_CAP_DEFAULT);
    
    if (!g_cam.xfer_buffer_a || !g_cam.xfer_buffer_b || !g_cam.frame_buffer) {
        ESP_LOGE(TAG, "缓冲区分配失败");
        goto cleanup;
    }
    
    /* 创建信号量 */
    g_cam.frame_sem = xSemaphoreCreateBinary();
    if (!g_cam.frame_sem) {
        ESP_LOGE(TAG, "信号量创建失败");
        goto cleanup;
    }
    
    /* 配置 UVC 流 */
    uvc_config_t uvc_config = {
        .frame_width = cfg.width,
        .frame_height = cfg.height,
        .frame_interval = FPS2INTERVAL(cfg.fps),
        .xfer_buffer_size = g_cam.xfer_buf_size,
        .xfer_buffer_a = g_cam.xfer_buffer_a,
        .xfer_buffer_b = g_cam.xfer_buffer_b,
        .frame_buffer_size = g_cam.frame_buf_size,
        .frame_buffer = g_cam.frame_buffer,
        .frame_cb = camera_frame_cb,
        .frame_cb_arg = NULL,
        /* 以下为可选参数，组件会自动探测 */
        .format = UVC_FORMAT_MJPEG,      /* 使用 MJPEG 格式 */
        .xfer_type = UVC_XFER_ISOC,      /* 等时传输，相机需支持 */
        .interface = 0,                  /* 0 表示自动探测 */
        .interface_alt = 0,
        .ep_addr = 0,
        .ep_mps = 0,
    };
    
    ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UVC配置失败: %d", ret);
        goto cleanup;
    }
    
    g_cam.width = cfg.width;
    g_cam.height = cfg.height;
    g_cam.fps = cfg.fps;
    g_cam.initialized = true;
    g_cam.stream_running = false;
    
    ESP_LOGI(TAG, "摄像头初始化成功: %dx%d @ %u fps", cfg.width, cfg.height, (unsigned)cfg.fps);
    return CAM_OK;

cleanup:
    if (g_cam.xfer_buffer_a) { free(g_cam.xfer_buffer_a); g_cam.xfer_buffer_a = NULL; }
    if (g_cam.xfer_buffer_b) { free(g_cam.xfer_buffer_b); g_cam.xfer_buffer_b = NULL; }
    if (g_cam.frame_buffer) { free(g_cam.frame_buffer); g_cam.frame_buffer = NULL; }
    if (g_cam.frame_sem) { vSemaphoreDelete(g_cam.frame_sem); g_cam.frame_sem = NULL; }
    return CAM_ERR_MEMORY;
}

int cam_start_stream(void) {
    if (!g_cam.initialized) return CAM_ERR;
    if (g_cam.stream_running) return CAM_OK;
    
    esp_err_t ret = usb_streaming_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动视频流失败: %d", ret);
        return CAM_ERR;
    }

    g_cam.stream_running = true;
    return CAM_OK;
}

int cam_stop_stream(void) {
    if (!g_cam.initialized || !g_cam.stream_running) return CAM_OK;

    esp_err_t ret = usb_streaming_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "停止视频流失败: %d", ret);
        return CAM_ERR;
    }
    
    g_cam.stream_running = false;
    return CAM_OK;
}

int cam_capture_frame(unsigned char **data, size_t *size, uint32_t timeout_ms) {
    if (!g_cam.initialized || !g_cam.stream_running) {
        return CAM_ERR;
    }
    
    /* 等待新帧 */
    if (g_cam.frame_sem) {
        if (xSemaphoreTake(g_cam.frame_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            return CAM_ERR_TIMEOUT;
        }
    }
    
    if (g_cam.last_frame_data && g_cam.last_frame_size > 0) {
        *data = g_cam.last_frame_data;
        *size = g_cam.last_frame_size;
        return CAM_OK;
    }
    
    return CAM_ERR;
}

int cam_capture_rgb(RGBImage *rgb, uint32_t timeout_ms) {
    unsigned char *data = NULL;
    size_t size = 0;
    
    if (cam_capture_frame(&data, &size, timeout_ms) != CAM_OK) {
        return CAM_ERR;
    }
    
    return decode_mjpeg_to_rgb(data, size, rgb);
}

int cam_get_resolution(int *width, int *height) {
    if (!g_cam.initialized) return CAM_ERR;
    *width = g_cam.width;
    *height = g_cam.height;
    return CAM_OK;
}

int cam_get_supported_resolutions(uint16_t list[][2], int max, int *count) {
    uvc_frame_size_t *frame_list = NULL;
    size_t list_size = 0;
    size_t cur_index = 0;
    esp_err_t ret;
    
    if (!g_cam.initialized) {
        return CAM_ERR;
    }
    
    /* 获取分辨率列表 */
    ret = uvc_frame_size_list_get(NULL, &list_size, &cur_index);
    if (ret != ESP_OK || list_size == 0) {
        ESP_LOGE(TAG, "获取分辨率列表失败");
        return CAM_ERR_NOT_FOUND;
    }
    
    frame_list = malloc(sizeof(uvc_frame_size_t) * list_size);
    if (!frame_list) return CAM_ERR_MEMORY;
    
    ret = uvc_frame_size_list_get(frame_list, &list_size, &cur_index);
    if (ret != ESP_OK) {
        free(frame_list);
        return CAM_ERR;
    }
    
    int cnt = 0;
    for (size_t i = 0; i < list_size && cnt < max; i++) {
        list[cnt][0] = frame_list[i].width;
        list[cnt][1] = frame_list[i].height;
        cnt++;
        ESP_LOGI(TAG, "支持分辨率: %d x %d", frame_list[i].width, frame_list[i].height);
    }
    
    *count = cnt;
    free(frame_list);
    return CAM_OK;
}

/* ==================== 音频函数 ==================== */

/* 等待 USB 设备连接的超时时间（毫秒），超时则返回错误，不再无限等待 */
#define UAC_CONNECT_WAIT_MS  5000

/**
 * 配置并启动 UAC 流。v1.5.2 中 mic/spk 在同一个 uac_config_t 里配置，
 * 用 usb_streaming_start() 统一启动，因此这里用 is_mic 决定只启用哪一路。
 * @param is_mic true=麦克风（录音），false=扬声器（播放）
 */
static esp_err_t uac_stream_start(const AudioConfig *cfg, bool is_mic) {
    uac_config_t uac_config = {0};
    uint32_t sample_rate = cfg ? cfg->sample_rate : 16000;
    uint16_t bit_resolution = cfg ? cfg->bit_resolution : 16;
    uint8_t channels = cfg ? cfg->channels : 1;
    uint32_t buf_size = sample_rate * channels * (bit_resolution / 8);

    if (is_mic) {
        uac_config.mic_ch_num = channels;
        uac_config.mic_bit_resolution = bit_resolution;
        uac_config.mic_samples_frequence = sample_rate;
        uac_config.mic_buf_size = buf_size;      /* 约 1 秒数据 */
    } else {
        uac_config.spk_ch_num = channels;
        uac_config.spk_bit_resolution = bit_resolution;
        uac_config.spk_samples_frequence = sample_rate;
        uac_config.spk_buf_size = buf_size;
    }

    esp_err_t ret = uac_streaming_config(&uac_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = usb_streaming_start();
    if (ret != ESP_OK) {
        return ret;
    }

    /* 等待 USB 设备连接，超时返回 ESP_ERR_TIMEOUT 由调用方处理 */
    return usb_streaming_connect_wait(UAC_CONNECT_WAIT_MS);
}

static esp_err_t uac_stream_stop(void) {
    return usb_streaming_stop();
}

int audio_record(const char *filename, int seconds) {
    AudioConfig cfg = get_default_audio_config();
    return audio_record_ex(filename, seconds, &cfg);
}

int audio_record_ex(const char *filename, int seconds, const AudioConfig *config) {
    FILE *fp;
    uint8_t *buf;
    size_t read_bytes;
    esp_err_t ret;
    AudioConfig cfg;
    
    if (config == NULL) {
        cfg = get_default_audio_config();
        config = &cfg;
    }
    
    /* 配置并启动麦克风流 */
    ret = uac_stream_start(config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "麦克风启动失败: %d", ret);
        return CAM_ERR;
    }

    /* 打开文件 */
    fp = fopen(filename, "wb");
    if (!fp) {
        uac_stream_stop();
        return CAM_ERR;
    }

    /* 分配读取缓冲区 */
    size_t buf_size = config->sample_rate * config->channels * (config->bit_resolution / 8);
    buf = malloc(buf_size);
    if (!buf) {
        fclose(fp);
        uac_stream_stop();
        return CAM_ERR_MEMORY;
    }

    ESP_LOGI(TAG, "开始录音 %d 秒...", seconds);

    int total_reads = seconds * (config->sample_rate / 1000);  /* 按毫秒读取 */
    for (int i = 0; i < total_reads; i++) {
        ret = uac_mic_streaming_read(buf, buf_size, &read_bytes, 10);
        if (ret == ESP_OK && read_bytes > 0) {
            fwrite(buf, 1, read_bytes, fp);
        }
    }

    ESP_LOGI(TAG, "录音完成");

    free(buf);
    fclose(fp);
    uac_stream_stop();

    return CAM_OK;
}

int audio_play(const char *filename) {
    AudioConfig cfg = get_default_audio_config();
    return audio_play_ex(filename, &cfg);
}

int audio_play_ex(const char *filename, const AudioConfig *config) {
    FILE *fp;
    uint8_t *buf;
    size_t bytes_read;
    esp_err_t ret;
    AudioConfig cfg;
    
    if (config == NULL) {
        cfg = get_default_audio_config();
        config = &cfg;
    }
    
    /* 配置并启动播放器流 */
    ret = uac_stream_start(config, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "播放器启动失败: %d", ret);
        return CAM_ERR;
    }

    /* 打开文件 */
    fp = fopen(filename, "rb");
    if (!fp) {
        uac_stream_stop();
        return CAM_ERR;
    }

    /* 分配缓冲区 */
    size_t buf_size = config->sample_rate * config->channels * (config->bit_resolution / 8) / 10;
    buf = malloc(buf_size);
    if (!buf) {
        fclose(fp);
        uac_stream_stop();
        return CAM_ERR_MEMORY;
    }

    ESP_LOGI(TAG, "开始播放...");

    while ((bytes_read = fread(buf, 1, buf_size, fp)) > 0) {
        ret = uac_spk_streaming_write(buf, bytes_read, 100);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "播放写入失败: %d", ret);
            break;
        }
    }

    ESP_LOGI(TAG, "播放完成");

    free(buf);
    fclose(fp);
    uac_stream_stop();

    return CAM_OK;
}

int audio_echo_test(int seconds) {
    const char *tmp = "/spiffs/echo.raw";
    int ret;
    
    ret = audio_record(tmp, seconds);
    if (ret != CAM_OK) return ret;
    
    ret = audio_play(tmp);
    remove(tmp);
    return ret;
}

int set_speaker_volume(int volume) {
    /* 越界钳位到 [0, 100] */
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;

    /* ctrl_value 直接传音量数值本身（0~100），不是指针 */
    esp_err_t ret = usb_streaming_control(STREAM_UAC_SPK, CTRL_UAC_VOLUME, (void *)(intptr_t)volume);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置扬声器音量失败: %s", esp_err_to_name(ret));
        return CAM_ERR;
    }
    ESP_LOGI(TAG, "扬声器音量 = %d", volume);
    return CAM_OK;
}

int set_speaker_mute(bool mute) {
    esp_err_t ret = usb_streaming_control(STREAM_UAC_SPK, CTRL_UAC_MUTE, (void *)(intptr_t)(mute ? 1 : 0));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置扬声器静音失败: %s", esp_err_to_name(ret));
        return CAM_ERR;
    }
    ESP_LOGI(TAG, "扬声器%s", mute ? "静音" : "取消静音");
    return CAM_OK;
}

/* ==================== 图像处理函数 ==================== */

/* 使用 esp_jpeg 组件解码 MJPEG */
#include "esp_jpeg_dec.h"

int decode_mjpeg_to_rgb(unsigned char *mjpeg, size_t size, RGBImage *rgb) {
    jpeg_dec_handle_t *jpeg_dec = NULL;
    jpeg_dec_io_t *jpeg_io = NULL;
    jpeg_dec_header_info_t *out_info = NULL;
    int ret = CAM_ERR;

    if (!mjpeg || !size || !rgb) return CAM_ERR_PARAM;
    memset(rgb, 0, sizeof(RGBImage));

    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = JPEG_RAW_TYPE_RGB888;

    jpeg_dec = jpeg_dec_open(&config);
    if (!jpeg_dec) {
        return CAM_ERR;
    }

    jpeg_io = calloc(1, sizeof(jpeg_dec_io_t));
    out_info = calloc(1, sizeof(jpeg_dec_header_info_t));
    if (!jpeg_io || !out_info) {
        ret = CAM_ERR_MEMORY;
        goto cleanup;
    }

    jpeg_io->inbuf = mjpeg;
    jpeg_io->inbuf_len = (int)size;

    if (jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info) != JPEG_ERR_OK) {
        goto cleanup;
    }

    rgb->width = out_info->width;
    rgb->height = out_info->height;
    rgb->channels = 3;
    rgb->total_size = rgb->width * rgb->height * 3;

    /* 输出缓冲区必须 16 字节对齐 */
    rgb->data = jpeg_malloc_align(rgb->total_size, 16);
    if (!rgb->data) {
        ret = CAM_ERR_MEMORY;
        goto cleanup;
    }

    /* 跳过已解析的文件头，解码剩余图像数据 */
    int inbuf_consumed = jpeg_io->inbuf_len - jpeg_io->inbuf_remain;
    jpeg_io->inbuf = mjpeg + inbuf_consumed;
    jpeg_io->inbuf_len = jpeg_io->inbuf_remain;
    jpeg_io->outbuf = rgb->data;

    if (jpeg_dec_process(jpeg_dec, jpeg_io) != JPEG_ERR_OK) {
        goto cleanup;
    }

    ret = CAM_OK;

cleanup:
    if (ret != CAM_OK && rgb->data) {
        jpeg_free_align(rgb->data);
        rgb->data = NULL;
    }
    if (out_info) free(out_info);
    if (jpeg_io) free(jpeg_io);
    if (jpeg_dec) jpeg_dec_close(jpeg_dec);
    return ret;
}

void free_rgb_image(RGBImage *rgb) {
    if (rgb && rgb->data) {
        jpeg_free_align(rgb->data);
        rgb->data = NULL;
        rgb->width = 0;
        rgb->height = 0;
        rgb->channels = 0;
        rgb->total_size = 0;
    }
}

int save_rgb_as_ppm(const char *filename, const RGBImage *rgb) {
    FILE *fp;
    if (!rgb || !rgb->data) return CAM_ERR_PARAM;
    
    fp = fopen(filename, "wb");
    if (!fp) return CAM_ERR;
    
    fprintf(fp, "P6\n%d %d\n255\n", rgb->width, rgb->height);
    fwrite(rgb->data, 1, rgb->total_size, fp);
    fclose(fp);
    return CAM_OK;
}

void rgb_to_grayscale(RGBImage *rgb) {
    if (!rgb || !rgb->data) return;

    unsigned char *ptr = rgb->data;
    size_t count = rgb->width * rgb->height;

    for (size_t i = 0; i < count; i++) {
        unsigned char gray = (unsigned char)(0.299 * ptr[0] + 0.587 * ptr[1] + 0.114 * ptr[2]);
        ptr[0] = ptr[1] = ptr[2] = gray;
        ptr += 3;
    }
}

/* RGB→HSV 单像素：R,G,B∈[0,255] → H∈[0,179], S∈[0,255], V∈[0,255]
 * 结果与 OpenCV 的 cv2.cvtColor(img, COLOR_BGR2HSV) 一致。 */
static void rgb_to_hsv_pixel(uint8_t r, uint8_t g, uint8_t b,
                             uint8_t *h, uint8_t *s, uint8_t *v)
{
    int mx = r, mn = r;
    if (g > mx) mx = g;
    if (b > mx) mx = b;
    if (g < mn) mn = g;
    if (b < mn) mn = b;
    int diff = mx - mn;

    *v = (uint8_t)mx;

    if (mx == 0) {
        *s = 0;
    } else {
        *s = (uint8_t)((diff * 255) / mx);
    }

    if (diff == 0) {
        *h = 0;
    } else {
        int hh;
        if (mx == r) {
            hh = (60 * (g - b)) / diff;      /* 可能为负 */
            if (hh < 0) hh += 360;
        } else if (mx == g) {
            hh = 120 + (60 * (b - r)) / diff;
        } else { /* mx == b */
            hh = 240 + (60 * (r - g)) / diff;
        }
        *h = (uint8_t)(hh / 2);              /* 360° → 0~179，与 OpenCV 一致 */
    }
}

int rgb_to_hsv(const RGBImage *rgb, HSVImage *hsv) {
    if (!rgb || !rgb->data || !hsv) return CAM_ERR_PARAM;
    if (rgb->channels != 3) return CAM_ERR_PARAM;
    memset(hsv, 0, sizeof(HSVImage));

    size_t count = (size_t)rgb->width * rgb->height;
    hsv->total_size = count * 3;
    hsv->data = (unsigned char *)malloc(hsv->total_size);
    if (!hsv->data) return CAM_ERR_MEMORY;

    hsv->width = rgb->width;
    hsv->height = rgb->height;
    hsv->channels = 3;

    const uint8_t *in = rgb->data;
    uint8_t *out = hsv->data;
    for (size_t i = 0; i < count; i++) {
        rgb_to_hsv_pixel(in[0], in[1], in[2], &out[0], &out[1], &out[2]);
        in += 3;
        out += 3;
    }
    return CAM_OK;
}

int hsv_in_range(const HSVImage *hsv, const uint8_t lower[3], const uint8_t upper[3], BWImage *mask) {
    if (!hsv || !hsv->data || !lower || !upper || !mask) return CAM_ERR_PARAM;
    if (hsv->channels != 3) return CAM_ERR_PARAM;
    memset(mask, 0, sizeof(BWImage));

    size_t count = (size_t)hsv->width * hsv->height;
    mask->data = (unsigned char *)malloc(count);
    if (!mask->data) return CAM_ERR_MEMORY;

    mask->width = hsv->width;
    mask->height = hsv->height;
    mask->channels = 1;

    const uint8_t *in = hsv->data;
    uint8_t *out = mask->data;
    for (size_t i = 0; i < count; i++) {
        bool ok = (in[0] >= lower[0] && in[0] <= upper[0]) &&
                  (in[1] >= lower[1] && in[1] <= upper[1]) &&
                  (in[2] >= lower[2] && in[2] <= upper[2]);
        *out = ok ? 0 : 255;   /* 反转：范围内=0(黑)，范围外=255(白) */
        in += 3;
        out += 1;
    }
    return CAM_OK;
}

void free_hsv_image(HSVImage *hsv) {
    if (hsv && hsv->data) {
        free(hsv->data);
        hsv->data = NULL;
        hsv->width = 0;
        hsv->height = 0;
        hsv->channels = 0;
        hsv->total_size = 0;
    }
}

void free_bw_image(BWImage *bw) {
    if (bw && bw->data) {
        free(bw->data);
        bw->data = NULL;
        bw->width = 0;
        bw->height = 0;
        bw->channels = 0;
    }
}

bool get_mask(BWImage* mask)
{
    RGBImage rgb;
    HSVImage hsv;
    const uint8_t lower[3] = {0, 0, 0};  //h-s-v三值过滤下限
     const uint8_t upper[3] = {255, 255, 255};//h-s-v三值过滤上限
    int ret = CAM_ERR_TIMEOUT;

    /* 摄像头刚启动需要一点时间连接，超时则重试 */
    for (int t = 0; t < CONNECT_RETRY; t++) {
        ret = cam_capture_rgb(&rgb, 100);
        if (ret == CAM_OK) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGE(TAG, "等待相机");
    }
    
    if (ret != CAM_OK) {
            ESP_LOGE(TAG, "抓帧失败: %d",ret);
            free_rgb_image(&rgb);
            free_hsv_image(&hsv);
            return false;
        }
    
    rgb_to_hsv(&rgb, &hsv);
    ret = hsv_in_range(&hsv, lower, upper, mask);

    if(ret != CAM_OK)
    {
        ESP_LOGE(TAG, "滤图失败: %d",ret);
        free_rgb_image(&rgb);
        free_hsv_image(&hsv);
        return false;
    }
    free_rgb_image(&rgb);
    free_hsv_image(&hsv);
    return true;
}

/* ==================== 清理函数 ==================== */

void cam_cleanup(void) {
    /* 停止所有流（UVC + UAC），未启动时返回错误但无害 */
    usb_streaming_stop();
    g_cam.stream_running = false;

    if (g_cam.xfer_buffer_a) { free(g_cam.xfer_buffer_a); g_cam.xfer_buffer_a = NULL; }
    if (g_cam.xfer_buffer_b) { free(g_cam.xfer_buffer_b); g_cam.xfer_buffer_b = NULL; }
    if (g_cam.frame_buffer) { free(g_cam.frame_buffer); g_cam.frame_buffer = NULL; }
    if (g_cam.frame_sem) { vSemaphoreDelete(g_cam.frame_sem); g_cam.frame_sem = NULL; }

    g_cam.initialized = false;
    g_cam.last_frame_data = NULL;
    g_cam.last_frame_size = 0;

    ESP_LOGI(TAG, "资源已清理");
}

/* ==================== WAV格式音频函数 ==================== */

/**
 * 录音并保存为WAV格式
 */
int audio_record_wav(const char *filename, int seconds) {
    AudioConfig cfg = {16000, 16, 1};  /* 默认: 16kHz, 16bit, 单声道 */
    return audio_record_wav_ex(filename, seconds, &cfg);
}

/**
 * 录音并保存为WAV格式（自定义配置）
 */
int audio_record_wav_ex(const char *filename, int seconds, const AudioConfig *config) {
    FILE *fp = NULL;
    uint8_t *buffer = NULL;
    size_t buffer_size;
    WAVHeader header;
    AudioConfig cfg;
    esp_err_t ret;
    size_t total_samples = 0;
    int result = -1;
    
    /* 使用默认配置或用户配置 */
    if (config == NULL) {
        cfg.sample_rate = 16000;
        cfg.bit_resolution = 16;
        cfg.channels = 1;
        config = &cfg;
    }

    ESP_LOGI(TAG, "开始WAV录音: %s, %d秒, %uHz, %u位, %u声道",
             filename, seconds, (unsigned)config->sample_rate,
             (unsigned)config->bit_resolution, (unsigned)config->channels);

    /* 配置并启动麦克风流 */
    ret = uac_stream_start(config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "麦克风启动失败: %d", ret);
        goto cleanup;
    }

    /* 打开输出文件 */
    fp = fopen(filename, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "创建文件失败: %s", filename);
        goto cleanup;
    }

    /* 分配音频缓冲区 */
    buffer_size = config->sample_rate * config->channels * (config->bit_resolution / 8);
    buffer = malloc(buffer_size);
    if (!buffer) {
        ESP_LOGE(TAG, "内存分配失败");
        goto cleanup;
    }

    /* 先写入WAV头（占位，稍后更新） */
    memset(&header, 0, sizeof(header));
    fwrite(&header, 1, sizeof(header), fp);

    /* 开始录音循环 */
    ESP_LOGI(TAG, "录音中... %d秒", seconds);

    int total_reads = seconds * (config->sample_rate / 100);
    for (int i = 0; i < total_reads; i++) {
        size_t read_bytes = 0;
        ret = uac_mic_streaming_read(buffer, buffer_size, &read_bytes, 10);

        if (ret == ESP_OK && read_bytes > 0) {
            fwrite(buffer, 1, read_bytes, fp);
            total_samples += read_bytes;
        }
    }

    ESP_LOGI(TAG, "录音完成，共 %zu 字节", total_samples);

    /* 更新WAV文件头 */
    uint32_t data_size = total_samples;
    uint32_t file_size = 36 + data_size;

    memcpy(header.chunk_id, "RIFF", 4);
    header.chunk_size = file_size;
    memcpy(header.format, "WAVE", 4);
    memcpy(header.subchunk1_id, "fmt ", 4);
    header.subchunk1_size = 16;
    header.audio_format = 1;
    header.num_channels = config->channels;
    header.sample_rate = config->sample_rate;
    header.bits_per_sample = config->bit_resolution;
    header.byte_rate = config->sample_rate * config->channels * (config->bit_resolution / 8);
    header.block_align = config->channels * (config->bit_resolution / 8);
    memcpy(header.subchunk2_id, "data", 4);
    header.subchunk2_size = data_size;

    fseek(fp, 0, SEEK_SET);
    fwrite(&header, 1, sizeof(header), fp);

    result = 0;
    ESP_LOGI(TAG, "WAV文件保存成功: %s", filename);

cleanup:
    if (buffer) free(buffer);
    if (fp) fclose(fp);
    uac_stream_stop();

    return result;
}

/**
 * 播放WAV格式文件
 */
/* 遍历 RIFF 子块，填充 WAVHeader，跳过 LIST/INFO 等非必要元数据块 */
static int parse_wav_header(FILE *fp, WAVHeader *header)
{
    uint8_t riff_hdr[12];

    if (fread(riff_hdr, 1, sizeof(riff_hdr), fp) != sizeof(riff_hdr)) {
        return -1;
    }
    if (memcmp(riff_hdr, "RIFF", 4) != 0 || memcmp(riff_hdr + 8, "WAVE", 4) != 0) {
        return -1;
    }

    memcpy(header->chunk_id, "RIFF", 4);
    memcpy(header->format, "WAVE", 4);

    bool have_fmt = false, have_data = false;

    for (;;) {
        uint8_t chunk_hdr[8];
        if (fread(chunk_hdr, 1, sizeof(chunk_hdr), fp) != sizeof(chunk_hdr)) {
            break; /* 没有更多块 */
        }
        uint32_t chunk_size = (uint32_t)chunk_hdr[4] | ((uint32_t)chunk_hdr[5] << 8) |
                              ((uint32_t)chunk_hdr[6] << 16) | ((uint32_t)chunk_hdr[7] << 24);

        if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (chunk_size < 16 || fread(fmt, 1, sizeof(fmt), fp) != sizeof(fmt)) {
                return -1;
            }
            memcpy(header->subchunk1_id, "fmt ", 4);
            header->subchunk1_size  = 16;
            header->audio_format    = (uint16_t)(fmt[0] | (fmt[1] << 8));
            header->num_channels    = (uint16_t)(fmt[2] | (fmt[3] << 8));
            header->sample_rate     = (uint32_t)fmt[4] | ((uint32_t)fmt[5] << 8) |
                                      ((uint32_t)fmt[6] << 16) | ((uint32_t)fmt[7] << 24);
            header->byte_rate       = (uint32_t)fmt[8] | ((uint32_t)fmt[9] << 8) |
                                      ((uint32_t)fmt[10] << 16) | ((uint32_t)fmt[11] << 24);
            header->block_align     = (uint16_t)(fmt[12] | (fmt[13] << 8));
            header->bits_per_sample = (uint16_t)(fmt[14] | (fmt[15] << 8));
            have_fmt = true;
            /* 跳过 fmt 块剩余部分（如 WAVE_FORMAT_EXTENSIBLE 的扩展字段） */
            if (chunk_size > sizeof(fmt)) {
                fseek(fp, chunk_size - sizeof(fmt), SEEK_CUR);
            }
        } else if (memcmp(chunk_hdr, "data", 4) == 0) {
            memcpy(header->subchunk2_id, "data", 4);
            header->subchunk2_size = chunk_size;
            have_data = true;
            break; /* data 块之后紧跟音频数据，指针已停在数据起点 */
        } else {
            /* 跳过其它块（LIST/INFO/fact 等），奇数大小补 1 字节对齐 */
            fseek(fp, chunk_size + (chunk_size & 1), SEEK_CUR);
        }
    }

    return (have_fmt && have_data) ? 0 : -1;
}

/* 软件音量：对 int16 PCM 采样做线性缩放（0~100）。
   硬件 SET_CUR 音量对部分摄像头无效，这是 100% 可靠的兜底方案 */
static void apply_pcm_volume(int16_t *samples, size_t sample_count, int volume) {
    if (volume >= 100) return;
    if (volume <= 0) {
        memset(samples, 0, sample_count * sizeof(int16_t));
        return;
    }
    for (size_t i = 0; i < sample_count; i++) {
        samples[i] = (int16_t)((int32_t)samples[i] * volume / 100);
    }
}

int audio_play_wav(const char *filename, int volume) {
    /* 先保存音量，audio_play_wav_ex 会在流启动后再应用（流启动前设置会失败） */
    g_spk_volume = volume;
    return audio_play_wav_ex(filename, NULL);
}

/**
 * 播放WAV格式文件（指定配置）
 */
int audio_play_wav_ex(const char *filename, const AudioConfig *config) {
    FILE *fp = NULL;
    uint8_t *buffer = NULL;
    size_t buffer_size;
    WAVHeader header;
    AudioConfig cfg;
    esp_err_t ret;
    int result = -1;
    
    ESP_LOGI(TAG, "开始播放WAV: %s", filename);
    
    /* ===== 1. 打开WAV文件 ===== */
    fp = fopen(filename, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "无法打开文件: %s", filename);
        goto cleanup;
    }
    
    /* ===== 2. 解析WAV文件头（跳过LIST/INFO等元数据块） ===== */
    if (parse_wav_header(fp, &header) != 0) {
        ESP_LOGE(TAG, "无效的WAV文件格式");
        goto cleanup;
    }

    if (header.audio_format != 1) {
        ESP_LOGE(TAG, "不支持的音频格式: %d (仅支持PCM)", header.audio_format);
        goto cleanup;
    }
    
    /* 获取音频参数 */
    if (config == NULL) {
        cfg.sample_rate = header.sample_rate;
        cfg.bit_resolution = header.bits_per_sample;
        cfg.channels = header.num_channels;
    } else {
        cfg = *config;
    }

    ESP_LOGI(TAG, "WAV信息: %uHz, %u位, %u声道, 数据大小: %u字节",
             (unsigned)cfg.sample_rate, (unsigned)cfg.bit_resolution,
             (unsigned)cfg.channels, (unsigned)header.subchunk2_size);

    /* 配置并启动播放器流 */
    ret = uac_stream_start(&cfg, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "播放器启动失败: %d", ret);
        goto cleanup;
    }

    /* 分配播放缓冲区 */
    buffer_size = cfg.sample_rate * cfg.channels * (cfg.bit_resolution / 8) / 20;
    if (buffer_size < 1024) buffer_size = 1024;
    buffer = malloc(buffer_size);
    if (!buffer) {
        ESP_LOGE(TAG, "内存分配失败");
        goto cleanup;
    }

    /* 开始播放循环 */
    ESP_LOGI(TAG, "播放中...");

    size_t total_read = 0; /* parse_wav_header 已把文件指针停在 data 数据起点 */

    while (total_read < header.subchunk2_size) {
        size_t to_read = buffer_size;
        if (to_read > header.subchunk2_size - total_read) {
            to_read = header.subchunk2_size - total_read;
        }

        size_t read_bytes = fread(buffer, 1, to_read, fp);
        if (read_bytes == 0) break;

        /* 软件音量缩放（16-bit PCM） */
        if (cfg.bit_resolution == 16) {
            apply_pcm_volume((int16_t *)buffer, read_bytes / 2, g_spk_volume);
        }

        ret = uac_spk_streaming_write(buffer, read_bytes, 100);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "播放写入失败: %d", ret);
            break;
        }

        total_read += read_bytes;

        /* 显示进度 */
        int progress = (int)(total_read * 100 / header.subchunk2_size);
        if (progress % 10 == 0 && progress > 0) {
            ESP_LOGI(TAG, "播放进度: %d%%", progress);
        }
    }

    ESP_LOGI(TAG, "播放完成，共 %zu 字节", total_read);
    result = 0;

cleanup:
    if (buffer) free(buffer);
    if (fp) fclose(fp);
    uac_stream_stop();

    return result;
}

/**
 * 获取WAV文件信息
 */
int wav_get_info(const char *filename, AudioConfig *config) {
    FILE *fp = NULL;
    WAVHeader header;
    int result = -1;
    
    if (!config) return -1;
    
    fp = fopen(filename, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "无法打开文件: %s", filename);
        goto cleanup;
    }
    
    if (parse_wav_header(fp, &header) != 0) {
        ESP_LOGE(TAG, "无效的WAV文件");
        goto cleanup;
    }
    
    config->sample_rate = header.sample_rate;
    config->bit_resolution = header.bits_per_sample;
    config->channels = header.num_channels;

    ESP_LOGI(TAG, "WAV信息: %uHz, %u位, %u声道, 数据大小: %u字节",
             (unsigned)config->sample_rate, (unsigned)config->bit_resolution,
             (unsigned)config->channels, (unsigned)header.subchunk2_size);
    
    result = 0;

cleanup:
    if (fp) fclose(fp);
    return result;
=======
/**
 * camera_audio_esp32.c - ESP32-S3 USB摄像头驱动实现
 * 基于 espressif/usb_stream 组件
 * 
 * 编译: 使用 ESP-IDF 构建系统，需在 idf_component.yml 中添加依赖
 *   dependencies:
 *     espressif/usb_stream: "^1.5.2"
 */

#include "camera_audio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "usb_stream.h"
#include "esp_vfs_fat.h"
#include <dirent.h>

/* ==================== 日志标签 ==================== */
static const char *TAG = "CAM_AUDIO";

/* ==================== 内部数据结构 ==================== */

/**
 * WAV文件头结构体
 */
typedef struct {
    /* RIFF头 */
    char     chunk_id[4];      /* "RIFF" */
    uint32_t chunk_size;       /* 文件大小 - 8 */
    char     format[4];        /* "WAVE" */
    
    /* fmt块 */
    char     subchunk1_id[4];  /* "fmt " */
    uint32_t subchunk1_size;   /* 16 (PCM格式) */
    uint16_t audio_format;     /* 1 = PCM */
    uint16_t num_channels;     /* 声道数 */
    uint32_t sample_rate;      /* 采样率 */
    uint32_t byte_rate;        /* sample_rate * num_channels * bits_per_sample/8 */
    uint16_t block_align;      /* num_channels * bits_per_sample/8 */
    uint16_t bits_per_sample;  /* 位深 */
    
    /* data块 */
    char     subchunk2_id[4];  /* "data" */
    uint32_t subchunk2_size;   /* 音频数据大小 */
} WAVHeader;

/**
 * 视频结构体
 */
typedef struct {
    bool initialized;
    bool stream_running;
    uint16_t width;
    uint16_t height;
    uint32_t fps;
    
    /* 缓冲区 */
    uint8_t *xfer_buffer_a;
    uint8_t *xfer_buffer_b;
    uint8_t *frame_buffer;
    size_t xfer_buf_size;
    size_t frame_buf_size;
    
    /* 同步 */
    SemaphoreHandle_t frame_sem;
    unsigned char *last_frame_data;
    size_t last_frame_size;
} CameraContext;

static CameraContext g_cam = {0};
static bool g_audio_init = false;
static int  g_spk_volume = 100;   /* 播放时应用的扬声器音量，默认 100 */
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;   /* storage 分区 wear-leveling 句柄 */

/* ==================== 内部函数 ==================== */

/**
 * 存储区挂载，运行后才允许调用存储区文件
 */
void storage_load(void)
{
        // 1. 挂载 FATFS（storage 分区）
    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl(
        "/storage",          // 挂载路径
        "storage",           // 分区名，必须和 partitions.csv 一致
        &mount_config,
        &s_wl_handle
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FATFS 挂载失败 (%s)", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "FATFS 挂载成功");

    // 2. 打印文件列表（调试用）
    DIR *dir = opendir("/storage");
    if (dir == NULL) {
        ESP_LOGE(TAG, "无法打开 /storage 目录");
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        // 跳过 . 和 ..
        if (entry->d_name[0] == '.') continue;
        ESP_LOGI(TAG, "文件: %s", entry->d_name);
    }
    closedir(dir);
}

/**
 * 摄像头帧回调函数 - 由 usb_stream 组件调用
 */
static void camera_frame_cb(uvc_frame_t *frame, void *arg) {
    (void)arg;

    if (!frame || frame->data_bytes == 0) return;

    /* 打印实际 MJPEG 帧大小，用于确认 frame_buffer_size 是否够用 */
    
    //ESP_LOGI(TAG, "帧大小: %u 字节", (unsigned)frame->data_bytes);

    /* 保存帧数据指针，供 cam_capture_frame 使用 */
    g_cam.last_frame_data = frame->data;
    g_cam.last_frame_size = frame->data_bytes;

    /* 通知有新的帧到达 */
    if (g_cam.frame_sem) {
        xSemaphoreGive(g_cam.frame_sem);
    }
}

/**
 * 获取默认摄像头配置
 */
static CameraConfig get_default_camera_config(void) {
    CameraConfig cfg = {
        .width = 640,
        .height = 480,
        .fps = 15,                       /* 默认 3fps；640x480 支持 25/15fps，15fps 稳定 */
        .xfer_buffer_size = 128 * 1024,  /* 传输缓冲：480p MJPEG 单帧约 20~40KB，128KB 足够 */
        .frame_buffer_size = 256 * 1024, /* 帧缓冲，需容纳整帧 MJPEG，留余量 */
    };
    return cfg;
}

/**
 * 获取默认音频配置
 */
static AudioConfig get_default_audio_config(void) {
    AudioConfig cfg = {
        .sample_rate = 16000,
        .bit_resolution = 16,
        .channels = 1,
    };
    return cfg;
}

/* ==================== 公共摄像头函数 ==================== */

int cam_init(const CameraConfig *config) {
    CameraConfig cfg;
    esp_err_t ret;
    
    if (g_cam.initialized) {
        cam_cleanup();
    }
    
    /* 使用默认配置或用户配置 */
    if (config == NULL) {
        cfg = get_default_camera_config();
    } else {
        cfg = *config;
    }
    
    /* 分配缓冲区 */
    g_cam.xfer_buf_size = cfg.xfer_buffer_size;
    g_cam.frame_buf_size = cfg.frame_buffer_size;
    
    g_cam.xfer_buffer_a = heap_caps_malloc(g_cam.xfer_buf_size, MALLOC_CAP_DEFAULT);
    g_cam.xfer_buffer_b = heap_caps_malloc(g_cam.xfer_buf_size, MALLOC_CAP_DEFAULT);
    g_cam.frame_buffer = heap_caps_malloc(g_cam.frame_buf_size, MALLOC_CAP_DEFAULT);
    
    if (!g_cam.xfer_buffer_a || !g_cam.xfer_buffer_b || !g_cam.frame_buffer) {
        ESP_LOGE(TAG, "缓冲区分配失败");
        goto cleanup;
    }
    
    /* 创建信号量 */
    g_cam.frame_sem = xSemaphoreCreateBinary();
    if (!g_cam.frame_sem) {
        ESP_LOGE(TAG, "信号量创建失败");
        goto cleanup;
    }
    
    /* 配置 UVC 流 */
    uvc_config_t uvc_config = {
        .frame_width = cfg.width,
        .frame_height = cfg.height,
        .frame_interval = FPS2INTERVAL(cfg.fps),
        .xfer_buffer_size = g_cam.xfer_buf_size,
        .xfer_buffer_a = g_cam.xfer_buffer_a,
        .xfer_buffer_b = g_cam.xfer_buffer_b,
        .frame_buffer_size = g_cam.frame_buf_size,
        .frame_buffer = g_cam.frame_buffer,
        .frame_cb = camera_frame_cb,
        .frame_cb_arg = NULL,
        /* 以下为可选参数，组件会自动探测 */
        .format = UVC_FORMAT_MJPEG,      /* 使用 MJPEG 格式 */
        .xfer_type = UVC_XFER_ISOC,      /* 等时传输，相机需支持 */
        .interface = 0,                  /* 0 表示自动探测 */
        .interface_alt = 0,
        .ep_addr = 0,
        .ep_mps = 0,
    };
    
    ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UVC配置失败: %d", ret);
        goto cleanup;
    }
    
    g_cam.width = cfg.width;
    g_cam.height = cfg.height;
    g_cam.fps = cfg.fps;
    g_cam.initialized = true;
    g_cam.stream_running = false;
    
    ESP_LOGI(TAG, "摄像头初始化成功: %dx%d @ %u fps", cfg.width, cfg.height, (unsigned)cfg.fps);
    return CAM_OK;

cleanup:
    if (g_cam.xfer_buffer_a) { free(g_cam.xfer_buffer_a); g_cam.xfer_buffer_a = NULL; }
    if (g_cam.xfer_buffer_b) { free(g_cam.xfer_buffer_b); g_cam.xfer_buffer_b = NULL; }
    if (g_cam.frame_buffer) { free(g_cam.frame_buffer); g_cam.frame_buffer = NULL; }
    if (g_cam.frame_sem) { vSemaphoreDelete(g_cam.frame_sem); g_cam.frame_sem = NULL; }
    return CAM_ERR_MEMORY;
}

int cam_start_stream(void) {
    if (!g_cam.initialized) return CAM_ERR;
    if (g_cam.stream_running) return CAM_OK;
    
    esp_err_t ret = usb_streaming_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "启动视频流失败: %d", ret);
        return CAM_ERR;
    }

    g_cam.stream_running = true;
    return CAM_OK;
}

int cam_stop_stream(void) {
    if (!g_cam.initialized || !g_cam.stream_running) return CAM_OK;

    esp_err_t ret = usb_streaming_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "停止视频流失败: %d", ret);
        return CAM_ERR;
    }
    
    g_cam.stream_running = false;
    return CAM_OK;
}

int cam_capture_frame(unsigned char **data, size_t *size, uint32_t timeout_ms) {
    if (!g_cam.initialized || !g_cam.stream_running) {
        return CAM_ERR;
    }
    
    /* 等待新帧 */
    if (g_cam.frame_sem) {
        if (xSemaphoreTake(g_cam.frame_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
            return CAM_ERR_TIMEOUT;
        }
    }
    
    if (g_cam.last_frame_data && g_cam.last_frame_size > 0) {
        *data = g_cam.last_frame_data;
        *size = g_cam.last_frame_size;
        return CAM_OK;
    }
    
    return CAM_ERR;
}

int cam_capture_rgb(RGBImage *rgb, uint32_t timeout_ms) {
    unsigned char *data = NULL;
    size_t size = 0;
    
    if (cam_capture_frame(&data, &size, timeout_ms) != CAM_OK) {
        return CAM_ERR;
    }
    
    return decode_mjpeg_to_rgb(data, size, rgb);
}

int cam_get_resolution(int *width, int *height) {
    if (!g_cam.initialized) return CAM_ERR;
    *width = g_cam.width;
    *height = g_cam.height;
    return CAM_OK;
}

int cam_get_supported_resolutions(uint16_t list[][2], int max, int *count) {
    uvc_frame_size_t *frame_list = NULL;
    size_t list_size = 0;
    size_t cur_index = 0;
    esp_err_t ret;
    
    if (!g_cam.initialized) {
        return CAM_ERR;
    }
    
    /* 获取分辨率列表 */
    ret = uvc_frame_size_list_get(NULL, &list_size, &cur_index);
    if (ret != ESP_OK || list_size == 0) {
        ESP_LOGE(TAG, "获取分辨率列表失败");
        return CAM_ERR_NOT_FOUND;
    }
    
    frame_list = malloc(sizeof(uvc_frame_size_t) * list_size);
    if (!frame_list) return CAM_ERR_MEMORY;
    
    ret = uvc_frame_size_list_get(frame_list, &list_size, &cur_index);
    if (ret != ESP_OK) {
        free(frame_list);
        return CAM_ERR;
    }
    
    int cnt = 0;
    for (size_t i = 0; i < list_size && cnt < max; i++) {
        list[cnt][0] = frame_list[i].width;
        list[cnt][1] = frame_list[i].height;
        cnt++;
        ESP_LOGI(TAG, "支持分辨率: %d x %d", frame_list[i].width, frame_list[i].height);
    }
    
    *count = cnt;
    free(frame_list);
    return CAM_OK;
}

/* ==================== 音频函数 ==================== */

/* 等待 USB 设备连接的超时时间（毫秒），超时则返回错误，不再无限等待 */
#define UAC_CONNECT_WAIT_MS  5000

/**
 * 配置并启动 UAC 流。v1.5.2 中 mic/spk 在同一个 uac_config_t 里配置，
 * 用 usb_streaming_start() 统一启动，因此这里用 is_mic 决定只启用哪一路。
 * @param is_mic true=麦克风（录音），false=扬声器（播放）
 */
static esp_err_t uac_stream_start(const AudioConfig *cfg, bool is_mic) {
    uac_config_t uac_config = {0};
    uint32_t sample_rate = cfg ? cfg->sample_rate : 16000;
    uint16_t bit_resolution = cfg ? cfg->bit_resolution : 16;
    uint8_t channels = cfg ? cfg->channels : 1;
    uint32_t buf_size = sample_rate * channels * (bit_resolution / 8);

    if (is_mic) {
        uac_config.mic_ch_num = channels;
        uac_config.mic_bit_resolution = bit_resolution;
        uac_config.mic_samples_frequence = sample_rate;
        uac_config.mic_buf_size = buf_size;      /* 约 1 秒数据 */
    } else {
        uac_config.spk_ch_num = channels;
        uac_config.spk_bit_resolution = bit_resolution;
        uac_config.spk_samples_frequence = sample_rate;
        uac_config.spk_buf_size = buf_size;
    }

    esp_err_t ret = uac_streaming_config(&uac_config);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = usb_streaming_start();
    if (ret != ESP_OK) {
        return ret;
    }

    /* 等待 USB 设备连接，超时返回 ESP_ERR_TIMEOUT 由调用方处理 */
    return usb_streaming_connect_wait(UAC_CONNECT_WAIT_MS);
}

static esp_err_t uac_stream_stop(void) {
    return usb_streaming_stop();
}

int audio_record(const char *filename, int seconds) {
    AudioConfig cfg = get_default_audio_config();
    return audio_record_ex(filename, seconds, &cfg);
}

int audio_record_ex(const char *filename, int seconds, const AudioConfig *config) {
    FILE *fp;
    uint8_t *buf;
    size_t read_bytes;
    esp_err_t ret;
    AudioConfig cfg;
    
    if (config == NULL) {
        cfg = get_default_audio_config();
        config = &cfg;
    }
    
    /* 配置并启动麦克风流 */
    ret = uac_stream_start(config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "麦克风启动失败: %d", ret);
        return CAM_ERR;
    }

    /* 打开文件 */
    fp = fopen(filename, "wb");
    if (!fp) {
        uac_stream_stop();
        return CAM_ERR;
    }

    /* 分配读取缓冲区 */
    size_t buf_size = config->sample_rate * config->channels * (config->bit_resolution / 8);
    buf = malloc(buf_size);
    if (!buf) {
        fclose(fp);
        uac_stream_stop();
        return CAM_ERR_MEMORY;
    }

    ESP_LOGI(TAG, "开始录音 %d 秒...", seconds);

    int total_reads = seconds * (config->sample_rate / 1000);  /* 按毫秒读取 */
    for (int i = 0; i < total_reads; i++) {
        ret = uac_mic_streaming_read(buf, buf_size, &read_bytes, 10);
        if (ret == ESP_OK && read_bytes > 0) {
            fwrite(buf, 1, read_bytes, fp);
        }
    }

    ESP_LOGI(TAG, "录音完成");

    free(buf);
    fclose(fp);
    uac_stream_stop();

    return CAM_OK;
}

int audio_play(const char *filename) {
    AudioConfig cfg = get_default_audio_config();
    return audio_play_ex(filename, &cfg);
}

int audio_play_ex(const char *filename, const AudioConfig *config) {
    FILE *fp;
    uint8_t *buf;
    size_t bytes_read;
    esp_err_t ret;
    AudioConfig cfg;
    
    if (config == NULL) {
        cfg = get_default_audio_config();
        config = &cfg;
    }
    
    /* 配置并启动播放器流 */
    ret = uac_stream_start(config, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "播放器启动失败: %d", ret);
        return CAM_ERR;
    }

    /* 打开文件 */
    fp = fopen(filename, "rb");
    if (!fp) {
        uac_stream_stop();
        return CAM_ERR;
    }

    /* 分配缓冲区 */
    size_t buf_size = config->sample_rate * config->channels * (config->bit_resolution / 8) / 10;
    buf = malloc(buf_size);
    if (!buf) {
        fclose(fp);
        uac_stream_stop();
        return CAM_ERR_MEMORY;
    }

    ESP_LOGI(TAG, "开始播放...");

    while ((bytes_read = fread(buf, 1, buf_size, fp)) > 0) {
        ret = uac_spk_streaming_write(buf, bytes_read, 100);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "播放写入失败: %d", ret);
            break;
        }
    }

    ESP_LOGI(TAG, "播放完成");

    free(buf);
    fclose(fp);
    uac_stream_stop();

    return CAM_OK;
}

int audio_echo_test(int seconds) {
    const char *tmp = "/spiffs/echo.raw";
    int ret;
    
    ret = audio_record(tmp, seconds);
    if (ret != CAM_OK) return ret;
    
    ret = audio_play(tmp);
    remove(tmp);
    return ret;
}

int set_speaker_volume(int volume) {
    /* 越界钳位到 [0, 100] */
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;

    /* ctrl_value 直接传音量数值本身（0~100），不是指针 */
    esp_err_t ret = usb_streaming_control(STREAM_UAC_SPK, CTRL_UAC_VOLUME, (void *)(intptr_t)volume);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置扬声器音量失败: %s", esp_err_to_name(ret));
        return CAM_ERR;
    }
    ESP_LOGI(TAG, "扬声器音量 = %d", volume);
    return CAM_OK;
}

int set_speaker_mute(bool mute) {
    esp_err_t ret = usb_streaming_control(STREAM_UAC_SPK, CTRL_UAC_MUTE, (void *)(intptr_t)(mute ? 1 : 0));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "设置扬声器静音失败: %s", esp_err_to_name(ret));
        return CAM_ERR;
    }
    ESP_LOGI(TAG, "扬声器%s", mute ? "静音" : "取消静音");
    return CAM_OK;
}

/* ==================== 图像处理函数 ==================== */

/* 使用 esp_jpeg 组件解码 MJPEG */
#include "esp_jpeg_dec.h"

int decode_mjpeg_to_rgb(unsigned char *mjpeg, size_t size, RGBImage *rgb) {
    jpeg_dec_handle_t *jpeg_dec = NULL;
    jpeg_dec_io_t *jpeg_io = NULL;
    jpeg_dec_header_info_t *out_info = NULL;
    int ret = CAM_ERR;

    if (!mjpeg || !size || !rgb) return CAM_ERR_PARAM;
    memset(rgb, 0, sizeof(RGBImage));

    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type = JPEG_RAW_TYPE_RGB888;

    jpeg_dec = jpeg_dec_open(&config);
    if (!jpeg_dec) {
        return CAM_ERR;
    }

    jpeg_io = calloc(1, sizeof(jpeg_dec_io_t));
    out_info = calloc(1, sizeof(jpeg_dec_header_info_t));
    if (!jpeg_io || !out_info) {
        ret = CAM_ERR_MEMORY;
        goto cleanup;
    }

    jpeg_io->inbuf = mjpeg;
    jpeg_io->inbuf_len = (int)size;

    if (jpeg_dec_parse_header(jpeg_dec, jpeg_io, out_info) != JPEG_ERR_OK) {
        goto cleanup;
    }

    rgb->width = out_info->width;
    rgb->height = out_info->height;
    rgb->channels = 3;
    rgb->total_size = rgb->width * rgb->height * 3;

    /* 输出缓冲区必须 16 字节对齐 */
    rgb->data = jpeg_malloc_align(rgb->total_size, 16);
    if (!rgb->data) {
        ret = CAM_ERR_MEMORY;
        goto cleanup;
    }

    /* 跳过已解析的文件头，解码剩余图像数据 */
    int inbuf_consumed = jpeg_io->inbuf_len - jpeg_io->inbuf_remain;
    jpeg_io->inbuf = mjpeg + inbuf_consumed;
    jpeg_io->inbuf_len = jpeg_io->inbuf_remain;
    jpeg_io->outbuf = rgb->data;

    if (jpeg_dec_process(jpeg_dec, jpeg_io) != JPEG_ERR_OK) {
        goto cleanup;
    }

    ret = CAM_OK;

cleanup:
    if (ret != CAM_OK && rgb->data) {
        jpeg_free_align(rgb->data);
        rgb->data = NULL;
    }
    if (out_info) free(out_info);
    if (jpeg_io) free(jpeg_io);
    if (jpeg_dec) jpeg_dec_close(jpeg_dec);
    return ret;
}

void free_rgb_image(RGBImage *rgb) {
    if (rgb && rgb->data) {
        jpeg_free_align(rgb->data);
        rgb->data = NULL;
        rgb->width = 0;
        rgb->height = 0;
        rgb->channels = 0;
        rgb->total_size = 0;
    }
}

int save_rgb_as_ppm(const char *filename, const RGBImage *rgb) {
    FILE *fp;
    if (!rgb || !rgb->data) return CAM_ERR_PARAM;
    
    fp = fopen(filename, "wb");
    if (!fp) return CAM_ERR;
    
    fprintf(fp, "P6\n%d %d\n255\n", rgb->width, rgb->height);
    fwrite(rgb->data, 1, rgb->total_size, fp);
    fclose(fp);
    return CAM_OK;
}

void rgb_to_grayscale(RGBImage *rgb) {
    if (!rgb || !rgb->data) return;

    unsigned char *ptr = rgb->data;
    size_t count = rgb->width * rgb->height;

    for (size_t i = 0; i < count; i++) {
        unsigned char gray = (unsigned char)(0.299 * ptr[0] + 0.587 * ptr[1] + 0.114 * ptr[2]);
        ptr[0] = ptr[1] = ptr[2] = gray;
        ptr += 3;
    }
}

/* RGB→HSV 单像素：R,G,B∈[0,255] → H∈[0,179], S∈[0,255], V∈[0,255]
 * 结果与 OpenCV 的 cv2.cvtColor(img, COLOR_BGR2HSV) 一致。 */
static void rgb_to_hsv_pixel(uint8_t r, uint8_t g, uint8_t b,
                             uint8_t *h, uint8_t *s, uint8_t *v)
{
    int mx = r, mn = r;
    if (g > mx) mx = g;
    if (b > mx) mx = b;
    if (g < mn) mn = g;
    if (b < mn) mn = b;
    int diff = mx - mn;

    *v = (uint8_t)mx;

    if (mx == 0) {
        *s = 0;
    } else {
        *s = (uint8_t)((diff * 255) / mx);
    }

    if (diff == 0) {
        *h = 0;
    } else {
        int hh;
        if (mx == r) {
            hh = (60 * (g - b)) / diff;      /* 可能为负 */
            if (hh < 0) hh += 360;
        } else if (mx == g) {
            hh = 120 + (60 * (b - r)) / diff;
        } else { /* mx == b */
            hh = 240 + (60 * (r - g)) / diff;
        }
        *h = (uint8_t)(hh / 2);              /* 360° → 0~179，与 OpenCV 一致 */
    }
}

int rgb_to_hsv(const RGBImage *rgb, HSVImage *hsv) {
    if (!rgb || !rgb->data || !hsv) return CAM_ERR_PARAM;
    if (rgb->channels != 3) return CAM_ERR_PARAM;
    memset(hsv, 0, sizeof(HSVImage));

    size_t count = (size_t)rgb->width * rgb->height;
    hsv->total_size = count * 3;
    hsv->data = (unsigned char *)malloc(hsv->total_size);
    if (!hsv->data) return CAM_ERR_MEMORY;

    hsv->width = rgb->width;
    hsv->height = rgb->height;
    hsv->channels = 3;

    const uint8_t *in = rgb->data;
    uint8_t *out = hsv->data;
    for (size_t i = 0; i < count; i++) {
        rgb_to_hsv_pixel(in[0], in[1], in[2], &out[0], &out[1], &out[2]);
        in += 3;
        out += 3;
    }
    return CAM_OK;
}

int hsv_in_range(const HSVImage *hsv, const uint8_t lower[3], const uint8_t upper[3], BWImage *mask) {
    if (!hsv || !hsv->data || !lower || !upper || !mask) return CAM_ERR_PARAM;
    if (hsv->channels != 3) return CAM_ERR_PARAM;
    memset(mask, 0, sizeof(BWImage));

    size_t count = (size_t)hsv->width * hsv->height;
    mask->data = (unsigned char *)malloc(count);
    if (!mask->data) return CAM_ERR_MEMORY;

    mask->width = hsv->width;
    mask->height = hsv->height;
    mask->channels = 1;

    const uint8_t *in = hsv->data;
    uint8_t *out = mask->data;
    for (size_t i = 0; i < count; i++) {
        bool ok = (in[0] >= lower[0] && in[0] <= upper[0]) &&
                  (in[1] >= lower[1] && in[1] <= upper[1]) &&
                  (in[2] >= lower[2] && in[2] <= upper[2]);
        *out = ok ? 0 : 255;   /* 反转：范围内=0(黑)，范围外=255(白) */
        in += 3;
        out += 1;
    }
    return CAM_OK;
}

void free_hsv_image(HSVImage *hsv) {
    if (hsv && hsv->data) {
        free(hsv->data);
        hsv->data = NULL;
        hsv->width = 0;
        hsv->height = 0;
        hsv->channels = 0;
        hsv->total_size = 0;
    }
}

void free_bw_image(BWImage *bw) {
    if (bw && bw->data) {
        free(bw->data);
        bw->data = NULL;
        bw->width = 0;
        bw->height = 0;
        bw->channels = 0;
    }
}

bool get_mask(BWImage* mask)
{
    RGBImage rgb;
    HSVImage hsv;
    const uint8_t lower[3] = {0, 0, 0};  //h-s-v三值过滤下限
     const uint8_t upper[3] = {255, 255, 255};//h-s-v三值过滤上限
    int ret = CAM_ERR_TIMEOUT;

    /* 摄像头刚启动需要一点时间连接，超时则重试 */
    for (int t = 0; t < CONNECT_RETRY; t++) {
        ret = cam_capture_rgb(&rgb, 100);
        if (ret == CAM_OK) break;
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGE(TAG, "等待相机");
    }
    
    if (ret != CAM_OK) {
            ESP_LOGE(TAG, "抓帧失败: %d",ret);
            free_rgb_image(&rgb);
            free_hsv_image(&hsv);
            return false;
        }
    
    rgb_to_hsv(&rgb, &hsv);
    ret = hsv_in_range(&hsv, lower, upper, mask);

    if(ret != CAM_OK)
    {
        ESP_LOGE(TAG, "滤图失败: %d",ret);
        free_rgb_image(&rgb);
        free_hsv_image(&hsv);
        return false;
    }
    free_rgb_image(&rgb);
    free_hsv_image(&hsv);
    return true;
}

/* ==================== 清理函数 ==================== */

void cam_cleanup(void) {
    /* 停止所有流（UVC + UAC），未启动时返回错误但无害 */
    usb_streaming_stop();
    g_cam.stream_running = false;

    if (g_cam.xfer_buffer_a) { free(g_cam.xfer_buffer_a); g_cam.xfer_buffer_a = NULL; }
    if (g_cam.xfer_buffer_b) { free(g_cam.xfer_buffer_b); g_cam.xfer_buffer_b = NULL; }
    if (g_cam.frame_buffer) { free(g_cam.frame_buffer); g_cam.frame_buffer = NULL; }
    if (g_cam.frame_sem) { vSemaphoreDelete(g_cam.frame_sem); g_cam.frame_sem = NULL; }

    g_cam.initialized = false;
    g_cam.last_frame_data = NULL;
    g_cam.last_frame_size = 0;

    ESP_LOGI(TAG, "资源已清理");
}

/* ==================== WAV格式音频函数 ==================== */

/**
 * 录音并保存为WAV格式
 */
int audio_record_wav(const char *filename, int seconds) {
    AudioConfig cfg = {16000, 16, 1};  /* 默认: 16kHz, 16bit, 单声道 */
    return audio_record_wav_ex(filename, seconds, &cfg);
}

/**
 * 录音并保存为WAV格式（自定义配置）
 */
int audio_record_wav_ex(const char *filename, int seconds, const AudioConfig *config) {
    FILE *fp = NULL;
    uint8_t *buffer = NULL;
    size_t buffer_size;
    WAVHeader header;
    AudioConfig cfg;
    esp_err_t ret;
    size_t total_samples = 0;
    int result = -1;
    
    /* 使用默认配置或用户配置 */
    if (config == NULL) {
        cfg.sample_rate = 16000;
        cfg.bit_resolution = 16;
        cfg.channels = 1;
        config = &cfg;
    }

    ESP_LOGI(TAG, "开始WAV录音: %s, %d秒, %uHz, %u位, %u声道",
             filename, seconds, (unsigned)config->sample_rate,
             (unsigned)config->bit_resolution, (unsigned)config->channels);

    /* 配置并启动麦克风流 */
    ret = uac_stream_start(config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "麦克风启动失败: %d", ret);
        goto cleanup;
    }

    /* 打开输出文件 */
    fp = fopen(filename, "wb");
    if (!fp) {
        ESP_LOGE(TAG, "创建文件失败: %s", filename);
        goto cleanup;
    }

    /* 分配音频缓冲区 */
    buffer_size = config->sample_rate * config->channels * (config->bit_resolution / 8);
    buffer = malloc(buffer_size);
    if (!buffer) {
        ESP_LOGE(TAG, "内存分配失败");
        goto cleanup;
    }

    /* 先写入WAV头（占位，稍后更新） */
    memset(&header, 0, sizeof(header));
    fwrite(&header, 1, sizeof(header), fp);

    /* 开始录音循环 */
    ESP_LOGI(TAG, "录音中... %d秒", seconds);

    int total_reads = seconds * (config->sample_rate / 100);
    for (int i = 0; i < total_reads; i++) {
        size_t read_bytes = 0;
        ret = uac_mic_streaming_read(buffer, buffer_size, &read_bytes, 10);

        if (ret == ESP_OK && read_bytes > 0) {
            fwrite(buffer, 1, read_bytes, fp);
            total_samples += read_bytes;
        }
    }

    ESP_LOGI(TAG, "录音完成，共 %zu 字节", total_samples);

    /* 更新WAV文件头 */
    uint32_t data_size = total_samples;
    uint32_t file_size = 36 + data_size;

    memcpy(header.chunk_id, "RIFF", 4);
    header.chunk_size = file_size;
    memcpy(header.format, "WAVE", 4);
    memcpy(header.subchunk1_id, "fmt ", 4);
    header.subchunk1_size = 16;
    header.audio_format = 1;
    header.num_channels = config->channels;
    header.sample_rate = config->sample_rate;
    header.bits_per_sample = config->bit_resolution;
    header.byte_rate = config->sample_rate * config->channels * (config->bit_resolution / 8);
    header.block_align = config->channels * (config->bit_resolution / 8);
    memcpy(header.subchunk2_id, "data", 4);
    header.subchunk2_size = data_size;

    fseek(fp, 0, SEEK_SET);
    fwrite(&header, 1, sizeof(header), fp);

    result = 0;
    ESP_LOGI(TAG, "WAV文件保存成功: %s", filename);

cleanup:
    if (buffer) free(buffer);
    if (fp) fclose(fp);
    uac_stream_stop();

    return result;
}

/**
 * 播放WAV格式文件
 */
/* 遍历 RIFF 子块，填充 WAVHeader，跳过 LIST/INFO 等非必要元数据块 */
static int parse_wav_header(FILE *fp, WAVHeader *header)
{
    uint8_t riff_hdr[12];

    if (fread(riff_hdr, 1, sizeof(riff_hdr), fp) != sizeof(riff_hdr)) {
        return -1;
    }
    if (memcmp(riff_hdr, "RIFF", 4) != 0 || memcmp(riff_hdr + 8, "WAVE", 4) != 0) {
        return -1;
    }

    memcpy(header->chunk_id, "RIFF", 4);
    memcpy(header->format, "WAVE", 4);

    bool have_fmt = false, have_data = false;

    for (;;) {
        uint8_t chunk_hdr[8];
        if (fread(chunk_hdr, 1, sizeof(chunk_hdr), fp) != sizeof(chunk_hdr)) {
            break; /* 没有更多块 */
        }
        uint32_t chunk_size = (uint32_t)chunk_hdr[4] | ((uint32_t)chunk_hdr[5] << 8) |
                              ((uint32_t)chunk_hdr[6] << 16) | ((uint32_t)chunk_hdr[7] << 24);

        if (memcmp(chunk_hdr, "fmt ", 4) == 0) {
            uint8_t fmt[16];
            if (chunk_size < 16 || fread(fmt, 1, sizeof(fmt), fp) != sizeof(fmt)) {
                return -1;
            }
            memcpy(header->subchunk1_id, "fmt ", 4);
            header->subchunk1_size  = 16;
            header->audio_format    = (uint16_t)(fmt[0] | (fmt[1] << 8));
            header->num_channels    = (uint16_t)(fmt[2] | (fmt[3] << 8));
            header->sample_rate     = (uint32_t)fmt[4] | ((uint32_t)fmt[5] << 8) |
                                      ((uint32_t)fmt[6] << 16) | ((uint32_t)fmt[7] << 24);
            header->byte_rate       = (uint32_t)fmt[8] | ((uint32_t)fmt[9] << 8) |
                                      ((uint32_t)fmt[10] << 16) | ((uint32_t)fmt[11] << 24);
            header->block_align     = (uint16_t)(fmt[12] | (fmt[13] << 8));
            header->bits_per_sample = (uint16_t)(fmt[14] | (fmt[15] << 8));
            have_fmt = true;
            /* 跳过 fmt 块剩余部分（如 WAVE_FORMAT_EXTENSIBLE 的扩展字段） */
            if (chunk_size > sizeof(fmt)) {
                fseek(fp, chunk_size - sizeof(fmt), SEEK_CUR);
            }
        } else if (memcmp(chunk_hdr, "data", 4) == 0) {
            memcpy(header->subchunk2_id, "data", 4);
            header->subchunk2_size = chunk_size;
            have_data = true;
            break; /* data 块之后紧跟音频数据，指针已停在数据起点 */
        } else {
            /* 跳过其它块（LIST/INFO/fact 等），奇数大小补 1 字节对齐 */
            fseek(fp, chunk_size + (chunk_size & 1), SEEK_CUR);
        }
    }

    return (have_fmt && have_data) ? 0 : -1;
}

/* 软件音量：对 int16 PCM 采样做线性缩放（0~100）。
   硬件 SET_CUR 音量对部分摄像头无效，这是 100% 可靠的兜底方案 */
static void apply_pcm_volume(int16_t *samples, size_t sample_count, int volume) {
    if (volume >= 100) return;
    if (volume <= 0) {
        memset(samples, 0, sample_count * sizeof(int16_t));
        return;
    }
    for (size_t i = 0; i < sample_count; i++) {
        samples[i] = (int16_t)((int32_t)samples[i] * volume / 100);
    }
}

int audio_play_wav(const char *filename, int volume) {
    /* 先保存音量，audio_play_wav_ex 会在流启动后再应用（流启动前设置会失败） */
    g_spk_volume = volume;
    return audio_play_wav_ex(filename, NULL);
}

/**
 * 播放WAV格式文件（指定配置）
 */
int audio_play_wav_ex(const char *filename, const AudioConfig *config) {
    FILE *fp = NULL;
    uint8_t *buffer = NULL;
    size_t buffer_size;
    WAVHeader header;
    AudioConfig cfg;
    esp_err_t ret;
    int result = -1;
    
    ESP_LOGI(TAG, "开始播放WAV: %s", filename);
    
    /* ===== 1. 打开WAV文件 ===== */
    fp = fopen(filename, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "无法打开文件: %s", filename);
        goto cleanup;
    }
    
    /* ===== 2. 解析WAV文件头（跳过LIST/INFO等元数据块） ===== */
    if (parse_wav_header(fp, &header) != 0) {
        ESP_LOGE(TAG, "无效的WAV文件格式");
        goto cleanup;
    }

    if (header.audio_format != 1) {
        ESP_LOGE(TAG, "不支持的音频格式: %d (仅支持PCM)", header.audio_format);
        goto cleanup;
    }
    
    /* 获取音频参数 */
    if (config == NULL) {
        cfg.sample_rate = header.sample_rate;
        cfg.bit_resolution = header.bits_per_sample;
        cfg.channels = header.num_channels;
    } else {
        cfg = *config;
    }

    ESP_LOGI(TAG, "WAV信息: %uHz, %u位, %u声道, 数据大小: %u字节",
             (unsigned)cfg.sample_rate, (unsigned)cfg.bit_resolution,
             (unsigned)cfg.channels, (unsigned)header.subchunk2_size);

    /* 配置并启动播放器流 */
    ret = uac_stream_start(&cfg, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "播放器启动失败: %d", ret);
        goto cleanup;
    }

    /* 分配播放缓冲区 */
    buffer_size = cfg.sample_rate * cfg.channels * (cfg.bit_resolution / 8) / 20;
    if (buffer_size < 1024) buffer_size = 1024;
    buffer = malloc(buffer_size);
    if (!buffer) {
        ESP_LOGE(TAG, "内存分配失败");
        goto cleanup;
    }

    /* 开始播放循环 */
    ESP_LOGI(TAG, "播放中...");

    size_t total_read = 0; /* parse_wav_header 已把文件指针停在 data 数据起点 */

    while (total_read < header.subchunk2_size) {
        size_t to_read = buffer_size;
        if (to_read > header.subchunk2_size - total_read) {
            to_read = header.subchunk2_size - total_read;
        }

        size_t read_bytes = fread(buffer, 1, to_read, fp);
        if (read_bytes == 0) break;

        /* 软件音量缩放（16-bit PCM） */
        if (cfg.bit_resolution == 16) {
            apply_pcm_volume((int16_t *)buffer, read_bytes / 2, g_spk_volume);
        }

        ret = uac_spk_streaming_write(buffer, read_bytes, 100);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "播放写入失败: %d", ret);
            break;
        }

        total_read += read_bytes;

        /* 显示进度 */
        int progress = (int)(total_read * 100 / header.subchunk2_size);
        if (progress % 10 == 0 && progress > 0) {
            ESP_LOGI(TAG, "播放进度: %d%%", progress);
        }
    }

    ESP_LOGI(TAG, "播放完成，共 %zu 字节", total_read);
    result = 0;

cleanup:
    if (buffer) free(buffer);
    if (fp) fclose(fp);
    uac_stream_stop();

    return result;
}

/**
 * 获取WAV文件信息
 */
int wav_get_info(const char *filename, AudioConfig *config) {
    FILE *fp = NULL;
    WAVHeader header;
    int result = -1;
    
    if (!config) return -1;
    
    fp = fopen(filename, "rb");
    if (!fp) {
        ESP_LOGE(TAG, "无法打开文件: %s", filename);
        goto cleanup;
    }
    
    if (parse_wav_header(fp, &header) != 0) {
        ESP_LOGE(TAG, "无效的WAV文件");
        goto cleanup;
    }
    
    config->sample_rate = header.sample_rate;
    config->bit_resolution = header.bits_per_sample;
    config->channels = header.num_channels;

    ESP_LOGI(TAG, "WAV信息: %uHz, %u位, %u声道, 数据大小: %u字节",
             (unsigned)config->sample_rate, (unsigned)config->bit_resolution,
             (unsigned)config->channels, (unsigned)header.subchunk2_size);
    
    result = 0;

cleanup:
    if (fp) fclose(fp);
    return result;
>>>>>>> 63765a10bd5850d70190c44f78dbd81bfbddc0f5
}