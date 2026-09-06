/**
 * camera_audio_esp32.h - JQ-CAM12 三合一摄像头 ESP32-S3 驱动接口
 * 基于 ESP-IDF usb_stream 组件
 * 
 * 依赖组件 (在 idf_component.yml 中声明):
 *   espressif/usb_stream: "^1.5.2"
 * 
 * 编译环境: ESP-IDF v5.0 或更高版本
 * 目标芯片: ESP32-S3 (需带 PSRAM)
 */


 /*
 *  使用流程，务必按步骤调用
 *  1. 挂载存储区
 *  2.初始化摄像头(NULL 使用默认 640x480 @ 15fps)
 *  3.启动视频流
 *  4.调用图像获取函数
 *  5.确认长时间(10s以上)不需要图像，则开始清理资源，包括关闭视频流，以及下行
 *  6.关闭摄像头
 * 
 * tips：记得释放存取图像数据的动态内存，包括分配给bw 的内存，rgb与hsv已做内部处理
 */
#ifndef CAMERA_AUDIO_ESP32_H
#define CAMERA_AUDIO_ESP32_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 错误码 ==================== */
#define CAM_OK             0
#define CAM_ERR            -1
#define CAM_ERR_MEMORY     -2
#define CAM_ERR_PARAM      -3
#define CAM_ERR_TIMEOUT    -4
#define CAM_ERR_NOT_FOUND  -5

#define MIN_PIXELS 50
#define LINE_MODE1 0
#define REDBALL_MODE 1
#define BLUEBALL_MODE 2
#define LINE_MODE2 3

/* ==================== 数据结构 ==================== */

/**
 * RGB图像结构体
 */
typedef struct {
    unsigned char *data;     /* RGB数据 (R,G,B连续存储) */
    int width;               /* 图像宽度 */
    int height;              /* 图像高度 */
    int channels;            /* 通道数 (固定为3) */
    size_t total_size;       /* 总字节数 */
} RGBImage;

/**
 * 黑白图像结构体
 */
typedef struct {
    unsigned char *data;     /* 黑白数据 */
    int width;               /* 图像宽度 */
    int height;              /* 图像高度 */
    int channels;            /* 通道数 (固定为1) */
} BWImage;

/**
 * HSV图像结构体
 * H: 0~179 (色相 0~360° 除以2，与 OpenCV COLOR_BGR2HSV 一致)
 * S: 0~255
 * V: 0~255
 */
typedef struct {
    unsigned char *data;     /* HSV数据 (H,S,V连续存储) */
    int width;               /* 图像宽度 */
    int height;              /* 图像高度 */
    int channels;            /* 通道数 (固定为3) */
    size_t total_size;       /* 总字节数 */
} HSVImage;

/**
 * 摄像头配置
 * 注意：这些参数需要根据你的摄像头实际描述符填写
 */
typedef struct {
    uint16_t width;          /* 图像宽度, 如 320, 640, 1280 */
    uint16_t height;         /* 图像高度, 如 240, 480, 720 */
    uint32_t fps;            /* 目标帧率, 如 15, 30 */
    uint32_t xfer_buffer_size; /* 传输缓冲区大小，需大于一帧大小 */
    uint32_t frame_buffer_size; /* 帧缓冲区大小 */
} CameraConfig;

/**
 * 音频配置
 */
typedef struct {
    uint32_t sample_rate;    /* 采样率, 如 16000, 44100 */
    uint8_t bit_resolution;  /* 位宽, 如 16 */
    uint8_t channels;        /* 声道数, 1=单声道 */
} AudioConfig;

#define CONNECT_RETRY   20    /* 等待摄像头连接的次数（每次 100ms，共约 2s） */
/* ==================== 核心功能函数 ==================== */

/** 存储区挂载函数
 * @param 无
 * @param 无
 */
void storage_load(void);

/** 取图函数
 * @param mask 获取黑白图的存储指针
 * @param type 切换函数功能的指示符
 * @return  false表示取图失败，mask中不是有效内容，true表示成功
 */
bool get_mask(BWImage* mask, int mode);

/** 取图函数加强版
 * @param mask 获取黑白图的存储指针
 * @param row_start 起始行
 * @param row_end   终止行
 * @param type 切换函数功能的指示符
 * @return  false表示取图失败，mask中不是有效内容，true表示成功
 */
bool get_mask_pro(BWImage* mask, int row_start, int row_end, int mode);

/* ============ 摄像头功能函数 ============ */
/**
 * 初始化摄像头驱动
 * @param config  配置参数, NULL则使用默认
 * @return        0=成功, 负数=错误码
 */
int cam_init(const CameraConfig *config);

/**
 * 启动视频流
 * @return    0=成功, 负数=错误码
 */
int cam_start_stream(void);

/**
 * 停止视频流 (暂停)
 * @return    0=成功, 负数=错误码
 */
int cam_stop_stream(void);

/**
 * 捕获一帧并解码为灰度图，然后输出滤波后的黑白图，只截取指定行区间（所有列全保留）
 * @param mask       输出: 行区间视黑白图，mask->data有独立动态空间
 * @param row_start  起始行 (含)，负值按 0 处理
 * @param row_end    结束行 (不含)，<=0 表示截到图像底边（0,0 = 整帧）
 * @param timeout_ms 超时时间(毫秒)
 * @param threshole  过滤阈值，小于此值的像素置为黑色
 * @return           0=成功, 负数=错误码
 */

int cam_capture_linemask(BWImage *mask, int row_start, int row_end, uint32_t timeout_ms, uint8_t thres);

/**
 * 捕获一帧并解码为RGB，然后输出滤波后的黑白图，只截取指定行区间（所有列全保留）
 * @param mask       输出: 行区间视黑白图，mask->data有独立动态空间
 * @param row_start  起始行 (含)，负值按 0 处理
 * @param row_end    结束行 (不含)，<=0 表示截到图像底边（0,0 = 整帧）
 * @param upper[3]   滤波上限
 * @param lower[3]   滤波下限
 * @param timeout_ms 超时时间(毫秒)
 * @return           0=成功, 负数=错误码
 */

int cam_capture_ballmask(BWImage *mask, int row_start, int row_end, uint32_t timeout_ms, uint8_t upper[3], uint8_t lower[3]);

/**
 * 捕获一帧并解码为RGB，只截取指定行区间（所有列全保留），零拷贝
 * @param rgb        输出: 行区间视图，data 指向 full->data 内部（勿 free rgb，应 free full）
 * @param full       输出: 整帧解码结果，拥有底层缓冲，用完后必须 free_rgb_image(full) 释放
 * @param row_start  起始行 (含)，负值按 0 处理
 * @param row_end    结束行 (不含)，<=0 表示截到图像底边（0,0 = 整帧）
 * @param timeout_ms 超时时间(毫秒)
 * @return           0=成功, 负数=错误码
 */
int cam_capture_rgb(RGBImage *rgb, RGBImage *full, int row_start, int row_end, uint32_t timeout_ms);

/**
 * 抓一帧并直接解码为灰度图（亮度 Y），跳过 RGB888 与 HSV 流程，用于巡线等只需亮度阈值的场景。
 * 内部：MJPEG -> RGB565 -> 64KB 查表转灰度，输出 1 字节/像素。
 * @param gray       输出: 灰度图 (0=黑, 255=白)，data 在 PSRAM，用 free_bw_image() 释放
 * @param row_start  起始行(含)，负值按 0 处理
 * @param row_end    结束行(不含)，<=0 表示截到图像底边（0,0 = 整帧）
 * @param timeout_ms 超时时间(毫秒)
 * @return           0=成功, 负数=错误码
 */
int cam_capture_gray(BWImage *gray, int row_start, int row_end, uint32_t timeout_ms);

/**
 * RGB转灰度图 (原地修改)
 */
void rgb_to_grayscale(RGBImage *rgb);

/**
 * 释放RGB图像内存
 */
void free_rgb_image(RGBImage *rgb);

/**
 * 清理所有资源 (程序结束前必须调用)
 */
void cam_cleanup(void);


/* ============ 录音机和扬声器功能函数 ============ */
/**
 * 播放WAV格式文件
 * @param filename  要播放的文件名
 * @param volume    播放音量
 * @return          0=成功, 负数=错误码
 */
int audio_play_wav(const char *filename, int volume);

/**
 * 录音 (保存为PCM文件)
 * @param filename  输出文件名
 * @param seconds   录音时长(秒)
 * @return          0=成功, 负数=错误码
 */
int audio_record(const char *filename, int seconds);

/**
 * 播放音频 (PCM格式)
 * @param filename  要播放的文件名
 * @return          0=成功, 负数=错误码
 */
int audio_play(const char *filename);

/* ============ 图像处理函数 ============ */

/**
 * 解码MJPEG为RGB
 */
int decode_mjpeg_to_rgb(unsigned char *mjpeg_data, size_t mjpeg_size, RGBImage *rgb);

/**
 * 保存RGB为PPM图像 (可用于查看)
 */
int save_rgb_as_ppm(const char *filename, const RGBImage *rgb);

/**
 * RGB图像编码为JPEG（MJPEG）
 * @param rgb      输入RGB图像（3通道RGB888）
 * @param out_data 输出：JPEG字节流（malloc分配，调用者 free 释放）
 * @param out_size 输出：JPEG字节数
 * @return         CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY / CAM_ERR
 */
int rgb_to_jpeg(const RGBImage *rgb, unsigned char **out_data, size_t *out_size);

/**
 * HSV图像编码为JPEG：把 H,S,V 三通道直接当作 R,G,B 编码，
 * 生成一张用颜色显示 H/S/V 各通道的可视化图（调试用）
 * @param hsv      输入HSV图像（3通道 H,S,V 连续存储）
 * @param out_data 输出：JPEG字节流（malloc分配，调用者 free 释放）
 * @param out_size 输出：JPEG字节数
 * @return         CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY / CAM_ERR
 */
int hsv_to_jpeg(const HSVImage *hsv, unsigned char **out_data, size_t *out_size);

/**
 * 灰度图(BWImage)编码为JPEG灰度字节流
 * @param bw       输入灰度图（1 通道，0=黑 255=白）
 * @param out_data 输出：JPEG字节流（malloc分配，调用者 free 释放）
 * @param out_size 输出：JPEG字节数
 * @return         CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY / CAM_ERR
 */
int gray_to_jpeg(const BWImage *bw, unsigned char **out_data, size_t *out_size);

/**
 * 二值图转RGB可视化图：0(黑线)->白，255(场地)->黑，用于电脑端查看滤波结果
 * @param bw   输入二值图
 * @param rgb  输出RGB图，data 用 jpeg_malloc_align 分配，需 free_rgb_image() 释放
 * @return     CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY
 */
int bw_to_rgb(const BWImage *bw, RGBImage *rgb);

/**
 * RGB转HSV
 * 说明：本函数与电脑端 OpenCV 对同一画面做 cv2.cvtColor(img, COLOR_BGR2HSV)
 *      的结果一致（H 范围 0~179），电脑上调好的 lower/upper 阈值可直接搬来用。
 * @param rgb  输入RGB图像
 * @param hsv  输出HSV图像 (调用后需 free_hsv_image() 释放)
 * @return     0=成功, 负数=错误码
 */
int rgb_to_hsv(const RGBImage *rgb, HSVImage *hsv);

/**
 * HSV阈值检测 (仿照 cv2.inRange(hsv, lower, upper))
 * 对每个像素判断 lower <= hsv <= upper (逐通道)，生成二值掩码。
 * 注意：色相 H 是按普通区间比较，不做跨 0 环绕；若目标色跨越 0/179（如红色），
 *      需调两次再取并集（与 OpenCV 里两次 inRange + bitwise_or 同理）。
 * @param hsv    输入HSV图像
 * @param lower  下限 [H,S,V]
 * @param upper  上限 [H,S,V]
 * @param mask   输出二值掩码 (0=在范围内, 255=不在)，调用后需 free_bw_image() 释放
 * @return       0=成功, 负数=错误码
 */
int hsv_in_range(const HSVImage *hsv, const uint8_t lower[3], const uint8_t upper[3], BWImage *mask);

/**
 * RGB阈值检测 (仿照 cv2.inRange(rgb, lower, upper))
 * 对每个像素判断 lower <= rgb <= upper（逐通道，R,G,B 各自比较），生成二值掩码。
 * 与 hsv_in_range 逻辑完全一致，只是直接在 RGB 三通道上做区间判断。
 * @param rgb    输入RGB图像
 * @param lower  下限 [R,G,B]
 * @param upper  上限 [R,G,B]
 * @param mask   输出二值掩码 (0=在范围内, 255=不在)，调用后需 free_bw_image() 释放
 * @return       0=成功, 负数=错误码
 */
int rgb_in_range(const RGBImage *rgb, const uint8_t lower[3], const uint8_t upper[3], BWImage *mask);






/**
 * 捕获一帧图像 (原始MJPEG数据)
 * @param data    输出: 图像数据指针 (指向内部缓冲区，下次调用可能被覆盖)
 * @param size    输出: 数据大小
 * @param timeout_ms 超时时间(毫秒)
 * @return        0=成功, 负数=错误码
 */
int cam_capture_frame(unsigned char **data, size_t *size, uint32_t timeout_ms);

/**
 * 获取当前分辨率
 */
int cam_get_resolution(int *width, int *height);

/**
 * 获取支持的分辨率列表
 * @param list    输出: 分辨率数组 (调用者分配)
 * @param max     最大数量
 * @param count   输出: 实际数量
 * @return        0=成功
 */
int cam_get_supported_resolutions(uint16_t list[][2], int max, int *count);

/**
 * 录音 (自定义配置)
 * @param filename  输出文件名
 * @param seconds   录音时长(秒)
 * @param config    音频配置, NULL则使用默认
 * @return          0=成功, 负数=错误码
 */
int audio_record_ex(const char *filename, int seconds, const AudioConfig *config);


/**
 * 播放音频 (自定义配置)
 * @param filename  要播放的文件名
 * @param config    音频配置, NULL则使用默认
 * @return          0=成功, 负数=错误码
 */
int audio_play_ex(const char *filename, const AudioConfig *config);

/**
 * 录音并保存为WAV格式
 * @param filename  输出文件名 (如 "/spiffs/recording.wav")
 * @param seconds   录音时长(秒)
 * @return          0=成功, 负数=错误码
 */
int audio_record_wav(const char *filename, int seconds);

/**
 * 录音并保存为WAV格式（自定义配置）
 * @param filename  输出文件名
 * @param seconds   录音时长(秒)
 * @param config    音频配置, NULL则使用默认
 * @return          0=成功, 负数=错误码
 */
int audio_record_wav_ex(const char *filename, int seconds, const AudioConfig *config);

/**
 * 播放WAV格式文件（指定配置）
 * @param filename  要播放的文件名
 * @param config    音频配置, NULL则从WAV头读取
 * @return          0=成功, 负数=错误码
 */
int audio_play_wav_ex(const char *filename, const AudioConfig *config);

/**
 * 获取WAV文件信息
 * @param filename  WAV文件名
 * @param config    输出: 音频配置
 * @return          0=成功, 负数=错误码
 */
int wav_get_info(const char *filename, AudioConfig *config);

/**
 * 回声测试 (录音并立即播放)
 * @param seconds   录音时长(秒)
 * @return          0=成功
 */
int audio_echo_test(int seconds);

/**
 * 调节扬声器音量
 * 注意：必须在扬声器流启动之后调用（例如 audio_play_wav 播放过程中），
 *       否则返回错误。音量越界会自动钳位到 [0, 100]。
 * @param volume  音量 0~100
 * @return        0=成功, 负数=错误码
 */
int set_speaker_volume(int volume);

/**
 * 扬声器静音 / 取消静音
 * 注意：同 set_speaker_volume，需在扬声器流启动之后调用。
 * @param mute    true=静音, false=取消静音
 * @return        0=成功, 负数=错误码
 */
int set_speaker_mute(bool mute);

/**
 * 释放HSV图像内存
 */
void free_hsv_image(HSVImage *hsv);

/**
 * 释放黑白图像内存
 */
void free_bw_image(BWImage *bw);

/**
 * 二值图腐蚀（3×3 结构元，前景=黑(0)，背景=白(255)）
 * 黑像素仅当其 3×3 邻域全为黑时才保留，否则置白：黑色区域收缩，
 * 用于去掉白底上的孤立小黑点、细毛刺。图像边界按背景(白)处理。
 * @param src 输入二值图
 * @param dst 输出二值图，可与 src 为同一对象（原地处理），旧缓冲会被释放
 * @return    CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY
 */
int mask_erode(const BWImage *src, BWImage *dst);

/**
 * 二值图膨胀（3×3 结构元，前景=黑(0)，背景=白(255)）
 * 像素 3×3 邻域内只要有黑就输出黑：黑色区域扩张，用于填掉线内小空洞、
 * 把断线重新连起来。图像边界按背景(白)处理。
 * @param src 输入二值图
 * @param dst 输出二值图，可原地处理
 * @return    CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY
 */
int mask_dilate(const BWImage *src, BWImage *dst);

/**
 * 连通域过滤：删除面积（像素数）小于 min_pixels 的黑色连通块（置白）。
 * 连通性按 8 邻域计算。用于去掉大块噪斑，保留线这类较大连通域。
 * @param src        输入二值图（0=黑/前景，255=白/背景）
 * @param dst        输出二值图，可原地处理
 * @param min_pixels 最小保留面积；面积 < 该值的黑块被整块删除
 * @return           CAM_OK / CAM_ERR_PARAM / CAM_ERR_MEMORY
 */
int mask_remove_small_blobs(const BWImage *src, BWImage *dst, int min_pixels);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_AUDIO_ESP32_H */