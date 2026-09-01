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

#define CONNECT_RETRY   50    /* 等待摄像头连接的次数（每次 100ms，共约 5s） */
/* ==================== 核心功能函数 ==================== */

/** 存储区挂载函数
 * @param 无
 * @param 无
 */
void storage_load(void);

/** 取图函数
 * @param mask 获取黑白图的存储指针，mask内data指向unsigned char 一维矩阵，借助height和width信息读取，约定黑色识别物体，白色为滤去背景
 * @return  false表示取图失败，mask中不是有效内容，true表示成功
 */
bool get_mask(BWImage* mask);

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
 * 捕获一帧并解码为RGB
 * @param rgb         输出: RGB图像 (调用后需调用 free_rgb_image() 释放)
 * @param timeout_ms  超时时间(毫秒)
 * @return            0=成功, 负数=错误码
 */
int cam_capture_rgb(RGBImage *rgb, uint32_t timeout_ms);

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

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_AUDIO_ESP32_H */