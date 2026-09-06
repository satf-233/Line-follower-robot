/**
 * WIFI.h - ESP32-S3 作为 AP，通过 HTTP 向电脑/手机推送 MJPEG 视频流
 *
 * 工作方式:
 *   ESP32-S3 开启 SoftAP，电脑/手机连上该热点后，用浏览器访问
 *   http://192.168.4.1/ 即可看到实时画面。
 *
 *   - /          返回一个内嵌 <img src="/stream"> 的网页
 *   - /stream    multipart/x-mixed-replace 流，连续发送 MJPEG 帧
 *   - /snapshot  返回单张 JPEG 快照
 *   - /mask      返回最新一帧二值/灰度图 (BWImage)，以灰度 JPEG 发送
 *
 * 依赖:
 *   - main/CMakeLists.txt 的 REQUIRES 需包含 esp_wifi 与 esp_http_server
 *
 * 使用流程:
 *   1. wifi_ap_init()    启动 AP
 *   2. wifi_http_start() 启动 HTTP 服务器
 *   3. 主循环抓帧后调用 wifi_stream_push_frame() 推流；
 *      若还需发送滤波后的二值图，调用 wifi_stream_push_bw() 推掩码
 *   4. 程序结束前 wifi_ap_deinit() 释放资源
 */

#ifndef WIFI_H
#define WIFI_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "camera_audio.h"   /* BWImage 类型 */

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 返回值 ==================== */
#define WIFI_OK             0
#define WIFI_ERR           -1
#define WIFI_ERR_PARAM     -2

/* ==================== AP 配置 ==================== */
#define WIFI_AP_SSID       "ESP32S3-CAM"
#define WIFI_AP_PASS       "12345678"      /* 留空 "" 则开放网络 */
#define WIFI_AP_CHANNEL    1
#define WIFI_AP_MAX_CONN   4

/* ==================== HTTP 服务器 ==================== */
#define WIFI_HTTP_PORT     80

/**
 * 初始化 NVS、Wi-Fi 协议栈并启动 AP 模式。
 * @return WIFI_OK 成功，否则为错误码
 */
int wifi_ap_init(void);

/**
 * 启动 HTTP 服务器，注册 /、/stream、/snapshot 路由。
 * @return WIFI_OK 成功，否则为错误码
 */
int wifi_http_start(void);

/**
 * 停止 HTTP 服务器（可再次 wifi_http_start 重启）。
 */
void wifi_http_stop(void);

/**
 * 把一帧 MJPEG 数据推送到流中（内部会拷贝，供所有 /stream 客户端读取）。
 * @param data  原始 MJPEG 帧数据
 * @param size  数据字节数
 * @return WIFI_OK 成功，否则为错误码
 */
int wifi_stream_push_frame(const uint8_t *data, size_t size);

/**
 * 推送一帧二值/灰度图 (BWImage) 到 /mask 端点。
 * 内部会把 w×h 的像素数据拷贝到 PSRAM，供 /mask 以灰度 JPEG 返回。
 * @param bw 输入二值图 (0=黑/前景, 255=白/背景)
 * @return WIFI_OK 成功，否则为错误码
 */
int wifi_stream_push_bw(const BWImage *bw);

/**
 * 当前正在观看 /stream 的客户端数量（无客户端时可跳过抓帧以省资源）。
 */
int wifi_stream_client_count(void);

/**
 * 把 AP 的 IPv4 地址写入 out（如 "192.168.4.1"）。
 * @param out     输出缓冲区
 * @param out_len 缓冲区长度
 */
void wifi_ap_get_ip_str(char *out, size_t out_len);

/**
 * 关闭 HTTP 服务器并释放 Wi-Fi 资源。
 */
void wifi_ap_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_H */
