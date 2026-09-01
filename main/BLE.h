/**
 * BLE.h - 通过 BLE 向电脑发送 MJPEG 帧的接口
 *
 * 基于 ESP-IDF Bluedroid GATT 服务器实现。ESP32-S3 作为 GATT 服务端，
 * 电脑（手机/树莓派等）作为中心设备连接后，订阅"帧数据"特征的通知，
 * 即可收到分片后的 MJPEG 帧。
 *
 * 依赖:
 *   - 需在 sdkconfig 开启 CONFIG_BT_ENABLED / CONFIG_BT_BLUEDROID_ENABLED / CONFIG_BT_BLE_ENABLED
 *   - main/CMakeLists.txt 的 REQUIRES 需包含 bt 与 nvs_flash
 *
 * 服务与特征 UUID:
 *   服务    0xFFE0
 *   帧数据  0xFFE1  (Notify)
 *   控制    0xFFE2  (Write, 可选)
 *
 * 帧传输协议 (在"帧数据"特征上以 Notification 分片发送):
 *   每个通知 = 6 字节头 + 数据负载
 *     [0]     同步字 0x5A
 *     [1]     标志位  bit0=首包(START)  bit1=末包(END)
 *     [2..3]  包序号 (uint16, 小端, 从 0 开始)
 *     [4..5]  本包负载字节数 (uint16, 小端)
 *     [6..]   MJPEG 数据负载
 *   电脑端按序累积负载，直到收到 END 标志即得到完整的一帧 JPEG 数据，
 *   可直接用图片库解码显示。
 *
 * 注意:
 *   - 建议电脑端协商 MTU 到 512 (>= 517) 以提高吞吐；未协商时按 23 处理，速度较慢。
 *   - 摄像头建议使用 320x240 或 640x480，单帧更小，BLE 传输更流畅。
 */

#ifndef BLE_H
#define BLE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 返回值 ==================== */
#define BLE_OK             0
#define BLE_ERR           -1
#define BLE_ERR_PARAM     -2
#define BLE_ERR_NOT_CONN  -3
#define BLE_ERR_BUSY      -4

/* ==================== UUID ==================== */
#define BLE_SERVICE_UUID      0xFFE0
#define BLE_CHAR_FRAME_UUID   0xFFE1
#define BLE_CHAR_CTRL_UUID    0xFFE2

/* ==================== 控制命令 (写入控制特征) ==================== */
#define BLE_CMD_STOP    0x00
#define BLE_CMD_START   0x01

/**
 * 初始化 BLE：初始化 NVS、控制器、Bluedroid，注册 GATT 服务并开始广播。
 * @return BLE_OK 成功，否则为错误码
 */
int ble_init(void);

/**
 * 阻塞等待中心设备连接。
 * @param timeout_ms 超时毫秒数，0 表示一直等待
 * @return BLE_OK 已连接，否则为错误码
 */
int ble_wait_connected(uint32_t timeout_ms);

/**
 * 是否已连接。
 */
bool ble_is_connected(void);

/**
 * 是否已就绪 (已连接且中心设备已订阅通知)。
 */
bool ble_is_ready(void);

/**
 * 发送一帧 MJPEG 数据（自动分片并通过 Notification 发出）。
 * @param data  原始 MJPEG 帧数据
 * @param size  数据字节数
 * @return BLE_OK 成功；BLE_ERR_NOT_CONN 未连接；BLE_ERR_BUSY 未订阅通知
 */
int ble_send_frame(const uint8_t *data, size_t size);

/**
 * 获取中心设备最后写入的控制命令 (见 BLE_CMD_*)。
 * @return 最后一条命令；无命令时返回 -1
 */
int ble_get_last_command(void);

/**
 * 获取协商后的 MTU 值。
 */
uint16_t ble_get_mtu(void);

/**
 * 关闭 BLE 并释放资源。
 */
void ble_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* BLE_H */
