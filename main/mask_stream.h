/**
 * mask_stream.h - 后台取图 + 非阻塞取帧接口
 *
 * 把 get_mask_pro 的阻塞（等摄像头新帧 + 解码）放到独立后台任务里：
 *   - 后台任务持续 get_mask_pro 更新掩码，并可选推送到 /mask（wifi_stream_push_bw）
 *   - 主控制循环只通过 mask_stream_get_latest / mask_stream_wait_new 取最新掩码，
 *     不再被取图卡住
 *
 * 使用流程：
 *   1. cam_init + cam_start_stream 之后调用 mask_stream_start(mode, push_wifi)
 *   2. 主循环用 mask_stream_wait_new(mask, timeout_ms) 等新帧（自然节流到摄像头帧率），
 *      或用 mask_stream_get_latest(mask) 非阻塞取最新帧
 *   3. 结束前 mask_stream_stop()
 */
#ifndef MASK_STREAM_H
#define MASK_STREAM_H

#include "camera_audio.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 启动后台取图任务
 * @param mode      取图模式（REDBALL_MODE / BLUEBALL_MODE / LINE_MODE2 ...）
 * @param push_wifi 是否把每一帧掩码推送到 /mask 端点（需先 wifi_ap_init + wifi_http_start）
 * @return true 成功，false 失败
 */
bool mask_stream_start(int mode, bool push_wifi);

/**
 * 运行时切换取图模式（线程安全，后台任务下一帧生效）
 */
void mask_stream_set_mode(int mode);

/**
 * 停止后台任务并释放资源
 */
void mask_stream_stop(void);

/**
 * 非阻塞取最新帧：把当前最新掩码深拷贝到 out（内部 RAM，用 free_bw_image 释放）
 * @return true 取到有效帧；false 尚无帧 / 参数错误
 */
bool mask_stream_get_latest(BWImage *out);

/**
 * 等待一帧新帧：阻塞直到后台任务发布新帧或超时
 * （有积压帧时立刻返回最新帧；主循环快于摄像头时自然节流在帧率）
 * @param out        输出：最新掩码（内部 RAM，用 free_bw_image 释放）
 * @param timeout_ms 超时(ms)，超时返回 false
 * @return true 取到新帧，false 超时 / 参数错误
 */
bool mask_stream_wait_new(BWImage *out, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* MASK_STREAM_H */
