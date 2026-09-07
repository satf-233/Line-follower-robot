// include本文件可以调用摄像头循迹函数 image_follow，执行视觉巡线功能
//
// 使用流程：
//   1. 主程序自行完成摄像头初始化（storage_load / cam_init / cam_start_stream）
//   2. image_follow_init();                       // 只调用一次，复位循迹控制状态
//   3. while (1) { int st = image_follow(); ... }  // 主循环里反复调用，一次调用 = 一帧闭环
//   4. 返回 IMAGE_FOLLOW_STOP / LOST / NO_FRAME 时由调用方决定是否退出循环

#ifndef IMAGE_H
#define IMAGE_H

#include <stdbool.h>
#include "camera_audio.h"   // BWImage 类型 / get_mask 等接口

// ===== image_follow() 的返回状态 =====
#define IMAGE_FOLLOW_OK        0   // 正常循迹中（含短暂丢线、偶发丢帧，电机未停）
#define IMAGE_FOLLOW_LOST      1   // 丢线超时，已停车
#define IMAGE_FOLLOW_STOP      2   // 检测到横线/终点，已停车
#define IMAGE_FOLLOW_NO_FRAME  3   // 连续取图失败，已停车

/**
 * 摄像头循迹初始化（上电后、进入循迹循环前调用一次）
 * 只复位 PID / 丢线 / 计时等内部控制状态
 * 注意：摄像头初始化（storage_load / cam_init / cam_start_stream）由主程序负责，
 *       电机初始化由 motor_init() 负责，本函数都不做
 * @return 0=成功（当前不会失败，返回值保留以便后续扩展）
 */
int image_init(void);

/**
 * 摄像头循迹主函数：取一帧二值图 -> 求横向偏差 -> PID -> 差速驱动
 * 在主循环里反复调用，函数内部已带一次让出 CPU 的延时
 * @return IMAGE_FOLLOW_OK / LOST / STOP / NO_FRAME
 */
int image_follow(void);

int image_follow_stop(void);

/**
 * 纯视觉函数：从二值图算出归一化横向偏差（不驱动电机，方便单独打印调试）
 * @param mask        输入二值图，黑线=0、场地=255（由 get_mask 提供）
 * @param error       输出：归一化横向偏差，范围 [-1,1]，正=黑线偏向画面右侧
 * @param curve       输出：预瞄曲率 = 远处行误差 - 近处行误差，用于弯道减速（可传 NULL）
 * @param cross_line  输出：是否检测到横线/终点线（可传 NULL）
 * @return true=本帧找到有效黑线（error 等有效），false=丢线（输出参数不更新）
 */

bool image_find_line(void);

bool image_find_ball(void);

bool image_get_line_error(const BWImage *mask, float *error, float *curve, bool *cross_line);

#endif // IMAGE_H

