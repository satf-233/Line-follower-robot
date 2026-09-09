#ifndef BALL_H
#define BALL_H

#include "motor.h"
#include "avoid.h"
#include "camera_audio.h"
#include "WIFI.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"


static const char *TAG_BALL = "BALL";

// 粗对准：根据球 mask 判断球相对画面竖直中线的位置，并驱动电机原地自转对准。
// mode 传入取图模式（REDBALL_MODE / BLUEBALL_MODE）
// 返回 1=已对正(停车)，0=旋转中，-1=没看到球
//int roughly_aim(BWImage* mask, int mode);

// 细对准：在粗对准基础上用更小死区微调
//int exactly_aim(BWImage* mask, int mode);

// 完整的找球 -> 对正 -> 靠近 -> 绕球流程
void aim(BWImage* mask, int mode);

// 推球：识别终点色块，检测到即推，终点消失即停止
void push(BWImage* mask);
//十二相机版aim
void aim_cam(BWImage* mask, int mode);



#endif
