#ifndef AVOID_H
#define AVOID_H

#include <stdbool.h>

// HC-SR04 测距模块

// 特殊返回值
#define AVOID_NO_ECHO_CM   (-1.0f)   // ECHO 从未变高：前方无遮挡 / 测量失败
#define AVOID_OUT_RANGE_CM (-2.0f)   // ECHO 高电平持续超时：超出量程

// 初始化超声波 TRIG/ECHO 引脚（启动时调用一次）
void avoid_init(void);

// 触发一次 HC-SR04 并返回测量距离（单位 cm）
//   返回值：
//     >= 2.0f 且 <= 400.0f   -> 正常距离
//     AVOID_NO_ECHO_CM        -> 无回波
//     AVOID_OUT_RANGE_CM      -> 超量程
float avoid_measure_cm(void);


/*                           ===== 执行避障程序 =====
    避障逻辑：测距小于进入阈值后，主程序状态切换为避障状态，避障函数内部进入绕行状态，小车
    向左平移，直到测距大于退出阈值后，避障函数内部进入找回状态，小车先向前移动一段距离，再
    向右平移，当红外传感器电平出现0（检测到黑色）时，返回true，主程序切换为循迹状态。

    TO DO: 优化返回true的逻辑，例如增加防抖：连续n次检测到黑色时返回true */ 
bool avoid_run();

//将来合并到motor.c当中
void Move(int type);
void Move_Fire(int type);

#endif // AVOID_H


