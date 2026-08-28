#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

// 电机编号，与 pins.h 中 E1（A电机）/ E2（B电机）/ E4（D电机）对应
typedef enum {
    MOTOR_A = 1,   // E1A / E1B
    MOTOR_B = 2,   // E2A / E2B
    MOTOR_D = 4,   // E4A / E4B
    MOTOR_MAX
} motor_id_t;

// 初始化霍尔编码器（启动时调用一次）
void encoder_init(void);

// 读取电机转速（RPM；正=正转，负=反转，方向取决于接线）
float encoder_get_rpm(motor_id_t motor);

// 读取累计脉冲数（带方向；用于里程/圈数统计）
int32_t encoder_get_pulses(motor_id_t motor);

// 清零某台电机的累计脉冲数
void encoder_reset(motor_id_t motor);

#endif // ENCODER_H
