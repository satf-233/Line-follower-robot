#ifndef MOTOR_H
#define MOTOR_H

// 初始化电机相关 GPIO（启动时调用一次）
void motor_init(void);

// 电机单独控制（保留接口，可以用于微调）
// CCW为逆时针，CW为顺时针
void motorA_CCW(float duty);
void motorA_CW(float duty);
void motorB_CCW(float duty);
void motorB_CW(float duty);
void motorD_CCW(float duty);
void motorD_CW(float duty);
void motorA_stop(void);
void motorB_stop(void);
void motorD_stop(void);

// 封装好后的控制函数
// 前进 / 后退（duty：占空比 0.0 ~ 1.0，用于控制速度，0为停止，1为全速）
void motor_forward(float duty);
void motor_backward(float duty);
// 左转 / 右转（duty：占空比 0.0 ~ 1.0，用于控制速度，0为停止，1为全速）
void motor_turn_left(float duty, float kp);
void motor_turn_right(float duty, float kp);
// 双向转向，speed_coe参数区间在-1~1，负数为左，正数为右
void motor_turn(float speed_coe);

// 原地旋转
void motor_turn_plus_CW(float duty);
void motor_turn_plus_CCW(float duty);

void motor_turn_plus(int dir, float duty);
/*
    不建议通过类似motor_forward(0)的方式来使驱动轮停止运转
    使用stop类函数可以让代码可读性更佳，并且易于查找
*/
// 停止动力轮（仅停止供电，不制动）（不会将 使能STBY 修改为0）
void motor_stop_drive(void);
// 停止转向轮（仅停止供电，不制动）（不会将 使能STBY 修改为0）
void motor_stop_turn(void);
// 停止所有轮（仅停止供电，不制动）（不会将 使能STBY 修改为0）
void motor_stop(void);

#endif // MOTOR_H
