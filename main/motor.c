#include "motor.h"
#include "pins.h"
#include "utils.h"
#include "driver/gpio.h"
#include "esp_log.h"

// ================================================================
// 电机运动控制 —— 框架
//   - 初始化：把 STBY/PWMA/AIN1/AIN2/PWMB/BIN1/BIN2/DIN1/DIN2/PWMD 配置为输出，拉高 STBY 使能
//   - 单路电机：PWMA/PWMB/PWMD 的 PWM 可用 utils.h 的 pwm_set_duty() 输出占空比
//   - A，D控制动力轮，B控制转向轮
//     
// ================================================================

void motor_init(void)
{
    // 把所有电机相关 GPIO 配置为输出
    // TB6612 每路电机需要 3 个控制信号（PWM、IN1、IN2），A/B 两路共用 STBY 使能脚
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PWMA) | (1ULL << AIN1) | (1ULL << AIN2)
                      | (1ULL << PWMB) | (1ULL << BIN1) | (1ULL << BIN2)
                      | (1ULL << PWMD) | (1ULL << DIN1) | (1ULL << DIN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    // 使能驱动板（STBY 不拉高，电机不动）
    //gpio_set_level(STBY, 1);
    ESP_LOGI("MOTOR", "Motor init success!");
}

// 电机单独控制
// CCW为逆时针，CW为顺时针
void motorA_CCW(float duty)
{
    gpio_set_level(AIN1, 1);
    gpio_set_level(AIN2, 0);
    pwm_set_duty(PWMA, duty);
}
void motorA_CW(float duty)
{
    gpio_set_level(AIN1, 0);
    gpio_set_level(AIN2, 1);
    pwm_set_duty(PWMA, duty);
}

void motorB_CCW(float duty)
{
    gpio_set_level(BIN1, 0);
    gpio_set_level(BIN2, 1);
    pwm_set_duty(PWMB, duty);
}
void motorB_CW(float duty)
{
    gpio_set_level(BIN1, 1);
    gpio_set_level(BIN2, 0);
    pwm_set_duty(PWMB, duty);
}

void motorD_CCW(float duty)
{
    gpio_set_level(DIN1, 0);
    gpio_set_level(DIN2, 1);
    pwm_set_duty(PWMD, duty);
}
void motorD_CW(float duty)
{
    gpio_set_level(DIN1, 1);
    gpio_set_level(DIN2, 0);
    pwm_set_duty(PWMD, duty);
}

void motorA_stop(void)
{
    gpio_set_level(AIN1, 0);
    gpio_set_level(AIN2, 0);
    pwm_set_duty(PWMA, 0);
}
void motorB_stop(void)
{
    gpio_set_level(BIN1, 0);
    gpio_set_level(BIN2, 0);
    pwm_set_duty(PWMB, 0);
}
void motorD_stop(void)
{
    gpio_set_level(DIN1, 0);
    gpio_set_level(DIN2, 0);
    pwm_set_duty(PWMD, 0);
}

void motor_forward(float duty)
{
    motorD_CCW(duty);
    motorA_CW(duty);
    motorB_stop();
}
void motor_backward(float duty)
{
    motorD_CW(duty);
    motorA_CCW(duty);
}

void motor_turn_left(float duty, float kp)
{
    motorA_CW(duty + kp);
    motorD_CCW(duty - kp);
}
void motor_turn_right(float duty, float kp)
{
    motorD_CCW(duty + kp);
    motorA_CW(duty - kp);
}

void motor_turn_plus_CW(float duty)//顺时针
{
    motorA_CCW(duty);
    motorB_CCW(duty);    
    motorD_CCW(duty);
}
void motor_turn_plus_CCW(float duty)//逆时针
{
    motorA_CW(duty);
    motorB_CW(duty);
    motorD_CW(duty);
}

// 1是顺时针，-1是逆时针
void motor_turn_plus(int dir, float duty)
{
    if (dir == 1)
    {
        motor_turn_plus_CW(duty);
    }
    else if (dir == -1)
    {
        motor_turn_plus_CCW(duty);
    }
}

void motor_stop_drive(void)
{
    motorA_stop();
    motorD_stop();
}

void motor_stop(void)
{
    motorA_stop();
    motorB_stop();
    motorD_stop();
}
