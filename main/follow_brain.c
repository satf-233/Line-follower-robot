#include "follow_brain.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"
#include "driver/gpio.h"
#include "led_strip.h"

#define COEFFICIENT 0.1;

// ================================================================
// 循迹逻辑：
//   自动调取红外传感器思路数据，根据高低电平判断路况得出一个-1到1的速度参数speed_coe,并发出转向指令
// ================================================================
// 传感器扫描到黑块时输出低电平，扫描到白块输出高电平，假定为白底黑线

void ir_init(void)
{
    // 把四路红外传感器引脚配置为输入，才能用 gpio_get_level 读到电平
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << IR1) | (1ULL << IR2) | (1ULL << IR3) | (1ULL << IR4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, // 红外模块多为开漏输出，需上拉；若模块自带推挽输出可改为 0
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

// void keep(int state, float speed)
// {
//     switch (state)
//         {
//         case -1:
//             motor_turn_left(speed);
//             break;
//         case 1:
//             motor_turn_right(speed);
//             break;
//         case 0:
//             motor_forward(speed);
//             break;
//         default:
//             break;
//         }
// }

void follow(led_strip_handle_t led_strip)
{
    // 需要调参
    float base_speed = 0.16;   // 直线基准速度（占空比 0~1）
    float kp = 0.03;           // 比例系数：误差越大，差速修正越强
    float kd = 0.02;           // 微分系数：阻尼左右摆动，防止蛇形
    float deadband = 0.5f;     // 死区：|误差|小于此值视为居中，直接直行
    float turn_speed_2 = 0.10;  // 急弯原地旋转速度
    int turn_delay_time = 100; // 急弯前冲延时（ms）
    int keep_time = 1;        // 一轮周期（ms）

    static float last_error = 0.0f; // 上一轮误差，用于微分项
    static int turn_dir = 0;        // 原地转弯方向锁存：+1=右转(CW)，-1=左转(CCW)，0=循迹

    // 读四路红外：白底黑线，读到黑线为 0，白为 1
    int IR[4];
    IR[0] = gpio_get_level(IR1);
    IR[1] = gpio_get_level(IR2);
    IR[2] = gpio_get_level(IR3);
    IR[3] = gpio_get_level(IR4);

    // b[i] = 1 - IR[i] 表示"压线程度"（1=压黑线），total 为压线传感器个数
    int b[4];
    b[0] = 1 - IR[0];
    b[1] = 1 - IR[1];
    b[2] = 1 - IR[2];
    b[3] = 1 - IR[3];
    int total = b[0] + b[1] + b[2] + b[3];

    if (total == 0) // 全白：丢线或原地转弯扫过，保持上一轮动作，绝不停止
    {
        // 不主动停电机，电机保持上一轮 PWM：
        // 原地转弯中扫到全白会继续转；循迹中短暂丢线会继续沿原方向前进。
        last_error = 0.0f;
    }
    else // total == 1 / 2 / 3 / 4（全黑时误差为 0，等效直行）
    {
        int following = 1; // 默认本轮走循迹

        if (turn_dir != 0) // 正在原地转弯
        {
            if (IR[0] == 1 && IR[3] == 1 && (IR[1] == 0 || IR[2] == 0)) // 线回到中央（白黑白白/白白黑白/白黑黑白），退出转弯
            {
                turn_dir = 0;
                last_error = 0.0f;
            }
            else // 线尚未回到中央，继续按锁存方向原地转
            {
                following = 0;
                if (turn_dir > 0)
                    motor_turn_plus_CW(turn_speed_2);
                else
                    motor_turn_plus_CCW(turn_speed_2);
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 255)); // 青色：转弯中
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            }
        }
        else if (total == 3 && (IR[0] == 1 || IR[3] == 1)) // 仅最外侧为白才是急弯；中间为白（黑白黑黑/黑黑白黑）走循迹
        {
            if (IR[0] == 1) // 白黑黑黑，右转
                turn_dir = +1;
            else            // 黑黑黑白，左转
                turn_dir = -1;

            following = 0;
            motor_forward(base_speed); // 先前进一小段，把旋转中心对准弯道
            vTaskDelay(pdMS_TO_TICKS(turn_delay_time));
            if (turn_dir > 0)
                motor_turn_plus_CW(turn_speed_2);
            else
                motor_turn_plus_CCW(turn_speed_2);
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 255)); // 青色：急弯
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            last_error = 0.0f;
        }

        if (following) // total == 1 或 2：连续比例控制
        {
            // 四路传感器从左到右的权重，负数=左、正数=右，误差范围约 -3 ~ +3
            const float w[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
            float error = (w[0] * b[0] + w[1] * b[1] + w[2] * b[2] + w[3] * b[3]) / (float)total;

            // 死区：线基本居中时直行，避免传感器噪声引起微抖
            if (error > -deadband && error < deadband)
            {
                error = 0.0f;
            }

            // PD 控制量：P 项按偏差修正，D 项阻尼摆动
            float d = error - last_error;
            last_error = error;
            float steer = kp * error + kd * d;

            // 左轮 D / 右轮 A 差速。误差为正（线偏右）→ 左轮加速、右轮减速 → 向右修正
            // 若上车后越修越偏，把这里的正负号（或 w 数组）对调即可
            float left  = base_speed + steer; // 左轮 D
            float right = base_speed - steer; // 右轮 A
            left  = (left  < 0.0f) ? 0.0f : (left  > 1.0f ? 1.0f : left);
            right = (right < 0.0f) ? 0.0f : (right > 1.0f ? 1.0f : right);

            motorD_CCW(left); // 左轮前进方向
            motorA_CW(right); // 右轮前进方向
            motorB_stop();    // 转向轮不参与常规循迹

            // 状态指示灯：绿=居中，蓝=微调，红=强修
            int r = 0, g = 0, bl = 0;
            if (error == 0.0f)
            {
                g = 255;
            }
            else if (error > -1.5f && error < 1.5f)
            {
                bl = 255;
            }
            else
            {
                r = 255;
            }
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, bl));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(keep_time));
}

int follow_to_stop(led_strip_handle_t led_strip)
{
    // 需要调参
    float base_speed = 0.16;   // 直线基准速度（占空比 0~1）
    float kp = 0.03;           // 比例系数：误差越大，差速修正越强
    float kd = 0.02;           // 微分系数：阻尼左右摆动，防止蛇形
    float deadband = 0.5f;     // 死区：|误差|小于此值视为居中，直接直行
    float turn_speed_2 = 0.10;  // 急弯原地旋转速度
    int turn_delay_time = 100; // 急弯前冲延时（ms）
    int keep_time = 1;        // 一轮周期（ms）

    static float last_error = 0.0f; // 上一轮误差，用于微分项
    static int turn_dir = 0;        // 原地转弯方向锁存：+1=右转(CW)，-1=左转(CCW)，0=循迹

    // 读四路红外：白底黑线，读到黑线为 0，白为 1
    int IR[4];
    IR[0] = gpio_get_level(IR1);
    IR[1] = gpio_get_level(IR2);
    IR[2] = gpio_get_level(IR3);
    IR[3] = gpio_get_level(IR4);

    // b[i] = 1 - IR[i] 表示"压线程度"（1=压黑线），total 为压线传感器个数
    int b[4];
    b[0] = 1 - IR[0];
    b[1] = 1 - IR[1];
    b[2] = 1 - IR[2];
    b[3] = 1 - IR[3];
    int total = b[0] + b[1] + b[2] + b[3];

    if (total == 0) // 全白：丢线或原地转弯扫过，保持上一轮动作，绝不停止
    {
        // 不主动停电机，电机保持上一轮 PWM：
        // 原地转弯中扫到全白会继续转；循迹中短暂丢线会继续沿原方向前进。
        last_error = 0.0f;
    }
    else if (total == 4) // 全黑时退出
    {
        motor_stop();
        return 1;
    }
    else // total == 1 / 2 / 3
    {
        int following = 1; // 默认本轮走循迹

        if (turn_dir != 0) // 正在原地转弯
        {
            if (IR[0] == 1 && IR[3] == 1 && (IR[1] == 0 || IR[2] == 0)) // 线回到中央（白黑白白/白白黑白/白黑黑白），退出转弯
            {
                turn_dir = 0;
                last_error = 0.0f;
            }
            else // 线尚未回到中央，继续按锁存方向原地转
            {
                following = 0;
                if (turn_dir > 0)
                    motor_turn_plus_CW(turn_speed_2);
                else
                    motor_turn_plus_CCW(turn_speed_2);
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 255)); // 青色：转弯中
                ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            }
        }
        else if (total == 3 && (IR[0] == 1 || IR[3] == 1)) // 仅最外侧为白才是急弯；中间为白（黑白黑黑/黑黑白黑）走循迹
        {
            if (IR[0] == 1) // 白黑黑黑，右转
                turn_dir = +1;
            else            // 黑黑黑白，左转
                turn_dir = -1;

            following = 0;
            motor_forward(base_speed); // 先前进一小段，把旋转中心对准弯道
            vTaskDelay(pdMS_TO_TICKS(turn_delay_time));
            if (turn_dir > 0)
                motor_turn_plus_CW(turn_speed_2);
            else
                motor_turn_plus_CCW(turn_speed_2);
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 255)); // 青色：急弯
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
            last_error = 0.0f;
        }

        if (following) // total == 1 或 2：连续比例控制
        {
            // 四路传感器从左到右的权重，负数=左、正数=右，误差范围约 -3 ~ +3
            const float w[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
            float error = (w[0] * b[0] + w[1] * b[1] + w[2] * b[2] + w[3] * b[3]) / (float)total;

            // 死区：线基本居中时直行，避免传感器噪声引起微抖
            if (error > -deadband && error < deadband)
            {
                error = 0.0f;
            }

            // PD 控制量：P 项按偏差修正，D 项阻尼摆动
            float d = error - last_error;
            last_error = error;
            float steer = kp * error + kd * d;

            // 左轮 D / 右轮 A 差速。误差为正（线偏右）→ 左轮加速、右轮减速 → 向右修正
            // 若上车后越修越偏，把这里的正负号（或 w 数组）对调即可
            float left  = base_speed + steer; // 左轮 D
            float right = base_speed - steer; // 右轮 A
            left  = (left  < 0.0f) ? 0.0f : (left  > 1.0f ? 1.0f : left);
            right = (right < 0.0f) ? 0.0f : (right > 1.0f ? 1.0f : right);

            motorD_CCW(left); // 左轮前进方向
            motorA_CW(right); // 右轮前进方向
            motorB_stop();    // 转向轮不参与常规循迹

            // 状态指示灯：绿=居中，蓝=微调，红=强修
            int r = 0, g = 0, bl = 0;
            if (error == 0.0f)
            {
                g = 255;
            }
            else if (error > -1.5f && error < 1.5f)
            {
                bl = 255;
            }
            else
            {
                r = 255;
            }
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, bl));
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }
    }

    vTaskDelay(pdMS_TO_TICKS(keep_time));
    return 0;
}