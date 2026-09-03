#include "follow_brain.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"
#include "driver/gpio.h"
#include "led_strip.h"

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
    static int circle_count = 0;
    // 需要调参
    float base_speed = 0.15;   // 直线基准速度（占空比 0~1）
    float kp = 0.015;           // 比例系数：误差越大，差速修正越强
    float kd = 0.01;           // 微分系数：阻尼左右摆动，防止蛇形
    float deadband = 0.5f;     // 死区：|误差|小于此值视为居中，直接直行
    float turn_speed_2 = 0.07;  // 急弯原地旋转速度
    int turn_delay_time = 75; // 急弯前冲延时（ms）
    int lost_delay = 0;       // 丢线后先直行找线的时间（ms）
    int keep_time = 20;        // 一轮周期（ms）

    static float last_error = 0.0f; // 上一轮误差，用于微分项
    static int turn_dir = 0;        // 原地转弯方向锁存：+1=右转(CW)，-1=左转(CCW)，0=循迹
    static int last_dir = 0;        // 丢线前的修正方向：+1=右，-1=左，0=未知
    static uint32_t lost_time = 0;  // 丢线起始 tick，0=未丢线

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
        if (turn_dir != 0)
        {
            // 原地转弯中扫到全白：不主动停电机，保持上一轮 PWM 继续转
        }
        else if (last_dir != 0)
        {
            // 循迹中丢线：先直行 lost_delay 时间（小缺口/轻微甩线会直接重新压线），
            // 仍未找回线再按丢线前最后修正方向原地旋转找线
            if (lost_time == 0)
                lost_time = xTaskGetTickCount(); // 刚丢线，记下起始时刻
            if (xTaskGetTickCount() - lost_time < pdMS_TO_TICKS(lost_delay))
            {
                motor_forward(base_speed); // 延迟窗口内直行
            }
            else
            {
                if (circle_count == 0)
                {
                    last_dir = +1;
                }
                // if (circle_count == 1)
                // {
                //     last_dir = -1;
                // }
                if (last_dir > 0)
                    motor_turn_plus_CW(turn_speed_2);
                else
                {
                    if (circle_count == 2)
                    {
                        motor_turn_plus_CCW(turn_speed_2);
                        vTaskDelay(pdMS_TO_TICKS(700));
                    }
                    else
                        motor_turn_plus_CCW(turn_speed_2);
                }
            }
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, 0, 255, 255)); // 青色：丢线搜索
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));
        }
        // 方向未知（last_dir == 0）时保持上一轮 PWM 直行
        last_error = 0.0f;
    }
    else // total == 1 / 2 / 3 / 4（全黑时误差为 0，等效直行）
    {
        lost_time = 0; // 线已出现，取消丢线直行窗口
        int following = 1; // 默认本轮走循迹

        if (turn_dir != 0) // 正在原地转弯
        {
            // 退出条件：CW（右转）必须等线压到 IR1 才停；CCW（左转）必须等线压到 IR2 才停。
            // 同时要求外侧对应边为白——锁存模式本身（白黑黑黑/黑黑黑白）就压着 IR1/IR2，
            // 不加外侧为白的守护会刚锁存就立即退出、转弯根本不执行。
            if ((turn_dir > 0 && IR[1] == 0 && IR[3] == 1) ||
                (turn_dir < 0 && IR[2] == 0 && IR[0] == 1))
            {
                turn_dir = 0;
                last_error = 0.0f;
                circle_count++;
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
        else if (total == 3) // 仅最外侧为白才是急弯；中间为白（黑白黑黑/黑黑白黑）走循迹
        {
            if (IR[0] == 1 || IR[1] == 1) // 白黑黑黑，右转
                turn_dir = +1;
            else            // 黑黑黑白，左转
                turn_dir = -1;

            following = 0;
            motor_forward(base_speed); // 先前进一小段，把旋转中心对准弯道
            vTaskDelay(pdMS_TO_TICKS(turn_delay_time));
            if (circle_count == 0)
            {
                last_dir = +1;
            }
            // if (circle_count == 1)
            // {
            //     last_dir = -1;
            // }
            
            if (turn_dir > 0)
                motor_turn_plus_CW(turn_speed_2);
            else
                {
                    if (circle_count == 2)
                    {
                        motor_turn_plus_CCW(turn_speed_2);
                        vTaskDelay(pdMS_TO_TICKS(700));
                    }
                    else
                        motor_turn_plus_CCW(turn_speed_2);
                }
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

            // 记录丢线前的修正方向（丢线搜索旋转时沿用）
            if (error > 0.0f)      last_dir = +1;
            else if (error < 0.0f) last_dir = -1;

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
    float base_speed = 0.14;   // 直线基准速度（占空比 0~1）
    float kp = 0.03;           // 比例系数：误差越大，差速修正越强
    float kd = 0.02;           // 微分系数：阻尼左右摆动，防止蛇形
    float turn_speed_2 = 0.07;  // 急弯原地旋转速度
    int turn_delay_time = 100; // 急弯前冲延时（ms）
    int lost_delay = 50;       // 丢线后先直行找线的时间（ms）
    int keep_time = 1;        // 一轮周期（ms）

    static float last_error = 0.0f; // 上一轮误差，用于微分项
    static int turn_dir = 0;        // 原地转弯方向锁存：+1=右转(CW)，-1=左转(CCW)，0=循迹
    static int last_dir = 0;        // 丢线前的修正方向：+1=右，-1=左，0=未知
    static uint32_t lost_time = 0;  // 丢线起始 tick，0=未丢线

    // 读四路红外：白底黑线，读到黑线为 0，白为 1
    int IR[4];
    IR[0] = gpio_get_level(IR1);
    IR[1] = gpio_get_level(IR2);
    IR[2] = gpio_get_level(IR3);
    IR[3] = gpio_get_level(IR4);

    // b[i] = 1 - IR[i] 表示"压线程度"（1=压黑线），total 为压线传感个数
    int b[4];
    b[0] = 1 - IR[0];
    b[1] = 1 - IR[1];
    b[2] = 1 - IR[2];
    b[3] = 1 - IR[3];
    int total = b[0] + b[1] + b[2] + b[3];

    if (total == 0) // 全白：丢线或原地转弯扫过
    {
        motor_stop();
        return 1;
    }
    else if (total == 4) // 全黑：到达停止线，停住并通知上层退出
    {
        motor_stop();
        return 1;
    }
    else // total == 1 / 2 / 3 / 4（全黑时误差为 0，等效直行）
    {
        lost_time = 0; // 线已出现，取消丢线直行窗口
        int following = 1; // 默认本轮走循迹

        if (turn_dir != 0) // 正在原地转弯
        {
            // 退出条件：CW（右转）必须等线压到 IR1 才停；CCW（左转）必须等线压到 IR2 才停。
            // 同时要求外侧对应边为白——锁存模式本身（白黑黑黑/黑黑黑白）就压着 IR1/IR2，
            // 不加外侧为白的守护会刚锁存就立即退出、转弯根本不执行。
            if ((turn_dir > 0 && IR[1] == 0 && IR[3] == 1) ||
                (turn_dir < 0 && IR[2] == 0 && IR[0] == 1))
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
            if (IR[0] == 1 || IR[2] == 1) // 白黑黑黑，黑黑白黑右转
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

        if (following) // total == 1 / 2，以及 total == 3 且白在中间（黑白黑黑/黑黑白黑）
        {
            // 四路传感器从左到右的权重，负数=左、正数=右，误差范围约 -3 ~ +3
            const float w[4] = {-3.0f, -1.0f, 1.0f, 3.0f};
            float error;
            if (total == 3)
            {
                // 三黑一白且白在中间：按白点位置给满强度修正（±3），
                // 不再把它当作"可能的噪声"降级处理，与其余误差同等对待。
                // 黑白黑黑（白在IR1，线偏右）→ +3 右转；黑黑白黑（白在IR2，线偏左）→ -3 左转
                error = (IR[1] == 1) ? 3.0f : -3.0f;
            }
            else
            {
                // 不做死区/噪声过滤：任何偏离都直接参与修正
                error = (w[0] * b[0] + w[1] * b[1] + w[2] * b[2] + w[3] * b[3]) / (float)total;
            }

            // 记录丢线前的修正方向（丢线搜索旋转时沿用）
            if (error > 0.0f)      last_dir = +1;
            else if (error < 0.0f) last_dir = -1;

            // PD 控制量：P 项按偏差修正，D 项阻尼摆动
            float d = error - last_error;
            last_error = error;
            float steer = kp * error + kd * d;

            // 差速限幅：防止单轮被压到电机死区（约0.08）以下而单轮停转、
            // 变成原地急转——急转过冲正是"左右摆"（蛇形）的主要来源。
            // 0.06 对应 base=0.14 时慢轮 0.08，刚好在死区上沿；若慢轮仍停转，
            // 把此值降回 0.05，或把 base 提到 0.15~0.16 再配更大的限幅。
            if (steer > 0.06f)       steer = 0.06f;
            else if (steer < -0.06f) steer = -0.06f;

            // 左轮 D / 右轮 A 差速。误差为正（线偏右）→ 左轮加速、右轮减速 → 向右修正
            // 若上车后越修越偏，把这里的正负号（或 w 数组）对调即可
            float left  = base_speed + steer; // 左轮 D
            float right = base_speed - steer; // 右轮 A
            left  = (left  < 0.0f) ? 0.0f : (left  > 1.0f ? 1.0f : left);
            right = (right < 0.0f) ? 0.0f : (right > 1.0f ? 1.0f : right);

            motorD_CCW(left); // 左轮前进方向
            motorA_CW(right); // 右轮前进方向
            motorB_stop();    // 转向轮不参与常规循迹

            // 状态指示灯：绿=居中，蓝=微调，红=强修（|误差|≥2.5，仅单边最外侧压线）
            int r = 0, g = 0, bl = 0;
            if (error == 0.0f)
            {
                g = 255;
            }
            else if (error > -2.5f && error < 2.5f)
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