#include "ball.h"
#include "lcd.h"
#include "mask_stream.h"
#include "esp_timer.h"
// ================================================================
// 粗对准：根据球 mask（球=0/黑，背景=255/白）判断球相对画面竖直中线的
// 位置，并驱动电机原地自转，使球心逼近中线。
//   返回  1 ：球已对正（球心落在中线死区内），已停车
//   返回  0 ：尚未对正，已按方向原地自转
//   返回 -1 ：本帧没看到球（球像素过少 / mask 非法）
// ================================================================

// 原地自转速度（占空比 0~1）
#define BALL_ROU_AIM_SPEED      0.08f
// 粗居中死区：球心偏离中线超过该比例(占半宽)才转，避免来回抖
#define BALL_ROU_AIM_DEADBAND   0.08f
// 细居中死区：球心偏离中线超过该比例(占半宽)才转
#define BALL_EXA_AIM_DEADBAND   0.04f
// 判球阈值：mask 中球=0，背景=255，低于此值视为球
#define BALL_MASK_TH        128
//绕球半径
#define BALL_DISTANCE 15
//半径允许偏差
#define BALL_DIS_ERR 1
//靠近速度
#define BALL_APPROACH_SPEED 0.14
//转圈速度
#define CIRCLE_SPEED 0.16
//径向调整速度
#define FOE_BACK_SPEED 0.16
//推球前进时按终点色块偏差做差速修正的比例系数（kp = err * PUSH_KP）-------------------不能太大
#define PUSH_KP         0.12f
//推球前进修正死区：|err| 小于此值不做差速，避免左右摆-----------------------------------不能太大
#define PUSH_DEADBAND   0.04f
//认为是属于终点的最短行长度
#define DETECT_LENTH_BOUNDRY 8  //8
//认为终点在画面中的最短行数
#define DETECT_ROW_BOUNDRY 3  //3

//约定绕圈距离对应的像素行位置
#define COL_LOCATION 100
//判定小球对正的死区
#define BALL_AIM_DEADBAND 0.04f
//判定小球距离合适的距离
#define BALL_DISTANCE_DEADBAND 0.04f
//判定终点对正的死区
#define FINAL_AIM_DEADBAND 0.02f
//微调时间
#define ADTIME 70
// 原地自转抖动速度（占空比 0~1）
#define BALL_MINI_AIM_SPEED      0.1f
//前进微调速度
#define BALL_ADAP_SPEED 0.16


// 从球 mask 求球心相对画面竖直中线的归一化偏差
//   error 输出：正=球偏右，负=球偏左，范围约 [-1,1]
//   返回 true 表示找到球（error 有效），false 表示没看到球
static bool ball_get_error(const BWImage *mask, float *error)
{

    if (!mask || !mask->data || mask->width <= 0 || mask->height <= 0)
    {
        return false;
    }

    const int w = mask->width;
    const int h = mask->height;

    // 统计球像素的质心（所有黑像素的 x 坐标均值，即球心水平位置）
    long long sum_x = 0;
    int count = 0;
    for (int y = 0; y < h; y++)
    {
        const unsigned char *row = mask->data + (size_t)y * w;
        for (int x = 0; x < w; x++)
        {
            if (row[x] < BALL_MASK_TH)
            {
                sum_x += x;
                count++;
            }
        }
    }

    // 球像素太少视为没看到球（MIN_PIXELS 沿用 camera_audio.h 的约定）
    if (count < MIN_PIXELS)
    {
        return false;
    }

    const float half_w = (float)w * 0.5f;
    const float cx     = (float)sum_x / (float)count;

    *error = (cx - half_w) / half_w;
    return true;
}

static int roughly_aim(BWImage* mask, int mode)
{
    (void)mode;   // 取图模式已由 mask_stream_set_mode 统一管理
    float err = 0.0f;
    if (!ball_get_error(mask, &err))
    {
        if(mode == REDBALL_MODE)//红球顺时针，蓝秋逆时针
        motor_turn_plus_CW(BALL_ROU_AIM_SPEED);
        else
        motor_turn_plus_CCW(BALL_ROU_AIM_SPEED);
        return -1;   // 本帧没看到球
    }

    if (err > BALL_ROU_AIM_DEADBAND)
    {
        // 球在画面右侧：顺时针自转，把车头向右转正对球
        // 若上车后发现转向相反，把 CW/CCW 两个分支对调即可
        int count = 0;
        motor_turn_plus_CW(BALL_ROU_AIM_SPEED);
        while(err > BALL_ROU_AIM_DEADBAND)
        {   
            if(!mask_stream_wait_new(mask, 200) || !ball_get_error(mask, &err))count++;
            else count = 0;
            if (count > 3)return -1;
        }
        motor_stop();
        return 0;
    }
    else if (err < -BALL_ROU_AIM_DEADBAND)
    {
        // 球在画面左侧：逆时针自转
        int count = 0;
        motor_turn_plus_CCW(BALL_ROU_AIM_SPEED);
        while(err < -BALL_ROU_AIM_DEADBAND)
        {   
            if(!mask_stream_wait_new(mask, 200) || !ball_get_error(mask, &err))count++;
            else count = 0;
            if (count > 3)return -1;
        }
        motor_stop();
        return 0;
    }
    // 已对正，停车
    motor_stop();
    return 1;
}

static int exactly_aim(BWImage* mask, int mode)
{
    (void)mode;   // 取图模式已由 mask_stream_set_mode 统一管理
    float err = -1;
    if(!mask_stream_wait_new(mask, 200)) return -1;

    int count = 0;
    float err_0;
    do
    {

        if(ball_get_error(mask, &err))
        {
            count = 0;
        }
        else count++;
        if (count > 3)
        {
            return -1;
        }
        //lcd_show_error(err);
        err_0 = err > 0? err : -err;
        mini_turn_error(err);
        vTaskDelay(pdMS_TO_TICKS(50));
        if(!mask_stream_wait_new(mask, 200)) return -1;
    }while (err_0 > BALL_EXA_AIM_DEADBAND);
    return 1;
}


float motor_circle(float dist, float error)//默认顺时针转
{
    float meas = avoid_measure_cm();

    lcd_show_dist(meas);

    if (meas < 0.0)
    {
        motor_stop();
        return meas;
    }
    else if((meas - dist) > error)//偏远，需要靠近
    {
        motorB_CCW(CIRCLE_SPEED);
        motor_forward_new(FOE_BACK_SPEED);
        vTaskDelay(pdMS_TO_TICKS(50));
        motor_forward_new(0.0);
        return meas;
    }
    else if((meas - dist) < -error)//偏近，需要远离
    {
        motorB_CCW(CIRCLE_SPEED);
        motor_backward_new(FOE_BACK_SPEED);
        vTaskDelay(pdMS_TO_TICKS(50));
        motor_backward_new(0.0);
        return meas;
    }
    else 
    {
        motorB_CCW(CIRCLE_SPEED);
        return meas;
    }
}




//先自转找球，知道球过中线，然后小转微调，使用超声波验证后退出
void aim(BWImage* mask, int mode)
{
    int dist;
    float err;
    seek_and_approach:
    mask_stream_set_mode(mode);   // 每次回到起点都先切回球取图模式
    if(!mask_stream_wait_new(mask, 200)) 
    {
        ESP_LOGW(TAG_BALL, "球取图失败，回退");
        goto seek_and_approach;
    }
    ESP_LOGI(TAG_BALL, "开始粗调");
    if(roughly_aim(mask, mode) == -1)//粗调中丢球
    {
        ESP_LOGW(TAG_BALL, "粗调丢球，回退");
        goto seek_and_approach;
    }
    ESP_LOGI(TAG_BALL, "开始细调");
    if(exactly_aim(mask, mode) == -1)//精调中丢球
    {
        ESP_LOGW(TAG_BALL, "细调丢球，回退");
        goto seek_and_approach;
    }
    ESP_LOGI(TAG_BALL, "开始靠近");

    uint64_t t0 = esp_timer_get_time();

    dist = avoid_measure_cm();
    lcd_show_dist(dist);
    while (dist > BALL_DISTANCE)
    {
        motor_forward_new(BALL_APPROACH_SPEED);
        //vTaskDelay(pdMS_TO_TICKS(100));
        dist = avoid_measure_cm();
        lcd_show_dist(dist);
    }

    motor_stop();
    if(dist < 0 )//靠近中丢球
    {
        ESP_LOGW(TAG_BALL, "靠近丢球，回退");
        uint64_t t1 = esp_timer_get_time();
        ESP_LOGI(TAG_BALL, "直行时间：%lld ms",(long long)(t1 - t0) / 1000);
        goto seek_and_approach;
    }
    ESP_LOGI(TAG_BALL, "开始转圈");
    err = 1.1;
    mask_stream_set_mode(LINE_MODE2);   // 转圈阶段改用线模式
    if(!mask_stream_wait_new(mask, 200)) 
        {
            ESP_LOGW(TAG_BALL, "终点取图失败，回退");
            goto seek_and_approach;
        }
    ball_get_error(mask, &err);

    float err0;
    do
    {
        if(motor_circle(BALL_DISTANCE, BALL_DIS_ERR) < 0)//允许偏差1厘米的球距离，可能也需要微调版本ccx
        {
            ESP_LOGW(TAG_BALL, "转圈丢球，回退");
            goto seek_and_approach;//转圈中丢球
        }
        if(!mask_stream_wait_new(mask, 200))
        {
            ESP_LOGW(TAG_BALL, "终点取图失败，回退");
            goto seek_and_approach;
        }

        ball_get_error(mask, &err);
        lcd_show_error(err);
        err0 = err > 0 ? err : -err;
    } while (err0 > 0.02);
    ESP_LOGI(TAG_BALL, "三点一线成功");
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));
}

// 判断画面中是否出现终点色块：
//   扫描所有行，统计「单行内存在 >= DETECT_LENTH_BOUNDRY 个连续目标色像素」的行数，
//   若该行数 > DETECT_ROW_BOUNDRY，则认为终点色块已在画面中。
//   目标色在 mask 中为黑(0)，背景为白(255)，阈值取 128（与 BALL_MASK_TH 一致）。
//   返回 true=检测到终点色块，false=未检测到
static bool see_final_if(BWImage* mask)
{
    if (!mask || !mask->data || mask->width <= 0 || mask->height <= 0)
    {
        return false;
    }

    const int w = mask->width;

    int hit_rows = 0;   // 满足「单行存在长连续段」的行数

    for (int h = 0; h < mask->height; h++)
    {
        const unsigned char *row = mask->data + (size_t)h * w;

        int  run = 0;    // 当前连续目标色像素数
        bool hit = false;
        for (int x = 0; x < w; x++)
        {
            if (row[x] < 128)          // 目标色像素
            {
                run++;
                if (run >= DETECT_LENTH_BOUNDRY)
                {
                    hit = true;
                    break;
                }
            }
            else
            {
                run = 0;
            }
        }

        if (hit)
        {
            hit_rows++;
        }
    }

    return hit_rows > DETECT_ROW_BOUNDRY;
}

static void fire_forward()
{
motor_forward(BALL_APPROACH_SPEED * 1.85);
vTaskDelay(pdMS_TO_TICKS(100));
}

void push(BWImage* mask)
{
    bool fire_flag = false;
    mask_stream_set_mode(LINE_MODE2);
    bool arrive = false;
    float err = 0.0f;
    do
    {
        // 用终点色块偏差做差速修正：终点偏右则右转、偏左则左转，对正或没看到终点则直行
        if (mask_stream_wait_new(mask, 200)) {
            arrive = !(see_final_if(mask));   // 拿到新帧才判断；超时则跳过，避免拿旧图重复判断
        }
        else
        {continue;}
        if (ball_get_error(mask, &err))
        {
            lcd_show_tri_error(err, 3);
            if (err > PUSH_DEADBAND)
            {
                motor_turn_right(BALL_APPROACH_SPEED, err * PUSH_KP);   // 终点偏右，向右修正
            }
            else if (err < -PUSH_DEADBAND)
            {
                motor_turn_left(BALL_APPROACH_SPEED, -err * PUSH_KP);   // 终点偏左，向左修正
            }
            else
            {
                if(fire_flag == false)
                {
                    fire_forward();
                    fire_flag = true;
                }
                motor_forward_new(BALL_APPROACH_SPEED);   // 已对正，直行
            }
        }
        else
        {
            motor_forward_new(BALL_APPROACH_SPEED);   // 本帧没看到终点，先直行
        }

        
    }while(!arrive);
    motor_stop();
    ESP_LOGI(TAG_BALL, "推球入洞成功");
    vTaskDelay(pdMS_TO_TICKS(1000));
}
//只有后退一段时间，重复上述过程


static void adjust_dis(float ball_error_x, float ball_error_y)//调整一次相对于小球的位置
{
        if(ball_error_x > BALL_AIM_DEADBAND)//小球偏右
    {
        if(ball_error_y > BALL_DISTANCE_DEADBAND)//距离太近
        {
            motor_turn_plus_CW(BALL_MINI_AIM_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
            motor_backward_new(BALL_ADAP_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();

        }
        else if(ball_error_y < -BALL_DISTANCE_DEADBAND)
        {
            motor_turn_plus_CW(BALL_MINI_AIM_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
            motor_forward_new(BALL_ADAP_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
        else
        {
            motor_turn_plus_CW(BALL_MINI_AIM_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
    }
    else if(ball_error_x < -BALL_AIM_DEADBAND)//小球偏左
    {
        if(ball_error_y > BALL_DISTANCE_DEADBAND)//距离太近
        {
            motor_turn_plus_CCW(BALL_MINI_AIM_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
            motor_backward_new(BALL_ADAP_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
        else if(ball_error_y < -BALL_DISTANCE_DEADBAND)
        {
            motor_turn_plus_CCW(BALL_MINI_AIM_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
            motor_forward_new(BALL_ADAP_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
        else
        {
            motor_turn_plus_CCW(BALL_MINI_AIM_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
    }
    else //小球正对
    {
        if(ball_error_y > BALL_DISTANCE_DEADBAND)
        {
            motor_backward_new(BALL_ADAP_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
        else if(ball_error_y < -BALL_DISTANCE_DEADBAND)
        {
            motor_forward_new(BALL_ADAP_SPEED);
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(ADTIME));
            motor_stop();
        }
    }
}
// 从球 mask 同时求横向与纵向偏差：
//   row_error：横向偏差，与 ball_get_error 的 error 完全一致（正=球偏右、负=球偏左）
//   col_error：纵向偏差，球质心纵坐标相对 COL_LOCATION 的标准化偏差
static bool ball_get_error_cam(const BWImage *mask, float *row_error, float *col_error)
{
    if (!mask || !mask->data || mask->width <= 0 || mask->height <= 0)
    {
        return false;
    }

    const int w = mask->width;
    const int h = mask->height;

    long long sum_x = 0, sum_y = 0;
    int count = 0;
    for (int y = 0; y < h; y++)
    {
        const unsigned char *row = mask->data + (size_t)y * w;
        for (int x = 0; x < w; x++)
        {
            if (row[x] < BALL_MASK_TH)
            {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    if (count < MIN_PIXELS)
    {
        return false;
    }

    const float half_w = (float)w * 0.5f;
    const float half_h = (float)h * 0.5f;
    const float cx     = (float)sum_x / (float)count;
    const float cy     = (float)sum_y / (float)count;

    if (row_error) *row_error = (cx - half_w) / half_w;
    if (col_error) *col_error = (cy - COL_LOCATION) / half_h;
    return true;
}


static bool keep_ball(BWImage* mask, int mode)//修正相对于小球的姿态
{
    float ball_error_x = 1;//球左侧为负，要左动
    float ball_error_y = 1;//球远离为负，要靠近
    float ball_error_x0 = 1;//绝对值
    float ball_error_y0 = 1;
    mask_stream_set_mode(mode);
    do
    {
        if(!mask_stream_wait_new(mask, 200)) 
        {
            ESP_LOGW(TAG_BALL, "转圈中球取图失败，重复");
            continue;
        }
        if(!ball_get_error_cam(mask, &ball_error_x, &ball_error_y))
        {
            ESP_LOGE(TAG_BALL, "转圈丢球");
            return false;
        }


        lcd_show_tri_error(ball_error_x, 1);
        lcd_show_tri_error(ball_error_y, 2);

        adjust_dis(ball_error_x, ball_error_y);//调整相对于小球的位置，旋转对正与前后退，直到对正
        ball_error_x0 = ball_error_x > 0? ball_error_x : -ball_error_x;
        ball_error_y0 = ball_error_y > 0? ball_error_y : -ball_error_y;
    }
    while(ball_error_x0 > BALL_AIM_DEADBAND || ball_error_y0 > BALL_DISTANCE_DEADBAND);
    return true;
}


static int motor_circle_cam(BWImage* mask, int mode)
{
    float final_error;
    do
    {
        if(!keep_ball(mask, mode))return -1;//修正丢球，重跑
        motor_stop();
        mask_stream_set_mode(LINE_MODE2);
        if(!mask_stream_wait_new(mask, 200)) 
        {
            ESP_LOGW(TAG_BALL, "转圈中终点取图失败，重复");
            continue;
        }//取一帧球位置
        if(!ball_get_error(mask, &final_error))
        {
            ESP_LOGE(TAG_BALL, "未看到终点");
            final_error = 0.999;
            lcd_show_tri_error(final_error, 3);
            if(mode == REDBALL_MODE)//红球顺时针转动
            {
                motorB_CCW(CIRCLE_SPEED);
                vTaskDelay(pdMS_TO_TICKS(ADTIME+30));//之前是70
                motor_stop();
            }
            else//蓝球逆时针动
            {
                motorB_CW(CIRCLE_SPEED);
                vTaskDelay(pdMS_TO_TICKS(ADTIME+30));//之前是70
                motor_stop();
            }
        }
        else //取得有效final_error
        {
            
            lcd_show_tri_error(final_error, 3);

            if(final_error > FINAL_AIM_DEADBAND)//终点偏右
            {
                motorB_CCW(CIRCLE_SPEED);
                vTaskDelay(pdMS_TO_TICKS(ADTIME+30));
                motor_stop();
            }
            else if(final_error < - FINAL_AIM_DEADBAND)//终点偏左
            {
                motorB_CW(CIRCLE_SPEED);
                vTaskDelay(pdMS_TO_TICKS(ADTIME+30));
                motor_stop();
            }
            else//终点正对
            {
                return 0;
            }
        }
    }while(1);
}



//先自转找球，知道球过中线，然后小转微调，使用超声波验证后退出
void aim_cam(BWImage* mask, int mode)
{
    seek_and_approach:
    mask_stream_set_mode(mode);   // 每次回到起点都先切回球取图模式
    if(!mask_stream_wait_new(mask, 200)) 
    {
        ESP_LOGW(TAG_BALL, "球取图失败，回退");
        goto seek_and_approach;
    }
    ESP_LOGI(TAG_BALL, "开始粗调");
    if(roughly_aim(mask, mode) == -1)//粗调中丢球
    {
        ESP_LOGW(TAG_BALL, "粗调丢球，回退");
        goto seek_and_approach;
    }
    ESP_LOGI(TAG_BALL, "开始细调");
    if(exactly_aim(mask, mode) == -1)//精调中丢球
    {
        ESP_LOGW(TAG_BALL, "细调丢球，回退");
        goto seek_and_approach;
    }

    motor_stop();
    ESP_LOGI(TAG_BALL, "开始转圈，尝试将小球与洞对齐");

    if(motor_circle_cam(mask, mode) == -1)
    {
        goto seek_and_approach;
    }

    ESP_LOGI(TAG_BALL, "三点一线成功");
    motor_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

}

