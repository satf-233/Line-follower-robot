#include "image.h"
#include "camera_audio.h"
#include "motor.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"      // 板载 WS2812（GPIO38）驱动
#include "pins.h"           // LED_GPIO / LED_NUM
#include <math.h>

// ================================================================
// 摄像头循迹逻辑：
//   1. get_mask() 取一帧二值图（黑线=0，场地=255）
//   2. 在图像下半部分取若干条 ROI 行，每行找"最长连续黑段"的中心
//   3. 各行中心归一化成 [-1,1] 的偏差后加权融合，得到本帧误差 error
//      （正 = 黑线偏向画面右侧 = 车偏左了，需要往右修）
//      同时得到 curve = 远处行误差 - 近处行误差，用于弯道减速
//   4. error 经 PID 得到差速修正量 steer，叠加到基准速度上驱动 A/D 两个动力轮
//      left = base + steer（左轮 D，前进为 CCW）；right = base - steer（右轮 A，前进为 CW）
//   5. 丢线 / 横线 / 取图失败分别有对应处理，见 image_follow()
// ================================================================

static const char *TAG = "IMAGE";

/* ==================== 待调参数：调试时只改这一段 ==================== */

// —— 视觉 ROI ——
#define IMG_COL_STEP              2        // 列扫描步长，越大越省 CPU（1=逐像素）
#define IMG_BLACK_TH              128      // 灰度阈值，像素 < 此值判为黑线（mask 是 0/255）
#define IMG_SEG_MIN_RATIO         0.01f   // 黑段最小宽度（占图宽比例），更窄的当噪点丢弃
#define IMG_SEG_MAX_RATIO         0.30f    // 黑段最大宽度，更宽的判为横线/终点线（0.3=图宽30%）

// ROI 区段：在图像高度的 [TOP, BOT] 区间内，每隔 IMG_ROI_ROW_STEP 行扫一条
//   1.0 = 画面最底部（最近处），0.0 = 画面最顶部（最远处）
#define IMG_ROI_BOT_RATIO         1.0f     // 最近处行位置（画面底部，取到图像最底）
#define IMG_ROI_TOP_RATIO         0.55f    // 最远处行位置（画面顶部）
#define IMG_ROI_ROW_STEP          2        // 每隔多少行扫一条（越大越省 CPU）
// ROI 列范围：仅在画面水平方向 [LEFT, RIGHT] 区间内找线（0.0=画面最左，1.0=画面最右）
// 默认全宽；若线常跑到画面左右边缘被场地/边界干扰，可往里收窄（注意会把跑到窗口外的线判为丢线）
#define IMG_ROI_LEFT_RATIO        0.0f     // 找线区间左边界（画面最左）
#define IMG_ROI_RIGHT_RATIO       1.0f     // 找线区间右边界（画面最右）

// 扫描行权重：底部(最近处) IMG_ROI_WEIGHT_NEAR，顶部(最远处) IMG_ROI_WEIGHT_FAR，两值间线性插值
// 注：权重是对 y 的比例线性插值，随 ROI 区间 [TOP,BOT] 变化自动适配
#define IMG_ROI_WEIGHT_NEAR       4.0f
#define IMG_ROI_WEIGHT_FAR        0.5f
#define IMG_ROI_MIN_VALID_RATIO   0.05f    // 有效行数占扫描行数比例低于此值判为丢线（防单行噪点误判）
#define IMG_ERROR_IGNORE_WIDE     0        // 0=超宽行参与误差平均(沿用旧行为)；1=超宽行不参与误差平均

// 横线 / 终点线判据：仅统计行坐标落在 [CROSS_TOP, CROSS_BOT] 区间内的超宽行
#define IMG_CROSS_BOT_RATIO       1.0f     // 判据下边界（画面底部 / 最近处）
#define IMG_CROSS_TOP_RATIO       0.8f     // 判据上边界
#define IMG_CROSS_MIN_ROWS        4        // 该区间内至少几行超宽才判定为横线/终点线，防误停

// —— PID（待定，需实车整定）——
// 误差已归一化到 ±1，所以 kp 比红外版 follow() 里的 0.03 大一个量级
#define IMG_KP                    0.08f    // 比例：偏差越大，差速修正越强（直线来回摆时先降 Kp）
#define IMG_KI                    0.0f     // 积分：默认关闭，消不掉的稳态偏差再开（建议 <0.02）
#define IMG_KD                    0.0f    // 微分：阻尼左右摆动，防蛇形
#define IMG_I_MAX                 0.05f    // 积分限幅（抗饱和）
#define IMG_D_ALPHA               0.5f     // 误差低通系数(0~1]，越小越平滑，抑制视觉噪声
#define IMG_DEADBAND              0.05f    // 死区：|error| 小于此值视为居中，直接直行（放宽以压直线上抖动）
#define IMG_STEER_SIGN            (+1.0f)  // 若上车"越修越偏"，把这里改成 -1.0f

// —— 速度（待定）——
#define IMG_BASE_SPEED            0.14f    // 直线基准占空比 0~1
#define IMG_MIN_SPEED             0.10f    // 弯道减速下限，太小会推不动
#define IMG_MAX_STEER             0.10f  
  // 差速修正量上限，建议不超过 IMG_BASE_SPEED
#define IMG_K_SLOW                0.1f     // 弯道减速强度：base*(1-K_SLOW*(|error|+|curve|))

// —— 丢线 / 异常 ——
#define IMG_LOST_HOLD_MS          300      // 丢线后先保持上一轮动作的时间
#define IMG_LOST_SEARCH_MS        10000     // 原地搜索总超时，超过则停车返回 LOST
#define IMG_SEARCH_SPEED          0.07f    // 原地搜索旋转速度
#define IMG_NOFRAME_MAX           10       // 连续取图失败多少次才停车

// —— 调试 ——
#define IMG_LOG_EVERY             1       // 每多少帧打印一次调试信息（0=不打印）
#define IMG_LOOP_DELAY_MS         1        // 每轮末尾的让出时间（ms），喂看门狗用

/* ==================== 待调参数结束 ==================== */

/* ==================== 内部状态 ==================== */

static float   s_last_error   = 0.0f;   // 上一帧的原始误差（丢线时按其符号决定搜索方向）
static float   s_err_filt     = 0.0f;   // 低通后的误差（微分项用）
static float   s_integral     = 0.0f;   // 积分累计
static bool    s_pid_ready    = false;  // 是否已有上一帧数据（首帧不算微分）
static int64_t s_last_us      = 0;      // 上一帧时间戳，用于实测控制周期 dt
static int     s_noframe_cnt  = 0;      // 连续取图失败次数
static int64_t s_lost_start_us = 0;     // 丢线起始时刻（0=当前没有丢线）
static float   s_last_left    = 0.0f;   // 上一次左轮输出（仅用于调试打印）
static float   s_last_right   = 0.0f;   // 上一次右轮输出（仅用于调试打印）
static uint32_t s_frame_cnt   = 0;      // 帧计数，用于限频打印

/* ==================== 板载 LED（WS2812，GPIO38）：丢线时亮红 ==================== */
/* 用 led_strip 组件的 RMT 后端驱动 GPIO38 上的那颗 WS2812。
 * 初始化是“惰性”的：带静态标志位，无论被调用多少次都只建一次。 */
static led_strip_handle_t s_led      = NULL;
static bool              s_led_ready = false;

static void led_init(void)
{
    if (s_led_ready) return;

    led_strip_config_t strip_config = {
        .strip_gpio_num         = LED_GPIO,
        .max_leds               = LED_NUM,
        .led_model              = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
        .flags = { .invert_out = false },
    };
    led_strip_rmt_config_t rmt_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = 10 * 1000 * 1000,   // 10MHz，WS2812 时序用
        .mem_block_symbols = 0,
        .flags = { .with_dma = false },
    };

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
    if (err != ESP_OK || !s_led)
    {
        ESP_LOGE(TAG, "板载 LED 初始化失败: %s", esp_err_to_name(err));
        s_led = NULL;
        return;
    }
    s_led_ready = true;
    led_strip_clear(s_led);      // 默认熄灭
    led_strip_refresh(s_led);
    ESP_LOGI(TAG, "板载 LED 初始化完成 (GPIO%d)", LED_GPIO);
}

// 设置唯一那颗 LED 的颜色（GRB 格式），并刷新到灯珠
static void led_set_rgb(uint32_t r, uint32_t g, uint32_t b)
{
    if (!s_led_ready || !s_led) return;
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

// 熄灭
static void led_set_off(void)
{
    led_set_rgb(0, 0, 0);
}

/* ==================== 内部工具函数 ==================== */

// 限幅
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// 扫描第 y 行，取"最长连续黑段"的中心
//   center_x : 输出黑段中心的列坐标（像素）
//   too_wide : 输出该黑段是否超宽（疑似横线/终点线）
//   返回 true 表示该行找到了有效黑段
static bool scan_row_center(const BWImage *mask, int y, float *center_x, bool *too_wide)
{
    const unsigned char *row = mask->data + (size_t)y * (size_t)mask->width;
    const int w = mask->width;

    // 列范围：仅在水平方向 [LEFT, RIGHT] 内找线（0.0=画面最左，1.0=画面最右）
    int x_left  = (int)(w * IMG_ROI_LEFT_RATIO);
    int x_right = (int)(w * IMG_ROI_RIGHT_RATIO);
    if (x_left < 0)           x_left  = 0;
    if (x_left > w - 1)       x_left  = w - 1;
    if (x_right < x_left + 1) x_right = x_left + 1;
    if (x_right > w)          x_right = w;

    int best_start = -1, best_len = 0;   // 目前最长的黑段
    int cur_start  = -1, cur_len  = 0;   // 正在统计的黑段

    for (int x = x_left; x < x_right; x += IMG_COL_STEP)
    {
        if (row[x] < IMG_BLACK_TH)       // 黑线像素
        {
            if (cur_start < 0)
            {
                cur_start = x;
                cur_len   = 0;
            }
            cur_len += IMG_COL_STEP;
        }
        else                             // 场地像素，一段黑段结束
        {
            if (cur_len > best_len)
            {
                best_len   = cur_len;
                best_start = cur_start;
            }
            cur_start = -1;
            cur_len   = 0;
        }
    }
    if (cur_len > best_len)              // 收尾：黑段一直延伸到行末
    {
        best_len   = cur_len;
        best_start = cur_start;
    }

    if (best_start < 0) return false;    // 整行全白，没有线

    int min_w = (int)(w * IMG_SEG_MIN_RATIO);
    if (min_w < 1) min_w = 1;
    int max_w = (int)(w * IMG_SEG_MAX_RATIO);

    if (best_len < min_w) return false;  // 太窄，当噪点丢弃

    if (best_len > max_w && too_wide)    // 太宽，疑似横线（中心仍然可用，等效直行）
    {
        *too_wide = true;
    }

    *center_x = (float)best_start + (float)best_len * 0.5f;
    return true;
}

/* ==================== 视觉：二值图 -> 归一化误差 ==================== */

bool image_get_line_error(const BWImage *mask, float *error, float *curve, bool *cross_line)
{
    if (!mask || !mask->data || mask->width <= 0 || mask->height <= 0)
    {
        return false;
    }

    float sum_w  = 0.0f;   // 权重之和
    float sum_we = 0.0f;   // 权重×误差之和
    float e_near = 0.0f;   // 最近的一条有效行的误差
    float e_far  = 0.0f;   // 最远的一条有效行的误差
    bool  has_near = false;
    int   valid_rows = 0;  // 有效行数
    int   scan_rows  = 0;  // 实际扫描的总行数
    int   cross_rows = 0;  // 终点线判据区间内超宽的行数

    const float half_w = (float)mask->width * 0.5f;

    // ROI 区段：由近(画面底部)到远(画面顶部)遍历，步长 IMG_ROI_ROW_STEP
    int y_bot = (int)((float)mask->height * IMG_ROI_BOT_RATIO);
    int y_top = (int)((float)mask->height * IMG_ROI_TOP_RATIO);
    if (y_bot >= mask->height) y_bot = mask->height - 1;
    if (y_bot < 0)             y_bot = 0;
    if (y_top < 0)             y_top = 0;

    for (int y = y_bot; y >= y_top; y -= IMG_ROI_ROW_STEP)
    {
        scan_rows++;

        // 该行在画面高度上的位置，用于权重与终点线判据
        float ratio = (float)y / (float)mask->height;

        float cx = 0.0f;
        bool  too_wide = false;
        if (!scan_row_center(mask, y, &cx, &too_wide))
        {
            continue;   // 该行没找到线，跳过
        }

        // 终点线判据：仅统计行坐标落在 [CROSS_TOP, CROSS_BOT] 区间内的超宽行
        if (too_wide && ratio >= IMG_CROSS_TOP_RATIO && ratio <= IMG_CROSS_BOT_RATIO)
        {
            cross_rows++;
        }

        // 归一化：与分辨率无关，换分辨率不用重新调 PID
        float e = clampf((cx - half_w) / half_w, -1.0f, 1.0f);

        // 默认超宽行仍参与误差平均（保留旧行为）；IMG_ERROR_IGNORE_WIDE=1 时跳过超宽行
        if (!(IMG_ERROR_IGNORE_WIDE && too_wide))
        {
            // 权重线性插值：顶部(远处) IMG_ROI_WEIGHT_FAR → 底部(近处) IMG_ROI_WEIGHT_NEAR
            float span = IMG_ROI_BOT_RATIO - IMG_ROI_TOP_RATIO;
            float t = (span > 0.0f) ? ((ratio - IMG_ROI_TOP_RATIO) / span) : 0.0f;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            float w = IMG_ROI_WEIGHT_FAR + t * (IMG_ROI_WEIGHT_NEAR - IMG_ROI_WEIGHT_FAR);

            sum_w  += w;
            sum_we += w * e;
        }
        valid_rows++;

        if (!has_near)                 // 从近到远扫描，第一条有效行即最近处
        {
            e_near   = e;
            has_near = true;
        }
        e_far = e;                     // 循环结束后即为最远处的有效行
    }

    // 丢线判据：有效行数不足（防单行噪点误判），或所有有效行的权重和为 0
    int min_valid = (int)((float)scan_rows * IMG_ROI_MIN_VALID_RATIO);
    if (min_valid < 1) min_valid = 1;
    if (valid_rows < min_valid || sum_w <= 0.0f)
    {
        return false;                  // 丢线
    }

    if (error)      *error      = sum_we / sum_w;
    if (curve)      *curve      = e_far - e_near;
    if (cross_line) *cross_line = (cross_rows >= IMG_CROSS_MIN_ROWS);
    return true;
}

/* ==================== 控制：PID 与电机输出 ==================== */

// 复位 PID 内部状态（初始化、长时间丢线后调用）
static void pid_reset(void)
{
    s_last_error    = 0.0f;
    s_err_filt      = 0.0f;
    s_integral      = 0.0f;
    s_pid_ready     = false;
    s_last_us       = 0;
    s_lost_start_us = 0;
}

// 位置式 PID：输入归一化误差与实测周期 dt(s)，输出差速修正量 steer
static float pid_update(float error, float dt)
{
    // 死区：线基本居中时当作 0，避免视觉噪声引起微抖
    if (error > -IMG_DEADBAND && error < IMG_DEADBAND)
    {
        error = 0.0f;
    }

    // 先低通再做微分，防止图像噪声被 D 项放大
    float prev_filt = s_err_filt;
    if (!s_pid_ready)
    {
        s_err_filt = error;
        prev_filt  = error;
    }
    else
    {
        s_err_filt = IMG_D_ALPHA * error + (1.0f - IMG_D_ALPHA) * s_err_filt;
    }

    // 积分 + 限幅抗饱和
    s_integral = clampf(s_integral + error * dt, -IMG_I_MAX, IMG_I_MAX);

    // 微分（首帧或 dt 异常时不算）
    float d = 0.0f;
    if (s_pid_ready && dt > 0.0f)
    {
        d = (s_err_filt - prev_filt) / dt;
    }

    float steer = IMG_KP * error + IMG_KI * s_integral + IMG_KD * d;
    s_pid_ready = true;

    return clampf(steer * IMG_STEER_SIGN, -IMG_MAX_STEER, IMG_MAX_STEER);
}

// 差速输出：左轮 D 前进为 CCW，右轮 A 前进为 CW（与 follow_brain.c 约定一致）
static void drive(float left, float right)
{
    left  = clampf(left,  0.0f, 0.5f);
    right = clampf(right, 0.0f, 0.5f);

    s_last_left  = left;
    s_last_right = right;

    motorD_CCW(left);    // 左轮
    motorA_CW(right);    // 右轮
    motorB_stop();       // 转向轮不参与常规循迹
}

/* ==================== 对外接口 ==================== */

int image_init(void)
{
    // 仅复位循迹控制状态
    // 摄像头的挂载/初始化/启动视频流由主程序负责，这里不做任何相机操作
    pid_reset();
    s_noframe_cnt = 0;
    s_frame_cnt   = 0;
    s_last_left   = 0.0f;
    s_last_right  = 0.0f;

    led_init();      // 初始化板载 WS2812（GPIO38），丢线时用于亮红灯

    ESP_LOGI(TAG, "摄像头循迹初始化完成（相机由主程序初始化）");
    return 0;
}

int image_follow(void)
{
    int     state = IMAGE_FOLLOW_OK;
    BWImage mask  = {0};

    led_init();      // 兜底：即使没走 image_init() 也能点亮 LED（惰性，只建一次）
    float   error = 0.0f;
    float   curve = 0.0f;
    bool    cross_line = false;   // 终点/横线标志
    int64_t a = esp_timer_get_time();
    int64_t b;
    // ---------- 1. 取一帧二值图（黑线=0，场地=255）----------
    if (!mask_stream_wait_new(&mask, 200))
    {
        b = esp_timer_get_time();
        ESP_LOGE(TAG, "取图用时：%lld", b - a);
        a = b;
        free_bw_image(&mask);      // 失败时 mask 里也可能已分配，统一释放
        s_noframe_cnt++;
        if (s_noframe_cnt >= IMG_NOFRAME_MAX)
        {
            motor_stop();
            ESP_LOGE(TAG, "连续 %d 次取图失败，停车", s_noframe_cnt);
            state = IMAGE_FOLLOW_NO_FRAME;
        }
        // 偶发丢帧：不停车，电机保持上一轮 PWM
    }
    else
    {
        b = esp_timer_get_time();
        ESP_LOGE(TAG, "取图用时：%lld", b-a);
        a = b;
        s_noframe_cnt = 0;

        // ---------- 2. 视觉：算出本帧误差 ----------
        bool found = image_get_line_error(&mask, &error, &curve, &cross_line);
        b = esp_timer_get_time();
        ESP_LOGE(TAG, "计算误差用时：%lld", b-a);
        a = b;
        free_bw_image(&mask);      // mask->data 由 get_mask 内部 malloc，必须每帧释放

        // ---------- 3. 实测控制周期 dt（视觉帧率会抖，不能用固定值）----------
        int64_t now = esp_timer_get_time();
        float   dt  = 0.0f;
        if (s_last_us > 0)
        {
            dt = (float)(now - s_last_us) / 1000000.0f;
            if (dt <= 0.0f || dt > 1.0f) dt = 0.0f;   // 首帧或异常长间隔，本轮不积分/微分
        }
        s_last_us = now;

        if (!found)
        {
            // ---------- 4. 丢线：保持 -> 原地搜索 -> 超时停车 ----------
            led_set_rgb(255, 0, 0);                   // 丢线：板载 LED 亮红灯，便于调试观察

            s_integral = 0.0f;                        // 丢线期间不积分，防止饱和
            if (s_lost_start_us == 0) s_lost_start_us = now;
            int lost_ms = (int)((now - s_lost_start_us) / 1000);

            if (lost_ms < IMG_LOST_HOLD_MS)
            {
                // 短暂丢线：保持上一轮动作，等线自己回到视野，不做任何操作
            }
            else if (lost_ms < IMG_LOST_SEARCH_MS)
            {
                // 按丢线前的偏差方向原地慢转找线：误差为正=线在右侧 -> 顺时针
                if (s_last_error >= 0.0f){
                    motor_turn_plus_CW(IMG_SEARCH_SPEED);
                }
                else{
                    motor_turn_plus_CCW(IMG_SEARCH_SPEED);
                }
            }
            else
            {
                motor_stop();
                ESP_LOGE(TAG, "丢线超过 %d ms，停车", IMG_LOST_SEARCH_MS);
                state = IMAGE_FOLLOW_LOST;
            }
        }
        else
        {
            s_lost_start_us = 0;                      // 线回来了，清丢线计时
            led_set_off();                            // 线回来了：熄灭红灯

            // ---------- 6. PID + 弯道减速 + 差速输出 ----------
            s_last_error = error;                     // 记录原始误差（丢线时定搜索方向）

            float steer = pid_update(error, dt);

            float slow = 1.0f - IMG_K_SLOW * (fabsf(error) + fabsf(curve));
            float base = clampf(IMG_BASE_SPEED * slow, IMG_MIN_SPEED, IMG_BASE_SPEED);
            b = esp_timer_get_time();
            ESP_LOGE(TAG, "PID计算用时：%lld", b-a);
            a = b;
            drive(base + steer, base - steer);


            // ---------- 7. 限频打印，便于调参 ----------
            s_frame_cnt++;
#if (IMG_LOG_EVERY > 0)
            if ((s_frame_cnt % IMG_LOG_EVERY) == 0)
            {
                ESP_LOGI(TAG, "err=%.3f curve=%.3f steer=%.3f L=%.3f R=%.3f dt=%.0fms",
                         error, curve, steer, s_last_left, s_last_right, dt * 1000.0f);
            }
#endif                                     
        }
    }

    vTaskDelay(pdMS_TO_TICKS(IMG_LOOP_DELAY_MS));     // 让出 CPU，喂看门狗
    return state;
}

int image_follow_stop(void)
{
    int     state = IMAGE_FOLLOW_OK;
    BWImage mask  = {0};
    float   error = 0.0f;
    float   curve = 0.0f;
    bool    cross_line = false;   // 终点/横线标志：当前未使用（终点停车逻辑已注释）

    // ---------- 1. 取一帧二值图（黑线=0，场地=255）----------
    if (!mask_stream_wait_new(&mask, 200))
    {
        free_bw_image(&mask);      // 失败时 mask 里也可能已分配，统一释放
        s_noframe_cnt++;
        if (s_noframe_cnt >= IMG_NOFRAME_MAX)
        {
            motor_stop();
            ESP_LOGE(TAG, "连续 %d 次取图失败，停车", s_noframe_cnt);
            state = IMAGE_FOLLOW_NO_FRAME;
        }

        // 偶发丢帧：不停车，电机保持上一轮 PWM
    }
    else
    {
        s_noframe_cnt = 0;

        // ---------- 2. 视觉：算出本帧误差 ----------
        bool found = image_get_line_error(&mask, &error, &curve, &cross_line);
        free_bw_image(&mask);      // mask->data 由 get_mask 内部 malloc，必须每帧释放

        // ---------- 3. 实测控制周期 dt（视觉帧率会抖，不能用固定值）----------
        int64_t now = esp_timer_get_time();
        float   dt  = 0.0f;
        if (s_last_us > 0)
        {
            dt = (float)(now - s_last_us) / 1000000.0f;
            if (dt <= 0.0f || dt > 1.0f) dt = 0.0f;   // 首帧或异常长间隔，本轮不积分/微分
        }
        s_last_us = now;

        if (!found)
        {
            // ---------- 4. 丢线：立即停车并请求停止循迹（不再原地搜索/超时停车）----------
            led_set_rgb(255, 0, 0);
            motor_stop();
            ESP_LOGI(TAG, "检测到丢线，请求停车");
            state = IMAGE_FOLLOW_STOP;
        }
        else
        {
            led_set_off();
            s_lost_start_us = 0;                      // 线回来了，清丢线计时

            //---------- 5. 横线 / 终点线 ----------
            
            if (cross_line)
            {
                motor_stop();
                ESP_LOGI(TAG, "检测到横线/终点，请求停车");
                state = IMAGE_FOLLOW_STOP;
            }
            else
            {

            // ---------- 6. PID + 弯道减速 + 差速输出 ----------
            s_last_error = error;                     // 记录原始误差（丢线时定搜索方向）

            float steer = pid_update(error, dt);

            float slow = 1.0f - IMG_K_SLOW * (fabsf(error) + fabsf(curve));
            float base = clampf(IMG_BASE_SPEED * slow, IMG_MIN_SPEED, IMG_BASE_SPEED);

            drive(base + steer, base - steer);

            // ---------- 7. 限频打印，便于调参 ----------
            s_frame_cnt++;
#if (IMG_LOG_EVERY > 0)
            if ((s_frame_cnt % IMG_LOG_EVERY) == 0)
            {
                ESP_LOGI(TAG, "err=%.3f curve=%.3f steer=%.3f L=%.3f R=%.3f dt=%.0fms",
                         error, curve, steer, s_last_left, s_last_right, dt * 1000.0f);
            }
#endif
            }                                      
        }
    }

    vTaskDelay(pdMS_TO_TICKS(IMG_LOOP_DELAY_MS));     // 让出 CPU，喂看门狗
    return state;
}

bool image_find_line(void)
{
    BWImage mask = {0};
    float   error = 0.0f;

    led_init();   // 确保板载 WS2812 就绪（惰性，只建一次）

    // 1. 取一帧二值图（黑线=0，场地=255），按指定行区间 [143, 0) 裁剪
    if (!mask_stream_wait_new(&mask, 200))
    {
        free_bw_image(&mask);
        return false;             // 未取到图，无法判断方向
    }

    // 2. 复用 image_get_line_error 求归一化横向偏差 error（内部含丢线判断）
    bool found = image_get_line_error(&mask, &error, NULL, NULL);
    free_bw_image(&mask);        // mask->data 由 get_mask_pro 内部 malloc，必须每帧释放

    if (!found)
    {
        led_set_rgb(255, 0, 0);   // 丢线：本帧没有找到有效黑线，亮红
        return false;
    }

    // 找到线：未丢线，亮绿灯
    led_set_rgb(0, 255, 0);

    // 3. error 为归一化横向偏差 [-1,1]：
    //    >0 黑线偏向画面右侧 -> false；<0 黑线偏向画面左侧 -> true；==0 居中 -> false
    return (error < 0.07f);
}

bool image_find_ball(void){
    return true;
}