#include "avoid.h"
#include "image.h"
#include "pins.h"
#include "motor.h"
#include "lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

// ================================================================
// HC-SR04 超声波测距（TRIG / ECHO）
//   原理：TRIG 发 >=10us 高电平触发，模块发射 40kHz 超声；
//         随后 ECHO 变高（发射时刻），直到收到回波才变低（返回时刻），
//         高电平持续时长 t 为声波往返时间。
//         距离 = 声速 * t / 2，取声速 343m/s => 距离cm = t[us] / 58。
// ================================================================

// 回波接收最大量程: 30cm 
#define AVOID_TIMEOUT_US      (1740u)
// 声波往返换算系数：t[us] / 58 ≈ 距离 cm
#define AVOID_SOUND_SCALE     (58.0f)
// 盲区（约 2cm）与量程上限（约 400cm）钳位
#define AVOID_MIN_CM          (2.0f)
#define AVOID_MAX_CM          (400.0f)

// 蠕动时原地旋转速度
#define mini_turn_speed 0.14f
// 蠕动时原地旋转时间// 单位ms
#define mini_turn_time 50

void avoid_init(void)
{
    // TRIG：输出，平时保持低电平
    gpio_config_t trig = {
        .pin_bit_mask = (1ULL << TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig);
    gpio_set_level(TRIG, 0);

    // ECHO：输入，读取电平判断回波
    gpio_config_t echo = {
        .pin_bit_mask = (1ULL << ECHO),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = 0,
        .pull_down_en = 0,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo);

    ESP_LOGI("AVOID", "Avoid init success!");
}

float avoid_measure_cm(void)
{
    // 发触发脉冲：TRIG 拉高 >=10us 后拉低
    gpio_set_level(TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG, 0);

    // 等 ECHO 从低变高（发射时刻）；一直不变高则判为无回波
    // 必须加超时：前方全空 / 擦墙角等收不到回波时 ECHO 不上跳，死等会卡死整个任务
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ECHO) == 0) {
        if (esp_timer_get_time() - t0 > 50000) {   // 50ms 等不到上升沿 -> 无回波
            return AVOID_NO_ECHO_CM;               // -1.0f，见 avoid.h
        }
    }
    int64_t start = esp_timer_get_time();

    // 等 ECHO 从高变低（回波返回时刻），得到高电平持续时长
    while (gpio_get_level(ECHO) == 1) {
        if (esp_timer_get_time() - start > AVOID_TIMEOUT_US) {
            return AVOID_OUT_RANGE_CM;
        }
    }
    int64_t dur_us = esp_timer_get_time() - start;

    // 换算：往返时间 / 58 ≈ 距离 cm
    float cm = (float)dur_us / AVOID_SOUND_SCALE;

    //盲区与量程钳位，防止读数异常
    if (cm < AVOID_MIN_CM) {
        cm = AVOID_MIN_CM;
    }
    if (cm > AVOID_MAX_CM) {
        cm = AVOID_MAX_CM;
    }
    return cm;
}


//定义方向left，right对应数字
#define LEFT_MOVE 1
#define RIGHT_MOVE 2

/*                               =====平移动作车轮速度参数调试方法=====
    (1)初步调节A和D速度相同，B为A和D速度的两倍
    (2)若小车发生旋转，则固定A和D不变，调节B，使得Speed_B与Speed_A(D)的比值移动0.1到0.2，若
       顺时针旋转则减小Speed_B，逆时针旋转则增大Speed_B。
    (3)迭代直到小车完全不发生旋转时，调节A和D的速度，消去前后平移分量。
    (4)初始时给一个瞬间的大速度，让车轮先转起来，规避静摩擦力。*/
#define speed_A 0.116f
#define speed_B 0.207f
#define speed_D 0.116f
#define fire_para 1.85f
#define fire_time 100
#define REACQUIRE_LINE_TIMEOUT_MS 1500   // 右移找回线超时(ms)

//向左和向右平移函数
void Move(int type){ 
    
    //向左平移
    if (type == LEFT_MOVE){
        motorA_CW(speed_A);
        motorB_CCW(speed_B);
        motorD_CW(speed_D);
    }

    //向右平移
    else if (type == RIGHT_MOVE){
        motorA_CCW(speed_A);
        motorB_CW(speed_B);
        motorD_CCW(speed_D);
    }
    else {
        //不执行任何指令
    }
}

//点火函数，在平移之前给小车一个持续时间为fire_time的瞬间大速度
void Move_Fire(int type){

    if (type == LEFT_MOVE){
        motorA_CW(speed_A * fire_para);
        motorB_CCW(speed_B * fire_para);
        motorD_CW(speed_D * fire_para);
        vTaskDelay(pdMS_TO_TICKS(fire_time));
    }

    else if (type ==  RIGHT_MOVE){
        motorA_CCW(speed_A * fire_para);
        motorB_CW(speed_B * fire_para);
        motorD_CCW(speed_D * fire_para);
        vTaskDelay(pdMS_TO_TICKS(fire_time));
    }
    else{
        //不执行任何指令
    }
}

void nonline_forward(){
    float forspeed_A = 0.177;
    float forspeed_D = 0.16;
    motorD_CCW(forspeed_D);
    motorA_CW(forspeed_A);
    motorB_stop();
}

void mini_turn_error(float err_percent){
    if (err_percent > 0)
    {
        motor_turn_plus_CCW(mini_turn_speed);
        vTaskDelay(pdMS_TO_TICKS(mini_turn_time));
        motor_stop();
    }
    else
    {
        motor_turn_plus_CW(mini_turn_speed);
        vTaskDelay(pdMS_TO_TICKS(mini_turn_time));
        motor_stop();
    }
}

void mini_turn_direction(int dir){
    motor_turn_plus(dir, mini_turn_speed); // 原地旋转
    vTaskDelay(pdMS_TO_TICKS(mini_turn_time));
    motor_stop();
}

//每次停止后，延迟1000ms使电机完全停下俩
#define STOP_DELAY 1000
//#define findway_thre 25 //检测到无障碍物的距离阈值(改)
#define pass_turn_time 50 //绕行状态的每轮时间
#define forward_time 100  //直行时间
#define findway_turn_time 1 //找回状态中，红外传感的检测间隔



//避障运动主程序
bool avoid_run(){

    //初始为绕行状态
    int avoid_state = 1;

    //向左平移起步点火  
    Move_Fire(LEFT_MOVE);
    Move(LEFT_MOVE);
    // lcd_show_dist(avoid_measure_cm());
    float dist = 0;
    while(avoid_state == 1)
    {
        //绕行状态，每轮时间约为100ms
        //若超声波检测得到前方没有障碍，则进入找回状态
        vTaskDelay(pdMS_TO_TICKS(pass_turn_time));
        dist = avoid_measure_cm();//用时1ms
        // lcd_show_dist (dist);
        if (dist < 0) 
        {
            //先保持平移运动500ms，保持距离显示刷新率不变
            avoid_state = 2;
            vTaskDelay(pdMS_TO_TICKS(pass_turn_time));
            lcd_show_dist (avoid_measure_cm());
            //motor_stop();
            //vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
        }  
    }
    //直行段
    //大概率要先点火，然后保持，沿用上面的思路走5个循环，一次100ms
    int forward_count = 0;
    nonline_forward();
    while (forward_count < 15)
    {
        // lcd_show_dist(avoid_measure_cm());
        vTaskDelay(pdMS_TO_TICKS(forward_time));
        forward_count++;
    }
    //motor_stop();
    //vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));

    //再向右平移
    Move_Fire(RIGHT_MOVE); //100ms
    // lcd_show_dist(avoid_measure_cm());
    int sense_count = 0;
    Move(RIGHT_MOVE);
    while(1){
        vTaskDelay(pdMS_TO_TICKS(findway_turn_time));
        //每100ms刷新一次距离数据
        if (sense_count > 100) {
            sense_count = 0;
            // lcd_show_dist(avoid_measure_cm()); 
        }sense_count++;
        //读取红外传感器电平
        //如果传感器接受到黑色信息，就返回1（算法待优化）
        // if (!(gpio_get_level(IR1)&gpio_get_level(IR2)&gpio_get_level(IR3)&gpio_get_level(IR4))){
            //motor_stop();
            //vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
            return true;
        // }
    }
}

// 摄像头循线特供避障
bool avoid_run_plus(){
    // 第一步变为调整姿态，因为摄像头循线不易达成完全沿路，容易在进入avoid_run_plus时小车朝向不对
    ESP_LOGI("AVOID", "Start avoid1!");
    float dist = avoid_measure_cm();
    float last_dist = dist;
    int dir = 1; // 旋转方向，默认先顺时针
    int wrong_count = 0; // 换方向次数
    ESP_LOGI("AVOID", "Start avoid2!");
    while (wrong_count < 5)
    {
        ESP_LOGI("AVOID", "Avoiding");
        mini_turn_direction(dir);
        vTaskDelay(pdMS_TO_TICKS(100));
        dist = avoid_measure_cm();
        vTaskDelay(pdMS_TO_TICKS(100));
        if (dist < 0) // 无回波/超量程：这个朝向没"看到"障碍，视为方向错误，换方向
        {
            last_dist = dist;
            dir = 0 - dir;
            wrong_count++;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (last_dist < 0)
        {
            // 恢复真实读数后的第一轮：上一轮无回波把比较基准污染成了负数，
            // 任何正数跟 -1 比都会被判成"变远"而误翻转（"距离变短仍换方向"的根因），
            // 因此只重建基准，不做方向判断，让小车继续沿当前方向转
            last_dist = dist;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        if (dist < last_dist) // 旋转后dist变小，说明方向正确，需要继续朝这个方向旋转
        {
            last_dist = dist;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        else // 旋转后dist变大，说明方向错误，需要换方向
        {
            last_dist = dist;
            dir = 0 - dir;
            wrong_count++;
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
    }
    
    //初始为绕行状态
    int avoid_state = 1;

    //向左平移起步点火  
    Move_Fire(LEFT_MOVE);
    Move(LEFT_MOVE);
    // lcd_show_dist(avoid_measure_cm());

    while(avoid_state == 1)
    {
        //绕行状态，每轮时间约为100ms
        //若超声波检测得到前方没有障碍，则进入找回状态
        vTaskDelay(pdMS_TO_TICKS(pass_turn_time));
        dist = avoid_measure_cm();//用时1ms
        // lcd_show_dist (dist);
        if (dist < 0) 
        {
            //先保持平移运动500ms，保持距离显示刷新率不变
            avoid_state = 2;
            vTaskDelay(pdMS_TO_TICKS(pass_turn_time));
            // lcd_show_dist (avoid_measure_cm());
            //motor_stop();
            //vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
        }  
    }
    //直行段
    //大概率要先点火，然后保持，沿用上面的思路走5个循环，一次100ms
    int forward_count = 0;
    nonline_forward();
    while (forward_count < 15)
    {
        // lcd_show_dist(avoid_measure_cm());
        vTaskDelay(pdMS_TO_TICKS(forward_time));
        forward_count++;
    }
    //motor_stop();
    //vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));

    //再向右平移
    Move_Fire(RIGHT_MOVE); //100ms
    // lcd_show_dist(avoid_measure_cm());
    // int sense_count = 0;
    Move(RIGHT_MOVE);
    int64_t reacquire_start_us = esp_timer_get_time();
    while (1)
    {
        if (image_find_line())
        {
            motor_stop();
            return true;
        }

        // 兜底：右移找回线超过 1500ms 仍未回到线中央，强停避免无限右移冲线
        if ((esp_timer_get_time() - reacquire_start_us) > ((int64_t)REACQUIRE_LINE_TIMEOUT_MS * 1000))
        {
            motor_stop();
            ESP_LOGW("AVOID", "Reacquire line timeout, abort");
            return false;
        }
    }
    // while(1){
    //     vTaskDelay(pdMS_TO_TICKS(findway_turn_time));
    //     //每100ms刷新一次距离数据
    //     if (sense_count > 100) {
    //         sense_count = 0;
    //         lcd_show_dist(avoid_measure_cm()); 
    //     }sense_count++;
    //     //读取红外传感器电平
    //     //如果传感器接受到黑色信息，就返回1（算法待优化）
    //     if (!(gpio_get_level(IR1)&gpio_get_level(IR2)&gpio_get_level(IR3)&gpio_get_level(IR4))){
    //         //motor_stop();
    //         //vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
    //         return true;
    //     }
    // }
}