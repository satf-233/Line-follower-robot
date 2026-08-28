#include "avoid.h"
#include "pins.h"
#include "motor.h"
#include "lcd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

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
}

float avoid_measure_cm(void)
{
    // 发触发脉冲：TRIG 拉高 >=10us 后拉低
    gpio_set_level(TRIG, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG, 0);

    // 等 ECHO 从低变高（发射时刻）；一直不变高则判为无回波
    while (gpio_get_level(ECHO) == 0) {
        //等待，不做处理
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

//每次停止后，延迟1000ms使电机完全停下俩
#define STOP_DELAY 1000

//避障运动主程序
bool avoid_run(){

    int findway_thre = 40; //检测到无障碍物的距离阈值(改)
    int pass_turn_time = 50; //绕行状态的每轮时间
    int findway_turn_time = 1; //找回状态中，红外传感的检测间隔
    //初始为绕行状态
    int avoid_state = 1;
    //向左平移起步点火  
    Move_Fire(1);
    lcd_show_dist(avoid_measure_cm());
    while(1){
        
        //绕行状态，每轮时间约为100ms
        if (avoid_state == 1){
            //向左平移避障
            Move(LEFT_MOVE);

            //若超声波检测得到前方没有障碍，则进入找回状态
            float dist = avoid_measure_cm();//用时1ms
            lcd_show_dist (dist);
            if (dist > findway_thre || dist < 0) {

                //先保持平移运动500ms，保持距离显示刷新率不变
                int move_count = 0;
                while (move_count < 3) {
                    vTaskDelay(pdMS_TO_TICKS(pass_turn_time));
                    lcd_show_dist (avoid_measure_cm());
                    move_count++;
                }
                move_count = 0;
                motor_stop();
                vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
                avoid_state = 2; 
            }
            vTaskDelay(pdMS_TO_TICKS(pass_turn_time));
        }

        //找回状态，该状态不会连续两次进入
        else if (avoid_state == 2){
            //大概率要先点火，然后保持，沿用上面的思路走5个循环，一次100ms
            int forward_time = 100;
            int forward_count = 0;
            while (forward_count < 12){
                nonline_forward();
                lcd_show_dist(avoid_measure_cm());
                vTaskDelay(pdMS_TO_TICKS(forward_time));
                forward_count++;
            }
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
            forward_count = 0;

            //再向右平移
            Move_Fire(RIGHT_MOVE); //100ms
            lcd_show_dist(avoid_measure_cm());
            int sense_count = 0;
            
            while(1){
                Move(RIGHT_MOVE);

                //每100ms刷新一次距离数据
                if (sense_count > 100) {
                    sense_count = 0;
                    lcd_show_dist(avoid_measure_cm()); 
                }

                //读取红外传感器电平
                int IR[4];
                IR[0] = gpio_get_level(IR1);
                IR[1] = gpio_get_level(IR2);
                IR[2] = gpio_get_level(IR3);
                IR[3] = gpio_get_level(IR4);

                //如果传感器接受到黑色信息，就返回1（算法待优化）
                if (IR[1] == 0 || IR[2] == 0 || IR[3] == 0 || IR[4] == 0){
                    motor_stop();
                    vTaskDelay(pdMS_TO_TICKS(STOP_DELAY));
                    return true;
                }
                sense_count++;
                vTaskDelay(pdMS_TO_TICKS(findway_turn_time));
            } 
        }
    }
}
