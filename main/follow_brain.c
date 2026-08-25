#include "follow_brain.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"
#include "driver/gpio.h"

#define COEFFICIENT 0.1;

// ================================================================
// 循迹逻辑：
//   自动调取红外传感器思路数据，根据高低电平判断路况得出一个-1到1的速度参数speed_coe,并发出转向指令
// ================================================================
//传感器扫描到黑块时输出低电平，扫描到白块输出高电平，假定为白底黑线

void ir_init(void)
{
    // 把四路红外传感器引脚配置为输入，才能用 gpio_get_level 读到电平
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << IR1) | (1ULL << IR2)
                      | (1ULL << IR3) | (1ULL << IR4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,      // 红外模块多为开漏输出，需上拉；若模块自带推挽输出可改为 0
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}



void follow(int tik, float forward_speed)
{
    static int turn_flag = 0; //转向信号，决定这次follow是转向还是修正
    int IR[4];
    IR[0] = 
    int IR_1 = gpio_get_level(IR1); 
    int IR_2 = gpio_get_level(IR2);
    int IR_3 = gpio_get_level(IR3);
    int IR_4 = gpio_get_level(IR4);
    int line_color = 1; //白底黑线，否则赋-1
    int line_forward = (IR_1+IR_2-IR_3-IR_4) * line_color;
    float speed_coe = line_forward * COEFFICIENT;
    if(speed_coe != 0){
        motor_stop();
        motor_turn(speed_coe);
        vTaskDelay(pdMS_TO_TICKS(500));
        motor_stop();
        motor_forward(forward_speed);
    }
    vTaskDelay(pdMS_TO_TICKS(500));
}

