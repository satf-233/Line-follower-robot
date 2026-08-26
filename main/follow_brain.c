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

void follow()
{
    //需要调参
    float for_speed = 0.1;         //前进速度
    float turn_speed = 0.1;        //转向速度，修正和转弯共用
    int keep_time = 200;            //一轮时间
    int turn_delay_time = 200;      //转弯延迟时间

    int turn_state = 0; //转向状态，决定这次follow是转向还是直行
    int IR[4];
    IR[0] = gpio_get_level(IR1);
    IR[1] = gpio_get_level(IR2);
    IR[2] = gpio_get_level(IR3);
    IR[3] = gpio_get_level(IR4);
    int count = IR[0] + IR[1] + IR[2] + IR[3];
    if (count == 4 || count == 0)
    {
        //keep(turn_state, speed);
    }
    else if(count == 3)
    {
        if(IR[0] == 0)//黑白白白，左修正
        {
            motor_turn_left(turn_speed);
        }
        else if(IR[3] == 0)//白白白黑，右修正
        {
            motor_turn_right(turn_speed);
        }
        else
        {
            motor_forward(for_speed);
        }
    }
    else if(count == 2)
    {
        int cnd = IR[0] + IR[1]*2 + IR[2]*4 + IR[3]*8; 
        switch (cnd)
        {
        case 3://白白黑黑。右修正
            motor_turn_right(turn_speed);
            break;
        case 6://白黑黑白，直行
            motor_forward(for_speed);
            break;
        case 12://黑黑白白，左修正
            motor_turn_left(turn_speed);
            break;
        default:
            //keep(turn_state, speed);
            break;
        }
    }
    else//转弯情形
    {
        if(IR[0] == 1)//白黑黑黑，右转
        {
            vTaskDelay(pdMS_TO_TICKS(turn_delay_time));
            motor_turn_plus_CW(turn_speed);
        }
        else if(IR[3] == 1)//黑黑黑白，左转
        {
            vTaskDelay(pdMS_TO_TICKS(turn_delay_time));
            motor_turn_plus_CCW(turn_speed);
        }
        else
        {
            //keep(turn_state, speed);
        }
    }
    vTaskDelay(pdMS_TO_TICKS(keep_time));
}

