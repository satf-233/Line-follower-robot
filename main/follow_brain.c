#include "follow_brain.h"
#include "motor.h"
#include "pins.h"
#include "driver/gpio.h"

#define COEFFICIENT 0.5;

// ================================================================
// 循迹逻辑：
//   自动调取红外传感器思路数据，根据高低电平判断路况得出一个-1到1的速度参数speed_coe,并发出转向指令
// ================================================================
//传感器扫描到黑块时输出低电平，扫描到白块输出高电平，假定为白底黑线

void follow(void)
{
    int IR_1 = gpio_get_level(IR1); 
    int IR_2 = gpio_get_level(IR2);
    int IR_3 = gpio_get_level(IR3);
    int IR_4 = gpio_get_level(IR4);
    
    int line_color = 1; //白底黑线，否则赋-1
    int line_forward = (IR_1+IR_2-IR_3-IR_4) * line_color;
    float speed_coe = line_forward * COEFFICIENT;
    motor_turn(speed_coe);
}

