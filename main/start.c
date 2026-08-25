#include "start.h"
#include "motor.h"
#include "follow_brain.h"
#include "pins.h"
#include "driver/gpio.h"

// ================================================================
// 启动函数：启动循迹程序
//     
// ================================================================

void start(void)
{    
    motor_init();
    ir_init();   // 先初始化，follow() 才能读到电平

    double speed = 0.15;
    motor_forward(speed);
    int turns = 0;
    while (turns++ < 100)
    {
        follow();
    }
    motor_stop();
}
