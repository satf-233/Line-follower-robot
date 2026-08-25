#include "start.h"
#include "motor.h"
#include "follow_brain.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pins.h"
#include "driver/gpio.h"

// ================================================================
// 启动函数：启动循迹程序
//     
// ================================================================

void start(void)
{
    motor_forward(0.5);
    int turns = 0;
    while (turns++ < 10)
    {
        follow();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    motor_stop();
}
