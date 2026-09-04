#include "start.h"
#include "motor.h"
#include "avoid.h"
#include "lcd.h"
#include "follow_brain.h"
#include "pins.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 等待按一下 BOOT 按键（低电平）后再启动，带简单消抖
static void wait_boot_press(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BOOT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,        // 上拉：未按下读到 1
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    int stable = 0;
    while (stable < 3)          // 连续 3 次读到低电平（约 30ms）才算按下
    {
        if (gpio_get_level(BOOT_GPIO) == 0)
            stable++;
        else
            stable = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ================================================================
// 启动函数：启动循迹程序    
// ================================================================

void start(void)
{    
    motor_init();
    ir_init();   // 先初始化，follow() 才能读到电平
    lcd_init();  // 液晶屏幕初始化  
    avoid_init();

    wait_boot_press();   // 等按一下 BOOT 再运行

    double speed = 0.16;
    motor_forward(speed);
    follow();
    avoid_run();
    follow_to_stop();
}
