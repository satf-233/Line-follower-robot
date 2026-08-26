#include "start.h"
#include "motor.h"
#include "lcd.h"
#include "follow_brain.h"
#include "pins.h"
#include "driver/gpio.h"

// ================================================================
// 启动函数：启动循迹程序    
// ================================================================

void start(void)
{    
    motor_init();
    ir_init();   // 先初始化，follow() 才能读到电平
    lcd_init();  // 液晶屏幕初始化  

    double speed = 0.15;
    motor_forward(speed);
    int turns = 0;
    while (turns++ < 100)
    {
        follow();
        //把四路电平IR1~IR4从左往右依次显示
        lcd_show_ir((uint8_t)gpio_get_level(IR1),
                    (uint8_t)gpio_get_level(IR2),
                    (uint8_t)gpio_get_level(IR3),
                    (uint8_t)gpio_get_level(IR4));
    }
    motor_stop();
}
