#include "start.h"
#include "motor.h"
#include "lcd.h"
#include "avoid.h"
#include "follow_brain.h"
#include "pins.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"

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
    avoid_init();// 超声波传感器初始化
    //TODO:将所有初始化函数包含到同一个头文件当中  

    wait_boot_press();   // 等按一下 BOOT 再运行

    
    // led配置
    led_strip_handle_t led_strip;

    // v3.x API：字段名变了
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_GPIO,
        .max_leds = LED_NUM,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,  // v3.x 用这个
        .flags = {
            .invert_out = false,
        }
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags = {
            .with_dma = false,
        }
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    double speed = 0.16;
    motor_forward(speed);
    int count = 0;
    int state = 1;
    while (1)
    {
        //默认为循迹状态
        if(state == 1){
            
            follow(led_strip); 
            
            //进入距离阈值
            int Dis_thre = 10;
            //每100ms判断一次前方是否有障碍物和显示距离
            if (count > 100) {
                count = 0;
                float dis = avoid_measure_cm();
                lcd_show_dist(dis);
                if (dis < Dis_thre && dis > 0){
                    motor_stop();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    state = 2;
                }
            }
            count++;
        }

        //避障状态，不会连续两次进入该状态
        else if(state == 2){
            count = 0;

            //避障完成后进入循迹-停止状态
            if (avoid_run() == true){
                motor_stop();
                state = 3;
            }
            else{
                //不进行任何处理
            }
        }

        //循迹-停止状态，加入4黑停止逻辑
        else if (state == 3) {
            //循迹，当出现4黑时直接跳出循环
            if(follow_to_stop(led_strip) == 1){
                break;
            }

            //每100ms刷新显示距离
            if (count > 100) {
                count = 0;
                lcd_show_dist(avoid_measure_cm());
            }
            count++;
        }

        //未知状态，跳出循环并停止
        else{
            break;
        }
    }
    motor_stop();
}