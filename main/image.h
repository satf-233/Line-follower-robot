#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 1.8" ST7735S TFT LCD（配置 GPIO/SPI、复位、发送初始化序列、清屏为黑色）。
void lcd_init(void);

// 刷新四路红外传感器的状态显示（数码管风格）。
// 参数 ir1..ir4 为 gpio_get_level() 的返回值（0 或 1），从左到右对应 IR1~IR4。
void lcd_show_ir(uint8_t ir1, uint8_t ir2, uint8_t ir3, uint8_t ir4);

#ifdef __cplusplus
}
#endif

#endif // IMAGE_H
