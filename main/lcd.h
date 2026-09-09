#ifndef LCD_H
#define LCD_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 1.8" ST7735S TFT LCD（配置 GPIO/SPI、复位、发送初始化序列、清屏为黑色）。
void lcd_init(void);

// 在屏幕显示超声波距离：格式 "Dist : xx"（cm，整数，四舍五入，2 位；负值显示 "Dist : --"）。
// 黑底白字；字号 / 位置 / 字符间距见 lcd.c 内宏定义。
void lcd_show_dist(float dist);

void lcd_show_error(float err);

void lcd_show_tri_error(float err, int mode);
#ifdef __cplusplus
}
#endif

#endif // LCD_H
