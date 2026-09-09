/*
 * ST7735S 1.8" 128x160 TFT LCD 显示驱动（SPI）。
 * 用于显示超声波距离，采用 8x16 点阵字体（仿宋体），黑底白字。
 *
 * 显示格式：第一行 "Dist : xx"（cm，整数，四舍五入，2 位；负值显示 "Dist : --"）。
 * 字号 / 位置 / 字符间距均通过下方宏调整。
 */

#include "lcd.h"
#include "pins.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* ======================== 配置参数 ======================== */

#define LCD_H_RES       128
#define LCD_V_RES       160

#define LCD_SPI_HOST    SPI2_HOST
#define LCD_SPI_CLK_HZ  (20 * 1000 * 1000)   // ST7735S：20MHz 稳妥；可逐步升到 32MHz

// 显示方向/色彩（MADCTL，ST7735S）
#define LCD_MADCTL      0xC0

// ST7735S 列/行偏移
#define LCD_OFFSET_X    0
#define LCD_OFFSET_Y    0

/* ======================== 颜色 ======================== */
#define LCD_COLOR_BLACK 0x0000
#define LCD_COLOR_WHITE 0xFFFF
#define LCD_COLOR_FG    LCD_COLOR_WHITE   // 前景（白字）
#define LCD_COLOR_BG    LCD_COLOR_BLACK   // 背景（黑底）

/* ======================== 文本显示参数 ======================== */
#define FONT_W          8    // 字模宽（像素）
#define FONT_H          16   // 字模高（像素）
#define LCD_TEXT_SCALE  1    // 字号：放大倍数（1=原始 8x16；2=16x32，注意 9 字符会超出屏宽）
#define LCD_COL_GAP     2    // 字符间距
// 文字纵向位置（改这里调整上下位置）
#define LCD_TEXT_Y      25

// 三台电机(A/B/D)脉冲数的行位置：在 Dist 下方，行距=字高+4px
#define LCD_LINE_STRIDE (FONT_H * LCD_TEXT_SCALE + 8)
#define LCD_TEXT_Y_A    (LCD_TEXT_Y + LCD_LINE_STRIDE)
#define LCD_TEXT_Y_B    (LCD_TEXT_Y_A + LCD_LINE_STRIDE)
#define LCD_TEXT_Y_D    (LCD_TEXT_Y_B + LCD_LINE_STRIDE)


/* ======================== 静态变量 ======================== */
static spi_device_handle_t s_spi;

/* ======================== 底层 SPI/GPIO ======================== */

// 一次由轮询发送 len 字节（CS 由调用方控制）
static void lcd_spi_write(const uint8_t *data, size_t len)
{
    if (len == 0) {
        return;
    }
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.tx_buffer = data;
    t.length    = len * 8;
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_spi, &t));
}

static void lcd_cs_low(void)  { //gpio_set_level(LCD_CS, 0);
                                 }
static void lcd_cs_high(void) { //gpio_set_level(LCD_CS, 1);
                             }
static void lcd_dc_low(void)  { gpio_set_level(LCD_DC, 0); }
static void lcd_dc_high(void) { gpio_set_level(LCD_DC, 1); }

//命令字节 + 可选数据字节（DC 先低后高，CS 一次拉低事务内完成）
static void lcd_send_cmd_with_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    lcd_cs_low();
    lcd_dc_low();
    lcd_spi_write(&cmd, 1);
    if (len) {
        lcd_dc_high();
        lcd_spi_write(data, len);
    }
    lcd_cs_high();
}

static void lcd_send_cmd(uint8_t cmd)
{
    lcd_send_cmd_with_data(cmd, NULL, 0);
}

// 开始/结束一次连续的数据写入（用于 ramwr 大数据，CS 保持拉低）
static void lcd_begin_data(void)
{
    lcd_cs_low();
    lcd_dc_high();
}
static void lcd_end_data(void)
{
    lcd_cs_high();
}

/* ======================== 窗口 / 填充 ======================== */

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    uint16_t sx0 = x0 + LCD_OFFSET_X, sx1 = x1 + LCD_OFFSET_X;
    uint16_t sy0 = y0 + LCD_OFFSET_Y, sy1 = y1 + LCD_OFFSET_Y;

    uint8_t cs[4] = { (uint8_t)(sx0 >> 8), (uint8_t)(sx0 & 0xFF),
                      (uint8_t)(sx1 >> 8), (uint8_t)(sx1 & 0xFF) };
    lcd_send_cmd_with_data(0x2A, cs, 4);   // CASET

    uint8_t rs[4] = { (uint8_t)(sy0 >> 8), (uint8_t)(sy0 & 0xFF),
                      (uint8_t)(sy1 >> 8), (uint8_t)(sy1 & 0xFF) };
    lcd_send_cmd_with_data(0x2B, rs, 4);   // RASET

    lcd_send_cmd(0x2C);                    // RAMWR
}

// 填充矩形（RGB565，自动裁剪到屏内）
static void lcd_fill_rect(uint16_t x0, uint16_t y0, uint16_t w, uint16_t h, uint16_t color)
{
    if (w == 0 || h == 0) {
        return;
    }
    if (x0 >= LCD_H_RES || y0 >= LCD_V_RES) {
        return;
    }
    if ((uint32_t)x0 + w > LCD_H_RES) {
        w = LCD_H_RES - x0;
    }
    if ((uint32_t)y0 + h > LCD_V_RES) {
        h = LCD_V_RES - y0;
    }
    if (w == 0 || h == 0) {
        return;
    }

    lcd_set_window(x0, y0, x0 + w - 1, y0 + h - 1);
    lcd_begin_data();

    // 分批发送，RGB565 高字节先发
    enum { CHUNK_PX = 256 };
    static uint8_t buf[CHUNK_PX * 2];
    uint8_t hi = (uint8_t)(color >> 8);
    uint8_t lo = (uint8_t)(color & 0xFF);
    for (int i = 0; i < CHUNK_PX; i++) {
        buf[i * 2]     = hi;
        buf[i * 2 + 1] = lo;
    }

    uint32_t n = (uint32_t)w * h;
    while (n >= CHUNK_PX) {
        lcd_spi_write(buf, CHUNK_PX * 2);
        n -= CHUNK_PX;
    }
    if (n) {
        lcd_spi_write(buf, (size_t)n * 2);   // buf 全是同一像素颜色，取前 n*2 字节即可
    }

    lcd_end_data();
}

/* ======================== 点阵字体（仿宋体） ======================== */
// 8x16 字模：每个字符 16 字节，每字节为一行，MSB(0x80) 为最左列。

static const uint8_t font_0[16] = {
    0x00,0x3C,0x66,0xC3,0xC3,0xC3,0xC3,0xC3,
    0xC3,0xC3,0xC3,0xC3,0xC3,0x66,0x3C,0x00 };
static const uint8_t font_1[16] = {
    0x00,0x18,0x38,0x18,0x18,0x18,0x18,0x18,
    0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00 };
static const uint8_t font_2[16] = {
    0x00,0x3C,0x66,0xC3,0xC3,0xC3,0x03,0x03,
    0x06,0x0C,0x18,0x30,0x60,0xC0,0xFF,0x00 };
static const uint8_t font_3[16] = {
    0x00,0x3C,0x66,0xC3,0xC3,0x03,0x06,0x0C, 
    0x06,0x03,0x03,0xC3,0xC3,0x66,0x3C,0x00 };
static const uint8_t font_4[16] = {
    0x00,0x06,0x06,0x0E,0x1E,0x36,0x66,0xC6,
    0xC6,0xFF,0x06,0x06,0x06,0x06,0x06,0x00 };
static const uint8_t font_5[16] = {
    0x00,0xFF,0xC0,0xC0,0xC0,0xFC,0x06,0x03,
    0x03,0xC3,0xC3,0xC3,0xC3,0x66,0x3C,0x00 };
static const uint8_t font_6[16] = {
    0x00,0x3C,0x66,0xC3,0xC0,0xC0,0xFC,0xC6,
    0xC3,0xC3,0xC3,0xC3,0xC3,0x66,0x3C,0x00 };
static const uint8_t font_7[16] = {
    0x00,0xFF,0x03,0x03,0x06,0x0C,0x0C,0x18,
    0x18,0x18,0x30,0x30,0x30,0x30,0x30,0x30 };
static const uint8_t font_8[16] = {
    0x00,0x3C,0x66,0xC3,0xC3,0xC3,0x66,0x3C,
    0x66,0xC3,0xC3,0xC3,0xC3,0x66,0x3C,0x00 };
static const uint8_t font_9[16] = {
    0x00,0x3C,0x66,0xC3,0xC3,0xC3,0xC3,0xC3,
    0x63,0x3F,0x03,0x03,0x03,0x66,0x3C,0x00 };
static const uint8_t font_D[16] = {
    0x00,0xF8,0xC6,0xC3,0xC3,0xC3,0xC3,0xC3,
    0xC3,0xC3,0xC3,0xC3,0xC3,0xC6,0xF8,0x00 };
/*........
#####...
##...##.
##....##
##....##
##....##
##....##
##....##
##....##
##....##
##....##
##....##
##....##
##...##.
#####...
........*/
static const uint8_t font_A[16] = {
    0x00,0x18,0x3C,0x66,0xC3,0xC3,0xC3,0xFF,
    0xFF,0xC3,0xC3,0xC3,0xC3,0xC3,0x00,0x00 };
static const uint8_t font_B[16] = {
    0x00,0xFC,0xC6,0xC3,0xC3,0xC3,0xC6,0xFC,
    0xC6,0xC3,0xC3,0xC3,0xC3,0xC6,0xFC,0x00 };
static const uint8_t font_i[16] = {
    0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x18,
    0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x00 };
/*........
........
........
...**...
...**...
........
........
...**...
...**...
...**...
...**...
...**...
...**...
...**...
...**...
........*/
static const uint8_t font_s[16] = {
    0x00,0x00,0x00,0x3C,0x66,0xC3,0xC0,0xC0,
    0x7C,0x06,0x03,0x03,0xC3,0x66,0x3C,0x00 };
/*........
........    
........
..####..
.##..##.
##....##
##......
##......
.#####..
.....##.
......##
......##
##....##
.##..##.
..####..
........*/
static const uint8_t font_t[16] = {
    0x00,0x00,000,0x18,0x18,0x7E,0x18,0x18,
    0x18,0x18,0x18,0x18,0x1A,0x1A,0x0C,0x00};
/*........
........
........
...##...
...##...
.######.
...##...
...##...
...##...
...##...
...##...
...##...
...##.#.
...##.#.
....##..
........*/
static const uint8_t font_colon[16] = {
    0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,
    0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00 };
static const uint8_t font_minus[16] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x7E,0x7E,0x00,0x00,0x00,0x00,0x00 };
static const uint8_t font_point[16] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x3C,0x3C,0x3C,0x3C,0x00};
static const uint8_t font_space[16] = { 0 };

// 取得某个字符对应的字模
static const uint8_t *font_glyph(char c)
{
    static const uint8_t *const digits[10] = {
        font_0, font_1, font_2, font_3, font_4,
        font_5, font_6, font_7, font_8, font_9
    };
    if (c >= '0' && c <= '9') {
        return digits[c - '0'];
    }
    switch (c) {
        case 'A':  return font_A;
        case 'B':  return font_B;
        case 'D':  return font_D;
        case 'i':  return font_i;
        case 's':  return font_s;
        case 't':  return font_t;
        case ':':  return font_colon;
        case '-':  return font_minus;
        case '.':  return font_point;
        default:   return font_space;
    }
}

// 绘制单个字符（scale 为放大倍数）：先整体清背景，再画前景像素
static void lcd_draw_char(uint16_t x, uint16_t y, char ch, uint8_t scale,
                          uint16_t fg, uint16_t bg)
{
    const uint8_t *g = font_glyph(ch);
    uint16_t w = FONT_W * scale;
    uint16_t h = FONT_H * scale;

    lcd_fill_rect(x, y, w, h, bg);   // 清掉该字符格

    for (uint8_t r = 0; r < FONT_H; r++) {
        uint8_t bits = g[r];
        for (uint8_t c = 0; c < FONT_W; c++) {
            if (bits & (0x80u >> c)) {
                lcd_fill_rect(x + c * scale, y + r * scale, scale, scale, fg);
            }
        }
    }
}

// 依次绘制字符串（自动按字符间距前进）
static void lcd_draw_text(uint16_t x, uint16_t y, const char *s, uint8_t scale,
                          uint16_t fg, uint16_t bg)
{
    while (*s) {
        lcd_draw_char(x, y, *s++, scale, fg, bg);
        x += (FONT_W * scale) + LCD_COL_GAP;
    }
}

// 整行水平居中显示
static void lcd_draw_text_centered(uint16_t y, const char *s, uint8_t scale,
                                   uint16_t fg, uint16_t bg)
{
    size_t n = strlen(s);
    uint16_t w = (n ? (uint16_t)(n * (FONT_W * scale) + (n - 1) * LCD_COL_GAP) : 0);
    uint16_t x = (w >= LCD_H_RES) ? 0 : (LCD_H_RES - w) / 2;
    lcd_draw_text(x, y, s, scale, fg, bg);
}

/* ======================== 初始化序列 ======================== */

typedef struct {
    uint8_t       cmd;
    uint8_t       len;
    const uint8_t *data;
    uint32_t      delay_ms;
} lcd_cmd_t;

// ST7735S 初始化参数值
static const uint8_t st_frm1[]   = { 0x01, 0x2C, 0x2D };                // FRMCTR1 帧率
static const uint8_t st_frm2[]   = { 0x01, 0x2C, 0x2D };                // FRMCTR2
static const uint8_t st_frm3[]   = { 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D }; // FRMCTR3
static const uint8_t st_invctr[] = { 0x07 };                            // 翻转控制
static const uint8_t st_pwr1[]   = { 0xA2, 0x02, 0x84 };                // PWCTR1 电源
static const uint8_t st_pwr2[]   = { 0xC5 };                            // PWCTR2
static const uint8_t st_pwr3[]   = { 0x0A, 0x00 };                      // PWCTR3
static const uint8_t st_vm1[]    = { 0x8A, 0x2A };                      // VMCTR1 VCOM
static const uint8_t st_vm2[]    = { 0x8A, 0xEE };                      // VMCTR2 VCOM
static const uint8_t st_vm3[]    = { 0x0E };                            // VMCTR3 VCOM
static const uint8_t st_colmod[] = { 0x05 };                            // 像素格式 16bit
static const uint8_t st_madctl[] = { LCD_MADCTL };                      // 方向/色彩
// ST7735S 正/负伽马（各 16 字节）
static const uint8_t st_gp[] = { 0x02,0x1C,0x07,0x12,0x37,0x32,0x29,0x2D,
                                 0x29,0x25,0x2B,0x39,0x00,0x01,0x03,0x10 };
static const uint8_t st_gn[] = { 0x03,0x1D,0x07,0x06,0x2E,0x2C,0x29,0x2D,
                                 0x2E,0x2E,0x37,0x3F,0x00,0x00,0x02,0x10 };

static const lcd_cmd_t lcd_init_cmds[] = {
    { 0x01, 0,  NULL,         10   },   // SWRESET 软复位
    { 0x11, 0,  NULL,         120  },   // SLPOUT 睡眠退出
    { 0x3A, 1,  st_colmod,    0    },   // COLMOD 像素格式 16bit
    { 0x36, 1,  st_madctl,    0    },   // MADCTL 方向/色彩
    { 0xB1, 3,  st_frm1,      0    },   // FRMCTR1 帧率
    { 0xB2, 3,  st_frm2,      0    },   // FRMCTR2
    { 0xB3, 6,  st_frm3,      0    },   // FRMCTR3
    { 0xB4, 1,  st_invctr,    0    },   // INVCTR 翻转
    { 0xC0, 3,  st_pwr1,      0    },   // PWCTR1 电源
    { 0xC1, 1,  st_pwr2,      0    },   // PWCTR2
    { 0xC2, 2,  st_pwr3,      0    },   // PWCTR3
    { 0xC3, 2,  st_vm1,       0    },   // VMCTR1 VCOM
    { 0xC4, 2,  st_vm2,       0    },   // VMCTR2 VCOM
    { 0xC5, 1,  st_vm3,       0    },   // VMCTR3 VCOM
    { 0xE0, 16, st_gp,        0    },   // GMCTRP1 正伽马
    { 0xE1, 16, st_gn,        0    },   // GMCTRN1 负伽马
    { 0x29, 0,  NULL,         0    },   // DISPON 显示开
};

/* ======================== 公共接口 ======================== */

void lcd_init(void)
{
    // 1) 输出 GPIO：RST / DC / CS（SCK/MOSI 由 SPI 总线配置）
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << LCD_RST) | (1ULL << LCD_DC),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    // 2) 硬件复位
    //gpio_set_level(LCD_CS, 1);
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    // 3) SPI 总线 + 设备
    spi_bus_config_t buscfg = {
        .sclk_io_num     = LCD_SCK,
        .mosi_io_num     = LCD_SDI,
        .miso_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * 2,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_SPI_CLK_HZ,
        .mode           = 0,
        .spics_io_num   = -1,           
        .queue_size     = 1,
        .flags          = SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_SPI_HOST, &devcfg, &s_spi));

    // 4) 发送初始化序列
    for (size_t i = 0; i < sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]); i++) {
        const lcd_cmd_t *c = &lcd_init_cmds[i];
        if (c->len) {
            lcd_send_cmd_with_data(c->cmd, c->data, c->len);
        } else {
            lcd_send_cmd(c->cmd);
        }
        if (c->delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(c->delay_ms));
        }
    }

    // 5) 清屏为黑色
    lcd_fill_rect(0, 0, LCD_H_RES, LCD_V_RES, LCD_COLOR_BLACK);
}

void lcd_show_dist(float dist)
{
    // 显示 "Dist : xx"（cm，整数，四舍五入，2 位）；负值（无回波/超量程）显示 "Dist : --"
    char buf[16];

    if (dist < 0) {
        strcpy(buf, "Dist : --");
    } else {
        int v = (int)(dist + 0.5f);
        if (v < 0)  v = 0;
        if (v > 99) v = 99;
        buf[0] = 'D'; buf[1] = 'i'; buf[2] = 's'; buf[3] = 't';
        buf[4] = ' '; buf[5] = ':'; buf[6] = ' ';
        buf[7] = (char)('0' + v / 10);
        buf[8] = (char)('0' + v % 10);
        buf[9] = '\0';
    }

    lcd_draw_text_centered(LCD_TEXT_Y, buf, LCD_TEXT_SCALE, LCD_COLOR_FG, LCD_COLOR_BG);
}

void lcd_show_error(float err){
    char buf[16];

    if (err < 0) {
        buf[0] = '-';
        err = -err;
    } 
    else {
        buf[0] = ' ';
    }
    int v = (int)(err * 1000);
    if (v > 999) v = 999;
    buf[1] = '0';
    buf[2] = '.';
    buf[3] = (char)('0'+(v/100));
    buf[4] = (char)('0'+((v % 100)/10));
    buf[5] = (char)('0'+ (v % 10));
    buf[6] = '\0';

    lcd_draw_text_centered(65, buf, LCD_TEXT_SCALE, LCD_COLOR_FG, LCD_COLOR_BG);
}

void lcd_show_tri_error(float err, int mode){
    char buf[16];
    if (err < 0) {
        buf[0] = '-';
        err = -err;
    } 
    else {
        buf[0] = ' ';
    }
    int v = (int)(err * 1000);
    if (v > 999) v = 999;
    buf[1] = '0';
    buf[2] = '.';
    buf[3] = (char)('0'+(v/100));
    buf[4] = (char)('0'+((v % 100)/10));
    buf[5] = (char)('0'+ (v % 10));
    buf[6] = '\0';

    lcd_draw_text_centered(LCD_TEXT_Y + mode * LCD_LINE_STRIDE, buf, LCD_TEXT_SCALE, LCD_COLOR_FG, LCD_COLOR_BG);
}
