/*
 * ST7735S 1.8" 128x160 TFT LCD 显示驱动（SPI），
 * 仅用于显示四路红外传感器 IR1~IR4 的状态，风格为精简的数码管（7 段）。
 *
 * ===== 数码管显示规则 =====
 *   - 数字顺序（从左到右）：IR1, IR2, IR3, IR4
 *   - 上排小号(绿色)：通道号 1~4
 *   - 下排大号(红色)：该通道当前电平
 *   - 电平 -> 数字（黑线=低/0，白底=高/1）：
 *        高电平 gpio_get_level()==1 (白底/未压线) -> 显示数字 '1'
 *        低电平 gpio_get_level()==0 (压到黑线)   -> 显示数字 '0'
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
#define LCD_COLOR_BLACK 0x0000   // 背景
#define LCD_COLOR_GHOST 0x2108   // 未点亮段（暗灰，形成数码管轮廓）
#define LCD_COLOR_VALUE 0xF800   // 电平数值点亮（红）
#define LCD_COLOR_IDX   0x07E0   // 通道号点亮（绿）

/* ======================== 数码管布局 ======================== */
#define IDX_DIGIT_W 16           // 通道号（小）
#define IDX_DIGIT_H 24
#define VAL_DIGIT_W 26           // 电平数值（大）
#define VAL_DIGIT_H 44
#define IDX_TOP_Y   26
#define VAL_TOP_Y   76
// 某列中心 x：16 + index*32  (i=0..3 -> 16,48,80,112)

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

static void lcd_cs_low(void)  { gpio_set_level(LCD_CS, 0); }
static void lcd_cs_high(void) { gpio_set_level(LCD_CS, 1); }
static void lcd_dc_low(void)  { gpio_set_level(LCD_DC, 0); }
static void lcd_dc_high(void) { gpio_set_level(LCD_DC, 1); }

// 在单次 CS 拉低事务里发送：命令字节 + 可选数据字节（DC 先低后高）
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

/* ======================== 7 段数码管 ======================== */

// seg 位定义：a=1<<0 b=1<<1 c=1<<2 d=1<<3 e=1<<4 f=1<<5 g=1<<6
static const uint8_t s_seg_map[10] = {
    0x3F, // 0: a b c d e f
    0x06, // 1: b c
    0x5B, // 2: a b g e d
    0x4F, // 3: a b g c d
    0x66, // 4: f g b c
    0x6D, // 5: a f g c d
    0x7D, // 6: a f g e d c
    0x07, // 7: a b c
    0x7F, // 8: a b c d e f g
    0x6F, // 9: a b c d f g
};

// 在 (dx,dy) 处绘制一个 7 段数码管数字 digit(0~9)，尺寸 w x h。
//   lit = 点亮段颜色；dark = 未点亮段颜色；先清空整个单元为黑色，避免残留。
static void lcd_draw_7seg_digit(uint16_t dx, uint16_t dy, uint16_t w, uint16_t h,
                                uint8_t digit, uint16_t lit, uint16_t dark)
{
    uint16_t t    = w / 5;
    if (t < 2) t  = 2;
    uint16_t half = h / 2;
    uint8_t segs  = s_seg_map[digit % 10];

    // 清空单元
    lcd_fill_rect(dx, dy, w, h, LCD_COLOR_BLACK);

    // 段几何（a 上, b 右上, c 右下, d 下, e 左下, f 左上, g 中）
    uint16_t sx[7] = { dx + t,      dx + w - t,  dx + w - t,  dx + t,
                       dx,          dx,          dx + t };
    uint16_t sy[7] = { dy,          dy + t,      dy + half,   dy + h - t,
                       dy + half,   dy + t,      dy + half - t / 2 };
    uint16_t sw[7] = { w - 2 * t,   t,           t,           w - 2 * t,
                       t,           t,           w - 2 * t };
    uint16_t sh[7] = { t,           half - t,    half - t,    t,
                       half - t,    half - t,    t };

    for (int i = 0; i < 7; i++) {
        uint16_t col = (segs & (1u << i)) ? lit : dark;
        lcd_fill_rect(sx[i], sy[i], sw[i], sh[i], col);
    }
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
        .pin_bit_mask = (1ULL << LCD_RST) | (1ULL << LCD_DC) | (1ULL << LCD_CS),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    // 2) 硬件复位
    gpio_set_level(LCD_CS, 1);
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
        .spics_io_num   = -1,            // CS 用 GPIO 手动控制
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

void lcd_show_ir(uint8_t ir1, uint8_t ir2, uint8_t ir3, uint8_t ir4)
{
    uint8_t vals[4] = { ir1, ir2, ir3, ir4 };

    for (int i = 0; i < 4; i++) {
        uint16_t cx = 16 + i * 32;   // 列中心

        // 上排：通道号 1~4（绿色，小号）
        lcd_draw_7seg_digit(cx - IDX_DIGIT_W / 2, IDX_TOP_Y,
                            IDX_DIGIT_W, IDX_DIGIT_H,
                            (uint8_t)(i + 1),
                            LCD_COLOR_IDX, LCD_COLOR_GHOST);

        // 下排：电平值（红色，大号）。高电平->1，低电平->0。
        uint8_t v = (vals[i] != 0) ? 1 : 0;
        lcd_draw_7seg_digit(cx - VAL_DIGIT_W / 2, VAL_TOP_Y,
                            VAL_DIGIT_W, VAL_DIGIT_H,
                            v, LCD_COLOR_VALUE, LCD_COLOR_GHOST);
    }
}

// 全屏填充红色，用于表示红外识别出现错误
void lcd_show_error(void)
{
    lcd_fill_rect(0, 0, LCD_H_RES, LCD_V_RES, LCD_COLOR_VALUE);
}
