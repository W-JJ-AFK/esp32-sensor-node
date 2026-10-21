/**
 * @file ssd1306.h
 * @brief 0.96 寸 128x64 OLED 驱动（I2C，页寻址 + 本地帧缓冲）。
 *
 * 绘制都写到 RAM 帧缓冲，ssd1306_flush() 才真正推屏：
 * 这样一次刷新 = 8 次 I2C 传输（每次 129 字节），避免逐像素访问总线。
 * 字模只支持大写字母/数字/常用符号（见 ssd1306_font.h），小写会自动转大写。
 */

#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGES  (SSD1306_HEIGHT / 8)

esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, uint8_t address);

/** @brief 清空帧缓冲（不推屏）。 */
void ssd1306_clear(void);

/**
 * @brief 在帧缓冲上绘制字符串。
 * @param x    起始列（像素，0~127）
 * @param page 起始页（0~7，每页 8 像素高）
 */
void ssd1306_draw_text(uint8_t x, uint8_t page, const char *text);

/** @brief 把帧缓冲推到屏幕。 */
esp_err_t ssd1306_flush(void);
