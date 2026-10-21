#include "ssd1306.h"

#include <string.h>

#include "esp_log.h"
#include "ssd1306_font.h"

#define CTRL_CMD       0x00
#define CTRL_DATA      0x40
#define I2C_TIMEOUT_MS 100

static const char *TAG = "SSD1306";

static i2c_master_dev_handle_t s_dev;
static uint8_t s_fb[SSD1306_PAGES][SSD1306_WIDTH];

static esp_err_t write_cmds(const uint8_t *cmds, size_t count)
{
    uint8_t buf[40];
    if (count + 1 > sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = CTRL_CMD;
    memcpy(&buf[1], cmds, count);
    return i2c_master_transmit(s_dev, buf, count + 1, I2C_TIMEOUT_MS);
}

esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, uint8_t address)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 400000,
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add device failed: %s", esp_err_to_name(err));
        return err;
    }

    static const uint8_t init_seq[] = {
        0xAE,       /* display off */
        0xD5, 0x80, /* clock div */
        0xA8, 0x3F, /* multiplex ratio = 64 */
        0xD3, 0x00, /* display offset 0 */
        0x40,       /* start line 0 */
        0x8D, 0x14, /* charge pump on */
        0x20, 0x02, /* page addressing mode */
        0xA1,       /* segment remap */
        0xC8,       /* COM scan direction reversed */
        0xDA, 0x12, /* COM pins */
        0x81, 0xCF, /* contrast */
        0xD9, 0xF1, /* pre-charge */
        0xDB, 0x40, /* VCOMH */
        0xA4,       /* resume from RAM */
        0xA6,       /* normal (non-inverted) */
        0xAF,       /* display on */
    };
    err = write_cmds(init_seq, sizeof(init_seq));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init sequence failed: %s", esp_err_to_name(err));
        return err;
    }

    ssd1306_clear();
    err = ssd1306_flush();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "initialized at 0x%02X", address);
    }
    return err;
}

void ssd1306_clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void ssd1306_draw_text(uint8_t x, uint8_t page, const char *text)
{
    if (text == NULL || page >= SSD1306_PAGES) {
        return;
    }

    uint8_t col = x;
    for (const char *p = text; *p != '\0'; p++) {
        if (col + FONT_WIDTH >= SSD1306_WIDTH) {
            break;
        }
        char c = *p;
        if (c >= 'a' && c <= 'z') {
            c = (char)(c - 32); /* 字模只覆盖大写 */
        }
        const int idx = (c < FONT_FIRST_CHAR || c > FONT_LAST_CHAR) ? 0 : (c - FONT_FIRST_CHAR);

        for (int i = 0; i < FONT_WIDTH; i++) {
            s_fb[page][col + i] = FONT5X7[idx][i];
        }
        s_fb[page][col + FONT_WIDTH] = 0x00; /* 字间距 1 像素 */
        col = (uint8_t)(col + FONT_WIDTH + 1);
    }
}

esp_err_t ssd1306_flush(void)
{
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[SSD1306_WIDTH + 1];
    buf[0] = CTRL_DATA;
    /* 中途失败也要把剩下的页发完：逐页 return 会让屏幕停在"上半新下半旧"，
     * 看起来像偶发的花屏。这里记住第一个错误，发完整帧再返回。 */
    esp_err_t first_err = ESP_OK;

    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        const uint8_t addr[] = {
            (uint8_t)(0xB0 | page), /* 设置页地址 */
            0x00,                   /* 列地址低 4 位 */
            0x10,                   /* 列地址高 4 位 */
        };
        esp_err_t err = write_cmds(addr, sizeof(addr));
        if (err == ESP_OK) {
            memcpy(&buf[1], s_fb[page], SSD1306_WIDTH);
            err = i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
        }
        if (err != ESP_OK && first_err == ESP_OK) {
            first_err = err;
        }
    }
    return first_err;
}
