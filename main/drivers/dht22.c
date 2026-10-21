#include "dht22.h"

#include <stdbool.h>

#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DHT22";

/* DHT22 规格：起始信号拉低至少 1ms，释放 20~40us 后从机响应 */
#define DHT_START_LOW_US     1200
#define DHT_RELEASE_US       30
#define DHT_TIMEOUT_US       120   /* 高/低电平等待上限，正常脉宽远小于此值 */
#define DHT_ONE_MIN_US       45    /* 高电平 > 45us 判 1（0: 26~28us，1: 70us） */
#define DHT_MIN_INTERVAL_US  2000000LL
#define DHT_RETRY_COUNT      3
#define DHT_RETRY_BACKOFF_MS 100   /* 第 n 次重试前等 n*100ms */

static gpio_num_t s_pin;
static bool s_initialized;
static int64_t s_last_read_us;
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static int wait_level(int level, int timeout_us)
{
    const int64_t start = esp_timer_get_time();
    while (gpio_get_level(s_pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) {
            return -1;
        }
    }
    return 0;
}

esp_err_t dht22_init(gpio_num_t pin)
{
    s_pin = pin;

    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        /* 开漏 + 输入输出一体：整个时序里**不再切换方向**。
         * DHT22 是单总线（双向）器件，若把引脚切成 GPIO_MODE_OUTPUT（推挽）
         * 去发起始信号，ESP32 在释放后会主动把线拉到 3.3V，与传感器随后的
         * 拉低响应"对着干"：脉宽被削、响应头丢边沿，接触稍差就整帧 CRC 错。
         * 开漏模式下写 1 = 释放（高阻 + 上拉），写 0 = 拉低，两者都不冲突。 */
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 内部上拉只有 45kΩ 左右，长线或杜邦线接触不良时上升沿会变缓（>2us），
     * 仍然建议外部 4.7k~10k 上拉到 3.3V。这里保留内部上拉做兜底。 */
    gpio_set_level(pin, 1); /* 释放总线，等传感器空闲 */

    s_last_read_us = 0;
    s_initialized = true;
    ESP_LOGI(TAG, "initialized on GPIO%d (open-drain, external 4.7k pull-up recommended)",
             (int)pin);
    return ESP_OK;
}

/**
 * 在临界区内测量一个 bit 的高电平宽度。
 *
 * 为什么按 bit 分，而不是把 40 个 bit 一起包起来：
 * 整段 bit-bang 约 4.6ms（1.2ms 起始 + 3.4ms 数据），期间本地核关中断。
 * ESP-IDF 的 WiFi 驱动要求中断响应在百微秒量级，连续关 4.6ms 会造成
 * beacon 丢失甚至断连。拆成"每 bit 一小段"后，最长连续关中断约 130us
 * （50us 低电平 + 70us 高电平），占空比 5.2ms/2s ≈ 0.26%，WiFi 完全无感。
 *
 * 更彻底的做法是用 RMT 外设接收脉宽，全程不关中断（见 README 的 ADR-12）：
 * 那需要真机标定 1us 分辨率下 80us 响应头的边界，本机没有硬件可验证，
 * 写一个无法验证的解析器风险高于收益，因此这里选择可证明正确的折中。
 */
static esp_err_t read_bit_locked(int *bit)
{
    int64_t high_us = 0;
    bool measured = false;

    portENTER_CRITICAL(&s_mux);
    if (wait_level(1, DHT_TIMEOUT_US) == 0) {
        const int64_t rising = esp_timer_get_time();
        if (wait_level(0, DHT_TIMEOUT_US) == 0) {
            high_us = esp_timer_get_time() - rising;
            measured = true;
        }
    }
    portEXIT_CRITICAL(&s_mux);

    if (!measured) {
        return ESP_ERR_TIMEOUT;
    }
    *bit = (high_us > DHT_ONE_MIN_US) ? 1 : 0;
    return ESP_OK;
}

/**
 * 完整的一次读时序。
 *
 * 起始信号与从机响应放在临界区外：前者只是"保持低电平 1.2ms"，没有需要
 * 精确测量的边沿；后者靠轮询检测电平变化，几百微秒的抖动不影响判断。
 */
static esp_err_t dht22_read_once(float *temperature_c, float *humidity_pct)
{
    uint8_t data[5] = {0};

    /* 1) 起始信号：开漏拉低 1.2ms（写 0 = 拉低，不需要切方向） */
    gpio_set_level(s_pin, 0);
    esp_rom_delay_us(DHT_START_LOW_US);

    /* 2) 释放总线：开漏写 1 = 高阻，由上拉拉高；传感器接管总线 */
    gpio_set_level(s_pin, 1);
    esp_rom_delay_us(DHT_RELEASE_US);

    /* 3) 从机响应：80us 低 + 80us 高，随后 80us 低电平准备发数据 */
    if (wait_level(0, DHT_TIMEOUT_US) != 0 ||
        wait_level(1, DHT_TIMEOUT_US) != 0 ||
        wait_level(0, DHT_TIMEOUT_US) != 0) {
        return ESP_ERR_TIMEOUT;
    }

    /* 4) 读 40 bit：按高电平宽度判定（0 = 26~28us，1 = 70us） */
    for (int i = 0; i < 40; i++) {
        int bit = 0;
        const esp_err_t err = read_bit_locked(&bit);
        if (err != ESP_OK) {
            return err;
        }
        data[i / 8] = (uint8_t)((data[i / 8] << 1) | (bit ? 1u : 0u));
    }

    const uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
    if (checksum != data[4]) {
        return ESP_ERR_INVALID_CRC;
    }

    const uint16_t raw_hum = ((uint16_t)data[0] << 8) | data[1];
    const uint16_t raw_temp = ((uint16_t)data[2] << 8) | data[3];

    /* 温度最高位为符号位，负温度用补码表示 */
    float temp = (raw_temp & 0x8000)
                     ? -(float)(raw_temp & 0x7FFF) / 10.0f
                     : (float)raw_temp / 10.0f;

    *temperature_c = temp;
    *humidity_pct = (float)raw_hum / 10.0f;
    return ESP_OK;
}

esp_err_t dht22_read(float *temperature_c, float *humidity_pct)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (temperature_c == NULL || humidity_pct == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 采样间隔限制只对"两次成功读取之间"有意义，只需在首次尝试前等一次。
     * 旧实现在重试循环里每次都重新满足 2s，最坏阻塞 6s —— 而 DHT22 失败的
     * 常见原因是接触不良，等 2s 并不会让它变好，短退避重试才是有效动作。 */
    if (s_last_read_us != 0) {
        const int64_t elapsed = esp_timer_get_time() - s_last_read_us;
        if (elapsed < DHT_MIN_INTERVAL_US) {
            const int64_t wait_ms = (DHT_MIN_INTERVAL_US - elapsed) / 1000 + 1;
            vTaskDelay(pdMS_TO_TICKS(wait_ms));
        }
    }

    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < DHT_RETRY_COUNT; attempt++) {
        if (attempt > 0) {
            vTaskDelay(pdMS_TO_TICKS(DHT_RETRY_BACKOFF_MS * attempt));
        }
        s_last_read_us = esp_timer_get_time();

        err = dht22_read_once(temperature_c, humidity_pct);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGW(TAG, "read failed (%d/%d): %s", attempt + 1, DHT_RETRY_COUNT,
                 esp_err_to_name(err));
    }

    return err;
}
