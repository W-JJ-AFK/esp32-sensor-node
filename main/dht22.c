#include "dht22.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

static const char *TAG = "DHT22";
static gpio_num_t dht_pin;

static int wait_level(int level, int timeout_us) {
    int64_t start = esp_timer_get_time();
    while (gpio_get_level(dht_pin) != level) {
        if ((esp_timer_get_time() - start) > timeout_us) return -1;
    }
    return 0;
}

void dht22_init(gpio_num_t pin) {
    dht_pin = pin;
    gpio_set_direction(dht_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_pin, 1);
}

int dht22_read(float *temperature, float *humidity) {
    uint8_t data[5] = {0};
    
    // 拉低 2ms
    gpio_set_direction(dht_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_pin, 0);
    ets_delay_us(2000);
    
    // 拉高 30us
    gpio_set_level(dht_pin, 1);
    ets_delay_us(30);
    
    // 切换为输入
    gpio_set_direction(dht_pin, GPIO_MODE_INPUT);
    
    // 等待 DHT22 响应
    if (wait_level(0, 100) < 0) {
        ESP_LOGE(TAG, "等待响应低电平超时");
        return -1;
    }
    if (wait_level(1, 100) < 0) {
        ESP_LOGE(TAG, "等待响应高电平超时");
        return -1;
    }
    if (wait_level(0, 100) < 0) {
        ESP_LOGE(TAG, "等待数据开始超时");
        return -1;
    }
    
    // 读取 40 位数据
    for (int i = 0; i < 40; i++) {
        if (wait_level(1, 100) < 0) {
            ESP_LOGE(TAG, "等待位高电平超时 (bit %d)", i);
            return -1;
        }
        ets_delay_us(40);
        int bit = gpio_get_level(dht_pin);
        if (wait_level(0, 100) < 0) {
            ESP_LOGE(TAG, "等待位低电平超时 (bit %d)", i);
            return -1;
        }
        data[i / 8] = (data[i / 8] << 1) | bit;
    }
    
    // 校验
    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        ESP_LOGE(TAG, "校验失败: %02x != %02x", checksum, data[4]);
        return -1;
    }
    
    uint16_t hum = (data[0] << 8) | data[1];
    uint16_t temp = (data[2] << 8) | data[3];
    if (temp & 0x8000) {
        temp = temp & 0x7FFF;
        *temperature = -temp / 10.0f;
    } else {
        *temperature = temp / 10.0f;
    }
    *humidity = hum / 10.0f;
    
    ESP_LOGI(TAG, "温度: %.1f°C, 湿度: %.1f%%", *temperature, *humidity);
    return 0;
}
