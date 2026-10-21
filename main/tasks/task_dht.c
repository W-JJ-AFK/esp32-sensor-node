#include "tasks.h"

#include "dht22.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "TASK_DHT";

/* 连续失败多少次判定传感器失效。
 * 为什么要判失效而不是"沿用上一次的值"：DHT22 拔掉/接触不良后读取一直失败，
 * 而 g_dht 里还留着最后一次成功值，valid 仍是 true，于是遥测继续每 2s 上报
 * 同一个温度、屏幕照常显示 —— 传感器失效伪装成"数据稳定"，这是最危险的一类故障。 */
#define DHT_FAIL_LIMIT    3
#define DHT_FAIL_DELAY_MS 1000

/**
 * DHT22 单独一个任务：读取要按 bit 关中断（每个 bit 约 130us），
 * 绝不能放在 100Hz 采样任务里，否则每次读温湿度都会打乱 IMU 采样节拍。
 * 2s 采样间隔由驱动内部保证，这里只负责失败处理和状态发布。
 */
static void dht_task(void *arg)
{
    (void)arg;
    uint32_t fail_streak = 0;

    for (;;) {
        float temperature = 0.0f;
        float humidity = 0.0f;
        const esp_err_t err = dht22_read(&temperature, &humidity);

        if (err == ESP_OK) {
            if (fail_streak >= DHT_FAIL_LIMIT) {
                ESP_LOGI(TAG, "sensor recovered after %u failed reads",
                         (unsigned)fail_streak);
            }
            fail_streak = 0;

            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_dht.temperature_c = temperature;
            g_dht.humidity_pct = humidity;
            g_dht.valid = true;
            g_dht.ts_ms = esp_timer_get_time() / 1000; /* 业务时间用单调时钟，不用日志时间戳 */
            xSemaphoreGive(g_state_mutex);

            continue; /* 间隔由驱动内部保证 */
        }

        fail_streak++;
        if (fail_streak == DHT_FAIL_LIMIT) {
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            const bool was_valid = g_dht.valid;
            g_dht.valid = false;
            xSemaphoreGive(g_state_mutex);

            if (was_valid) {
                ESP_LOGE(TAG, "%u consecutive read failures, sensor marked INVALID",
                         (unsigned)fail_streak);
            }
        }

        /* 失败时降速重试：驱动内部已做过 3 次短退避，这里再兜一层，
         * 避免接触不良时每 100ms 刷一条 WARN。 */
        vTaskDelay(pdMS_TO_TICKS(DHT_FAIL_DELAY_MS));
    }
}

void task_dht_start(void)
{
    xTaskCreate(dht_task, "task_dht", APP_STACK_DHT, NULL, APP_PRIO_DHT,
                &g_task_handles[APP_TASK_DHT]);
}
