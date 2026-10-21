#include "tasks.h"

#include "esp_log.h"
#include "esp_system.h"

static const char *TAG = "TASK_MONITOR";

static const char *const k_task_names[APP_TASK_COUNT] = {
    "imu", "detect", "dht", "net", "display", "monitor",
};

/**
 * 资源监控任务：定期打印各任务栈高水位、堆使用、队列/环形缓冲水位。
 * 栈高水位单位是「字」(word)，ESP32 上 1 word = 4 字节。
 */
static void monitor_task(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(APP_MONITOR_PERIOD_MS));

        ESP_LOGI(TAG, "heap free=%u min=%u bytes",
                 (unsigned)esp_get_free_heap_size(),
                 (unsigned)esp_get_minimum_free_heap_size());

        for (int i = 0; i < APP_TASK_COUNT; i++) {
            if (g_task_handles[i] == NULL) {
                continue;
            }
            const UBaseType_t words = uxTaskGetStackHighWaterMark(g_task_handles[i]);
            ESP_LOGI(TAG, "stack %-8s free=%u words (%u bytes)", k_task_names[i],
                     (unsigned)words, (unsigned)(words * sizeof(StackType_t)));
        }

        ESP_LOGI(TAG, "imu_ring used=%u dropped=%u alert_q=%u alert_backlog=%u(lost %u)",
                 (unsigned)imu_ring_count(&g_imu_ring),
                 (unsigned)imu_ring_dropped(&g_imu_ring),
                 (unsigned)uxQueueMessagesWaiting(g_alert_queue),
                 (unsigned)task_net_backlog_depth(),
                 (unsigned)task_net_backlog_dropped());
    }
}

void task_monitor_start(void)
{
    xTaskCreate(monitor_task, "task_monitor", APP_STACK_MONITOR, NULL,
                APP_PRIO_MONITOR, &g_task_handles[APP_TASK_MONITOR]);
}
