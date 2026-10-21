#include "tasks.h"

#include "esp_log.h"

static const char *TAG = "TASK_DETECT";

/**
 * 检测任务：每 50ms 把环形缓冲里积压的样本全部喂给检测器。
 *
 * 注意这里是「批量消费」而不是「降采样」——100Hz 的每一个样本都会经过状态机，
 * 所以连续确认 5 次实际只代表 50ms，而不是 250ms。
 */
static void detect_task(void *arg)
{
    (void)arg;
    imu_sample_t sample;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(APP_DETECT_PERIOD_MS));

        while (imu_ring_pop(&g_imu_ring, &sample)) {
            float rms = 0.0f;
            const bool fire = edge_detector_feed(&g_detector, sample.ax, sample.ay,
                                                 sample.az, sample.ts_us, &rms);

            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_edge.rms = rms;
            g_edge.state = g_detector.state;
            g_edge.ts_ms = sample.ts_us / 1000;
            if (rms > g_edge.peak) {
                g_edge.peak = rms; /* 峰值由 task_net 每次上报后清零 */
            }
            xSemaphoreGive(g_state_mutex);

            if (fire) {
                const edge_alert_t alert = {
                    .rms = rms,
                    .threshold = g_detector.threshold_on,
                    .state = g_detector.state,
                    .ts_ms = sample.ts_us / 1000,
                };
                if (xQueueSend(g_alert_queue, &alert, 0) != pdPASS) {
                    ESP_LOGW(TAG, "alert queue full, event dropped");
                } else {
                    ESP_LOGW(TAG, "ALARM rms=%.3fg (threshold %.3fg)", rms,
                             g_detector.threshold_on);
                }
            }
        }
    }
}

void task_detect_start(void)
{
    xTaskCreate(detect_task, "task_detect", APP_STACK_DETECT, NULL,
                APP_PRIO_DETECT, &g_task_handles[APP_TASK_DETECT]);
}
