#include "tasks.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "mpu6050.h"

static const char *TAG = "TASK_IMU";

/**
 * 100Hz 硬实时采样任务，全工程最高优先级。
 * 只做「读 I2C + 入环」，不做任何计算，保证采样节拍不被拉长。
 */
static void imu_task(void *arg)
{
    (void)arg;
    imu_sample_t sample = {0};
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t error_count = 0;

    for (;;) {
        if (mpu6050_read_accel(&sample.ax, &sample.ay, &sample.az) == ESP_OK) {
            sample.ts_us = esp_timer_get_time();
            /* 环满说明检测任务跟不上，直接丢弃新样本：宁可丢点也不阻塞采样 */
            (void)imu_ring_push(&g_imu_ring, &sample);
            if (error_count > 0) {
                ESP_LOGW(TAG, "recovered after %u failed reads", (unsigned)error_count);
                error_count = 0;
            }
        } else if ((error_count++ % 100) == 0) {
            ESP_LOGW(TAG, "imu read failed (%u)", (unsigned)error_count);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(APP_IMU_PERIOD_MS));
    }
}

void task_imu_start(void)
{
    xTaskCreate(imu_task, "task_imu", APP_STACK_IMU, NULL, APP_PRIO_IMU,
                &g_task_handles[APP_TASK_IMU]);
}
