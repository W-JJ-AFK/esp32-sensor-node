#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "RTOS_DEMO";

typedef struct {
    uint32_t id;
    float value;
    uint32_t timestamp;
} data_packet_t;

static QueueHandle_t data_queue;
static SemaphoreHandle_t uart_mutex;

static TaskHandle_t producer_handle;
static TaskHandle_t consumer_handle;
static TaskHandle_t monitor_handle;

static volatile uint32_t produced_count = 0;
static volatile uint32_t consumed_count = 0;

// 任务1: 生产者 (优先级3)
static void vProducerTask(void *arg) {
    data_packet_t packet;
    uint32_t counter = 0;
    float mock_value = 25.0f;

    for (;;) {
        packet.id = counter++;
        packet.timestamp = esp_timer_get_time() / 1000;
        mock_value = 25.0f + 3.0f * sinf(counter * 0.1f);
        packet.value = mock_value;

        if (xQueueSend(data_queue, &packet, 0) == pdPASS) {
            produced_count++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 任务2: 消费者 (优先级2)
static void vConsumerTask(void *arg) {
    data_packet_t packet;
    static float filter_buffer[5] = {0};
    static int filter_idx = 0;

    for (;;) {
        if (xQueueReceive(data_queue, &packet, portMAX_DELAY) == pdPASS) {
            consumed_count++;
            filter_buffer[filter_idx] = packet.value;
            filter_idx = (filter_idx + 1) % 5;

            float sum = 0;
            for (int i = 0; i < 5; i++) sum += filter_buffer[i];
            float filtered = sum / 5.0f;

            if (consumed_count % 50 == 0) {
                xSemaphoreTake(uart_mutex, portMAX_DELAY);
                printf("[Consumer] #%lu raw=%.2f filtered=%.2f queue=%u\n",
                       (unsigned long)packet.id, packet.value, filtered,
                       (unsigned)uxQueueMessagesWaiting(data_queue));
                xSemaphoreGive(uart_mutex);
            }
        }
    }
}

// 任务3: 系统监控 (优先级2)
static void vMonitorTask(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));

        UBaseType_t p_stack = uxTaskGetStackHighWaterMark(producer_handle);
        UBaseType_t c_stack = uxTaskGetStackHighWaterMark(consumer_handle);
        UBaseType_t m_stack = uxTaskGetStackHighWaterMark(monitor_handle);

        xSemaphoreTake(uart_mutex, portMAX_DELAY);
        printf("\n===== System Monitor =====\n");
        printf("Producer: %lu packets\n", (unsigned long)produced_count);
        printf("Consumer: %lu packets\n", (unsigned long)consumed_count);
        printf("Queue: %u used / %u free\n",
               (unsigned)uxQueueMessagesWaiting(data_queue),
               (unsigned)uxQueueSpacesAvailable(data_queue));
        printf("Stack free - P:%u C:%u M:%u words\n",
               (unsigned)p_stack, (unsigned)c_stack, (unsigned)m_stack);
        printf("Free heap: %lu bytes\n", (unsigned long)esp_get_free_heap_size());
        printf("==========================\n\n");
        xSemaphoreGive(uart_mutex);
    }
}

// 任务4: 心跳任务 (优先级1) - 替代软件定时器
static void vHeartbeatTask(void *arg) {
    int beat = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        beat++;
        xSemaphoreTake(uart_mutex, portMAX_DELAY);
        printf("[Heartbeat] #%d - system alive\n", beat);
        xSemaphoreGive(uart_mutex);
    }
}

// 任务5: 信号量演示 (优先级1)
static void vSemaphoreDemoTask(void *arg) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));

        if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(100)) == pdPASS) {
            printf("[Semaphore] mutex acquired, UART protected\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            printf("[Semaphore] mutex released\n");
            xSemaphoreGive(uart_mutex);
        } else {
            printf("[Semaphore] mutex timeout!\n");
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "FreeRTOS multi-task demo started");
    ESP_LOGI(TAG, "4 tasks + 1 semaphore task");

    data_queue = xQueueCreate(20, sizeof(data_packet_t));
    uart_mutex = xSemaphoreCreateMutex();

    xTaskCreate(vProducerTask, "Producer", 4096, NULL, 3, &producer_handle);
    xTaskCreate(vConsumerTask, "Consumer", 4096, NULL, 2, &consumer_handle);
    xTaskCreate(vMonitorTask, "Monitor", 4096, NULL, 2, &monitor_handle);
    xTaskCreate(vHeartbeatTask, "Heartbeat", 4096, NULL, 1, NULL);
    xTaskCreate(vSemaphoreDemoTask, "SemDemo", 4096, NULL, 1, NULL);

    ESP_LOGI(TAG, "Priorities: Producer(3) > Consumer(2) = Monitor(2) > Heartbeat(1) = SemDemo(1)");
    ESP_LOGI(TAG, "Queue: 20 packets\n");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}