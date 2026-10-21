#include "tasks.h"

#include <stdio.h>

#include "esp_log.h"
#include "ssd1306.h"
#include "wifi_mgr.h"

static const char *TAG = "TASK_DISPLAY";

/**
 * OLED 刷新任务：先在 RAM 帧缓冲里画完整一帧，再一次性推屏，
 * 避免"边画边推"导致的撕裂。
 */
static void display_task(void *arg)
{
    (void)arg;
    char line[24];
    bool oled_ok = true;

    for (;;) {
        dht_snapshot_t dht;
        edge_snapshot_t edge;

        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        dht = g_dht;
        edge = g_edge;
        xSemaphoreGive(g_state_mutex);

        ssd1306_clear();
        ssd1306_draw_text(0, 0, "EDGE NODE");
        ssd1306_draw_text(72, 0, wifi_mgr_is_connected() ? "WIFI OK" : "WIFI --");

        if (dht.valid) {
            snprintf(line, sizeof(line), "T %.1fC H %.0f%%", dht.temperature_c,
                     dht.humidity_pct);
        } else {
            /* 传感器失效必须显式显示，不能继续显示最后一次成功值 */
            snprintf(line, sizeof(line), "T --.-C H --%%");
            ssd1306_draw_text(0, 5, "DHT22 FAIL");
        }
        ssd1306_draw_text(0, 2, line);

        if (edge.state == EDGE_STATE_UNKNOWN) {
            ssd1306_draw_text(0, 3, "RMS --- G");
        } else {
            snprintf(line, sizeof(line), "RMS %.3fG", edge.rms);
            ssd1306_draw_text(0, 3, line);
        }

        snprintf(line, sizeof(line), "STATE %s", edge_state_name(edge.state));
        ssd1306_draw_text(0, 4, line);

        if (ssd1306_flush() != ESP_OK && oled_ok) {
            ESP_LOGW(TAG, "oled flush failed, check I2C wiring at 0x%02X",
                     APP_OLED_I2C_ADDR);
            oled_ok = false; /* 只报一次，避免日志刷屏 */
        }

        vTaskDelay(pdMS_TO_TICKS(APP_DISPLAY_PERIOD_MS));
    }
}

void task_display_start(void)
{
    xTaskCreate(display_task, "task_display", APP_STACK_DISPLAY, NULL,
                APP_PRIO_DISPLAY, &g_task_handles[APP_TASK_DISPLAY]);
}
