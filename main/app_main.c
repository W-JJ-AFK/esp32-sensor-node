#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "common.h"
#include "dht22.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mqtt_mgr.h"
#include "mpu6050.h"
#include "nvs_flash.h"
#include "protocol.h"
#include "ssd1306.h"
#include "tasks.h"
#include "time_sync.h"
#include "wifi_mgr.h"

static const char *TAG = "APP";

/* ---------------- 跨任务共享状态（声明见 tasks.h） ---------------- */
app_context_t     g_app;
SemaphoreHandle_t g_state_mutex;
dht_snapshot_t    g_dht;
edge_snapshot_t   g_edge;
imu_ring_t        g_imu_ring;
QueueHandle_t     g_alert_queue;
edge_detector_t   g_detector;
volatile uint32_t g_telemetry_period_ms = CONFIG_APP_TELEMETRY_PERIOD_MS;
TaskHandle_t      g_task_handles[APP_TASK_COUNT];

/* reboot 定时器只创建一次并复用。
 * 旧实现在每次下发 reboot 时都 esp_timer_create 一个新 timer 且从不销毁，
 * 如果重启因为某种原因没发生，重复下发会持续泄漏定时器结构。 */
static esp_timer_handle_t s_reboot_timer;

static void reboot_timer_cb(void *arg)
{
    (void)arg;
    ESP_LOGW(TAG, "rebooting as requested by remote command");
    esp_restart();
}

static void reboot_deferred(void)
{
    if (s_reboot_timer == NULL) {
        const esp_timer_create_args_t timer_args = {
            .callback = reboot_timer_cb,
            .name = "reboot",
        };
        if (esp_timer_create(&timer_args, &s_reboot_timer) != ESP_OK) {
            ESP_LOGE(TAG, "reboot timer create failed, rebooting now");
            esp_restart();
            return;
        }
    }
    /* 已经排了一次就不用再排，返回值无害 */
    if (esp_timer_is_active(s_reboot_timer)) {
        return;
    }
    esp_timer_start_once(s_reboot_timer, 500 * 1000);
}

/**
 * 远程指令处理器。运行在 MQTT 事件任务上下文，必须快速返回，
 * 因此这里只做参数校验和状态更新，不做阻塞操作。
 *
 * 作用域说明（与 agent/protocol.py 契约表一致）：
 * - set_interval 是**节点级**的：同一块板上的两个 device_id 共用一个上报周期；
 * - reboot 是节点级的：整台节点重启；
 * - set_threshold 只影响振动链路。
 */
static const char *app_handle_command(const char *device_id,
                                      const app_command_t *cmd, char *detail,
                                      size_t detail_len)
{
    if (strcmp(cmd->action, "set_interval") == 0) {
        if (!cmd->has_value) {
            snprintf(detail, detail_len, "missing value");
            return "error";
        }
        const long ms = (long)cmd->value;
        if (ms < 500 || ms > 60000) {
            snprintf(detail, detail_len, "interval must be 500-60000 ms");
            return "error";
        }
        g_telemetry_period_ms = (uint32_t)ms;
        snprintf(detail, detail_len, "telemetry period=%ld ms (node-wide)", ms);
        return "ok";
    }

    if (strcmp(cmd->action, "set_threshold") == 0) {
        if (!cmd->has_value) {
            snprintf(detail, detail_len, "missing value");
            return "error";
        }
        /* 参数单位 g，允许 0.05~4.0g */
        if (cmd->value < 0.05f || cmd->value > 4.0f) {
            snprintf(detail, detail_len, "threshold must be 0.05-4.0 g");
            return "error";
        }
        if (!g_app.imu_ok) {
            snprintf(detail, detail_len, "imu unavailable, threshold not applied");
            return "error";
        }
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        edge_detector_set_thresholds(&g_detector, cmd->value, cmd->value * 0.6f);
        xSemaphoreGive(g_state_mutex);
        snprintf(detail, detail_len, "threshold=%.2fg (release %.2fg)", cmd->value,
                 cmd->value * 0.6f);
        return "ok";
    }

    if (strcmp(cmd->action, "reboot") == 0) {
        /* 延迟 500ms 重启，让 ack 有机会发出去 */
        reboot_deferred();
        snprintf(detail, detail_len, "rebooting in 500 ms");
        return "ok";
    }

    snprintf(detail, detail_len, "unsupported action: %s (device %s)", cmd->action,
             device_id);
    return "error";
}

static esp_err_t init_i2c_bus(void)
{
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = APP_I2C_SDA_GPIO,
        .scl_io_num = APP_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&bus_cfg, &g_app.i2c_bus);
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== IoT edge node starting ===");
    ESP_LOGI(TAG, "temp device: %s, vib device: %s", APP_TEMP_DEVICE_ID,
             APP_VIB_DEVICE_ID);

    init_nvs();

    /* ---- 共享状态 ---- */
    g_state_mutex = xSemaphoreCreateMutex();
    g_alert_queue = xQueueCreate(8, sizeof(edge_alert_t));
    assert(g_state_mutex != NULL && g_alert_queue != NULL);
    imu_ring_init(&g_imu_ring);
    edge_detector_init(&g_detector, 0.02f,
                       CONFIG_APP_VIB_THRESHOLD_ON_MG / 1000.0f,
                       CONFIG_APP_VIB_THRESHOLD_OFF_MG / 1000.0f);

    /* 默认按"传感器不可用"起步：只有 IMU 初始化成功才降到 NORMAL。
     * 反过来（默认 NORMAL）会让坏掉的传感器上报 rms=0/NORMAL。 */
    g_edge.state = EDGE_STATE_UNKNOWN;
    g_dht.valid = false;

    /* ---- 硬件初始化：任一失败都不阻塞启动，便于分步接线调试 ---- */
    ESP_ERROR_CHECK(init_i2c_bus());

    g_app.imu_ok = (mpu6050_init(g_app.i2c_bus, APP_MPU_I2C_ADDR) == ESP_OK);
    if (!g_app.imu_ok) {
        ESP_LOGE(TAG, "MPU6050 init failed, vibration chain marked UNKNOWN");
    }
    if (dht22_init(APP_DHT_GPIO) != ESP_OK) {
        ESP_LOGE(TAG, "DHT22 init failed");
    }
    if (ssd1306_init(g_app.i2c_bus, APP_OLED_I2C_ADDR) != ESP_OK) {
        ESP_LOGW(TAG, "OLED init failed, continuing headless");
    }

    /* ---- 任务先起来：本地采样/检测/显示不依赖网络 ----
     * 旧实现先同步等 WiFi 最多 30s 再创建任务，"断网时本地检测照常运行"
     * 在"开机就没网"的场景下根本不成立。 */
    if (g_app.imu_ok) {
        task_imu_start();
    } else {
        ESP_LOGW(TAG, "task_imu not started (sensor unavailable)");
    }
    task_detect_start();
    task_dht_start();
    task_net_start();
    task_display_start();
    task_monitor_start();

    /* ---- 网络（异步） ---- */
    if (wifi_mgr_start() != ESP_OK) {
        ESP_LOGE(TAG, "wifi init failed, running offline (edge detection still active)");
    }
    if (time_sync_start() != ESP_OK) {
        ESP_LOGW(TAG, "time sync unavailable, timestamp will be omitted");
    }
    /* 极小概率：time_sync 尚未初始化时已经拿到 IP。这里补一次通知，幂等。 */
    if (wifi_mgr_is_connected()) {
        time_sync_notify_connected();
    }
    if (mqtt_mgr_start(app_handle_command) != ESP_OK) {
        ESP_LOGE(TAG, "mqtt start failed");
    }

    ESP_LOGI(TAG, "=== %d tasks launched, heap free=%u ===", APP_TASK_COUNT,
             (unsigned)esp_get_free_heap_size());

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
