#include "time_sync.h"

#include <time.h>

#include "esp_log.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"

static const char *TAG = "TIME";

/* 只要在这之后，基本可以确定是"同步过的真实时间"而不是 1970 附近 */
#define TIME_VALID_EPOCH_S 1600000000LL /* 2020-09-13 */
#define TIME_SNTP_SERVER   CONFIG_APP_SNTP_SERVER

static bool s_initialized;
static bool s_started;

esp_err_t time_sync_start(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (TIME_SNTP_SERVER[0] == '\0') {
        /* 留空 = 明确禁用。上报层会因为 epoch 恒为 0 而省略 timestamp，
         * 由 Agent 回退到接收时间——功能不受影响，只是少了设备侧时间。 */
        ESP_LOGW(TAG, "sntp disabled by config, timestamp will be omitted");
        return ESP_OK;
    }

    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(TIME_SNTP_SERVER);
    /* 先不启动：还没联网时启动只会不断失败刷日志，等 GOT_IP 再起 */
    cfg.start = false;

    const esp_err_t err = esp_netif_sntp_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "sntp init failed: %s (timestamp 将缺失)", esp_err_to_name(err));
        return err;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "sntp ready, server=%s", TIME_SNTP_SERVER);
    return ESP_OK;
}

void time_sync_notify_connected(void)
{
    if (!s_initialized || s_started) {
        return;
    }
    if (esp_netif_sntp_start() == ESP_OK) {
        s_started = true;
        ESP_LOGI(TAG, "sntp started, waiting for first sync");
    }
}

bool time_sync_valid(void)
{
    if (!s_started) {
        return false;
    }
    return (int64_t)time(NULL) >= TIME_VALID_EPOCH_S;
}

int64_t time_sync_epoch_s(void)
{
    return time_sync_valid() ? (int64_t)time(NULL) : 0;
}

uint32_t time_sync_uptime_s(void)
{
    const int64_t us = esp_timer_get_time();
    return (us > 0) ? (uint32_t)(us / 1000000) : 0u;
}
