#include "wifi_mgr.h"

#include <string.h>

#include "esp_check.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "time_sync.h"

static const char *TAG = "WIFI";

#define WIFI_CONNECTED_BIT BIT0

/* 退避：1s、2s、4s、8s、16s、32s，之后固定 32s 重试一次，永不放弃。
 * 为什么不是"重试 10 次就放弃"：10 次退避前的实现没有延时，
 * 几秒内就把配额烧完了，路由器重启超过 10 秒节点就再也不会回线，
 * 只能靠人工复位——这与"断网后自动恢复"的承诺直接矛盾。 */
#define WIFI_BACKOFF_BASE_MS 500u
#define WIFI_BACKOFF_SHIFT_MAX 6u

static EventGroupHandle_t s_events;
static esp_timer_handle_t s_retry_timer;
static volatile bool s_connected;
static volatile uint32_t s_retry;
static bool s_started;

static void retry_timer_cb(void *arg)
{
    (void)arg;
    /* 定时器回调里可以调用，esp_wifi_connect 本身不阻塞 */
    esp_wifi_connect();
}

static void schedule_reconnect(void)
{
    const uint32_t retry = ++s_retry;
    const uint32_t shift = (retry < WIFI_BACKOFF_SHIFT_MAX) ? retry : WIFI_BACKOFF_SHIFT_MAX;
    uint32_t delay_ms = WIFI_BACKOFF_BASE_MS << shift;
    if (delay_ms > 60000u) {
        delay_ms = 60000u;
    }

    ESP_LOGW(TAG, "disconnected, reconnect #%u in %u ms", (unsigned)retry,
             (unsigned)delay_ms);

    /* 事件回调里不能阻塞，所以用单次定时器做退避；重复触发时先停掉旧的 */
    esp_timer_stop(s_retry_timer);
    esp_timer_start_once(s_retry_timer, (uint64_t)delay_ms * 1000ull);
}

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        s_connected = false;
        xEventGroupClearBits(s_events, WIFI_CONNECTED_BIT);
        schedule_reconnect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = (const ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry = 0;
        s_connected = true;
        xEventGroupSetBits(s_events, WIFI_CONNECTED_BIT);
        time_sync_notify_connected();
    }
}

esp_err_t wifi_mgr_start(void)
{
    if (s_started) {
        return ESP_OK;
    }

    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const esp_timer_create_args_t timer_args = {
        .callback = retry_timer_cb,
        .name = "wifi_retry",
    };
    ESP_RETURN_ON_ERROR(esp_timer_create(&timer_args, &s_retry_timer), TAG,
                        "create retry timer failed");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "wifi init failed");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler,
                            NULL, NULL),
                        TAG, "register wifi handler failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
                            IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler,
                            NULL, NULL),
                        TAG, "register ip handler failed");

    wifi_config_t wifi_cfg = {0};
    strncpy((char *)wifi_cfg.sta.ssid, CONFIG_APP_WIFI_SSID,
            sizeof(wifi_cfg.sta.ssid) - 1);
    strncpy((char *)wifi_cfg.sta.password, CONFIG_APP_WIFI_PASSWORD,
            sizeof(wifi_cfg.sta.password) - 1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    /* WPA2/WPA3 过渡模式（很多家用路由的默认设置）要求 STA 声明 PMF 能力，
     * 否则关联会被 AP 以 reason=205/PERMANENTLY_REMOVED 拒绝；
     * required=false 保证只支持 WPA2 的老路由也能连上。 */
    wifi_cfg.sta.pmf_cfg.capable = true;
    wifi_cfg.sta.pmf_cfg.required = false;

    /* 出厂默认凭据没改时给出明确指引：否则用户看到的是无尽的
     * "disconnected, reconnect #N"，很难联想到"根本没配 WiFi"。 */
    if (strcmp(CONFIG_APP_WIFI_SSID, "YOUR_WIFI_SSID") == 0) {
        ESP_LOGE(TAG, "WiFi SSID is still the placeholder: run 'idf.py menuconfig' ->");
        ESP_LOGE(TAG, "  Edge Node Configuration -> WiFi SSID / password, "
                      "then rebuild and re-flash");
    }

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg), TAG,
                        "set config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");

    s_started = true;
    ESP_LOGI(TAG, "connecting to \"%s\" (async, edge detection runs regardless)",
             CONFIG_APP_WIFI_SSID);

    /* 注意：这里不等待连接完成。
     * 旧实现在 app_main 里同步等最多 30s，WiFi 不可用时边缘检测要等 30s 才开始跑，
     * 与"断网时本地检测照常运行"矛盾。联网状态由 task_net 每次发送前查询。 */
    return ESP_OK;
}

bool wifi_mgr_wait_connected(uint32_t timeout_ms)
{
    if (s_events == NULL) {
        return false;
    }
    if (s_connected) {
        return true;
    }
    const EventBits_t bits = xEventGroupWaitBits(
        s_events, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(timeout_ms));
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

bool wifi_mgr_is_connected(void)
{
    return s_connected;
}
