#include "tasks.h"

#include <stdio.h>

#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "mqtt_mgr.h"
#include "protocol.h"
#include "time_sync.h"
#include "topic.h"

static const char *TAG = "TASK_NET";

#define TOPIC_LEN   TOPIC_BUF_LEN
#define PAYLOAD_LEN 384

/* 主循环轮询间隔：既是遥测节拍的最小粒度，也是补发的最大延迟 */
#define NET_POLL_MS 200
/* 每次循环最多补发几条，避免重连瞬间一次性灌爆 outbox */
#define NET_BACKLOG_BURST 4

/* 断网期间的告警缓存。64 条 × 24 字节 ≈ 1.5KB，覆盖 30 分钟故障足够。
 * 为什么必须有它：告警是从队列里"取出来之后"才判断网络的，取出来就没了；
 * 而队列只有 8 格，断网几分钟就丢光——恰恰是最该保留的时候。
 * 环形覆盖式淘汰（丢最旧）比"满了就丢新"更合理：新旧都留着没用，
 * 但近期的告警对定位问题的价值更高。 */
#define ALERT_BACKLOG_SIZE 64

static edge_alert_t s_backlog[ALERT_BACKLOG_SIZE];
static size_t s_backlog_rd;
static size_t s_backlog_wr;
static size_t s_backlog_count;
static uint32_t s_backlog_dropped;

/* 本次开机的随机标记，用于保证 alert_id 跨重启不重复。
 * alert_id 里原本只有"开机毫秒数 + rms"，重启后开机毫秒会从 0 重新数，
 * 于是同一块板子重启后的第一条告警很可能与上一轮某条告警 ID 完全相同：
 * Agent 侧按 alert_id 幂等落库，这条真实的新告警会被当成重复静默丢弃。
 * 开机时取一次随机数即可，同一进程内保持不变 —— 补发同一条积压告警时 ID 依然稳定。 */
static uint32_t s_boot_tag;

static uint32_t boot_tag(void)
{
    if (s_boot_tag == 0) {
        s_boot_tag = (esp_random() % 0xFFFFFFu) + 1u;
    }
    return s_boot_tag;
}

/* ---------------- 告警缓存 ---------------- */

static void backlog_push(const edge_alert_t *a)
{
    if (s_backlog_count == ALERT_BACKLOG_SIZE) {
        s_backlog_rd = (s_backlog_rd + 1) % ALERT_BACKLOG_SIZE;
        s_backlog_count--;
        s_backlog_dropped++;
    }
    s_backlog[s_backlog_wr] = *a;
    s_backlog_wr = (s_backlog_wr + 1) % ALERT_BACKLOG_SIZE;
    s_backlog_count++;
}

static bool backlog_peek(edge_alert_t *out)
{
    if (s_backlog_count == 0) {
        return false;
    }
    *out = s_backlog[s_backlog_rd];
    return true;
}

static void backlog_drop_oldest(void)
{
    if (s_backlog_count == 0) {
        return;
    }
    s_backlog_rd = (s_backlog_rd + 1) % ALERT_BACKLOG_SIZE;
    s_backlog_count--;
}

uint32_t task_net_backlog_depth(void)
{
    return (uint32_t)s_backlog_count;
}

uint32_t task_net_backlog_dropped(void)
{
    return s_backlog_dropped;
}

/* ---------------- 发布 ---------------- */

static esp_err_t publish_alert(const edge_alert_t *alert)
{
    char alert_id[64];
    char topic[TOPIC_LEN];
    char payload[PAYLOAD_LEN];

    /* alert_id 用「本次开机随机标记 + 开机毫秒数 + 量化后的 rms」构造：
     * 同一进程内重连补发时 ID 不变（Agent 按 alert_id 去重，不会把补发算成新告警），
     * 但跨重启一定不同（见 boot_tag 的注释）。 */
    snprintf(alert_id, sizeof(alert_id), "%s-%06x-%lld-%d", APP_VIB_DEVICE_ID,
             (unsigned)boot_tag(), (long long)alert->ts_ms,
             (int)(alert->rms * 1000.0f));
    if (topic_build(topic, sizeof(topic), APP_VIB_DEVICE_ID, TOPIC_ALERT) < 0) {
        return ESP_FAIL;
    }

    /* 严重级别只有 critical 一种：WARNING 状态刻意不发告警（只是"越过阈值、
     * 尚未确认"），所以这里不存在 warning 分支——旧代码的三元表达式是死代码。 */
    const int n = protocol_build_alert(
        payload, sizeof(payload), APP_VIB_DEVICE_ID, alert_id, "vibration_high",
        alert->rms, alert->threshold, "critical", time_sync_epoch_s(),
        time_sync_uptime_s());

    if (n <= 0) {
        return ESP_FAIL;
    }
    return mqtt_mgr_publish(topic, payload, 1, false);
}

static void publish_telemetry(void)
{
    dht_snapshot_t dht;
    edge_snapshot_t edge;

    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    dht = g_dht;
    edge = g_edge;
    g_edge.peak = 0.0f; /* 峰值按上报周期统计 */
    xSemaphoreGive(g_state_mutex);

    const int64_t epoch = time_sync_epoch_s();
    const uint32_t uptime = time_sync_uptime_s();
    char topic[TOPIC_LEN];
    char payload[PAYLOAD_LEN];

    /* 温度：读失败时 valid=false 且**不发数值**。
     * 以前发 0.0 会让上层把"传感器坏了"看成"稳定在 0°C"，0%RH 更是物理不可能。 */
    if (topic_build(topic, sizeof(topic), APP_TEMP_DEVICE_ID, TOPIC_TELEMETRY) > 0 &&
        protocol_build_telemetry_env(payload, sizeof(payload), APP_TEMP_DEVICE_ID,
                                     APP_TEMP_DEVICE_TYPE, epoch, uptime,
                                     dht.valid, dht.temperature_c,
                                     dht.humidity_pct) > 0) {
        (void)mqtt_mgr_publish(topic, payload, 0, false);
    }

    /* 振动：IMU 不可用时 state=UNKNOWN + valid=false，而不是 rms=0/NORMAL */
    const bool vib_valid = (edge.state != EDGE_STATE_UNKNOWN);
    static uint32_t seq = 0;
    if (topic_build(topic, sizeof(topic), APP_VIB_DEVICE_ID, TOPIC_TELEMETRY) > 0 &&
        protocol_build_telemetry_vib(payload, sizeof(payload), APP_VIB_DEVICE_ID,
                                     APP_VIB_DEVICE_TYPE, epoch, uptime, vib_valid,
                                     seq++, edge.rms, edge.peak,
                                     edge_state_name(edge.state)) > 0) {
        (void)mqtt_mgr_publish(topic, payload, 0, false);
    }
}

/* ---------------- 任务 ---------------- */

/**
 * 网络任务：事件驱动上报。
 * - 告警：立刻发，QoS1；发不出去就进 backlog，重连后按序补发
 * - 遥测：按 g_telemetry_period_ms 周期发，QoS0（可被远程指令动态调整）
 *
 * 判定网络的时机在"取出告警之前"不再有意义，所以这里改成"先尝试发布、
 * 失败才缓存"——不再有"取出来发现断网就丢掉"的窗口。
 */
static void net_task(void *arg)
{
    (void)arg;
    TickType_t last_publish = xTaskGetTickCount();
    bool backlog_warned = false;

    for (;;) {
        /* 1) 先把断网期间攒下的告警补发出去（每次限流，避免灌爆 outbox） */
        if (mqtt_mgr_connected() && s_backlog_count > 0) {
            if (!backlog_warned) {
                ESP_LOGI(TAG, "flushing %u cached alerts", (unsigned)s_backlog_count);
                backlog_warned = true;
            }
            for (int i = 0; i < NET_BACKLOG_BURST; i++) {
                edge_alert_t pending;
                if (!backlog_peek(&pending)) {
                    break;
                }
                if (publish_alert(&pending) != ESP_OK) {
                    break;
                }
                backlog_drop_oldest();
            }
            if (s_backlog_count == 0) {
                backlog_warned = false;
            }
        }

        /* 2) 新告警：发出去了才算完成，否则进缓存 */
        edge_alert_t alert;
        if (xQueueReceive(g_alert_queue, &alert, pdMS_TO_TICKS(NET_POLL_MS)) == pdPASS) {
            if (publish_alert(&alert) != ESP_OK) {
                backlog_push(&alert);
                if (!backlog_warned) {
                    ESP_LOGW(TAG, "mqtt offline, alert cached (%u waiting)",
                             (unsigned)s_backlog_count);
                    backlog_warned = true;
                }
            }
        }

        /* 3) 周期遥测 */
        const TickType_t period = pdMS_TO_TICKS(g_telemetry_period_ms);
        if ((xTaskGetTickCount() - last_publish) >= period) {
            last_publish = xTaskGetTickCount();
            if (mqtt_mgr_connected()) {
                publish_telemetry();
            } else {
                ESP_LOGD(TAG, "mqtt offline, telemetry skipped");
            }
        }
    }
}

void task_net_start(void)
{
    xTaskCreate(net_task, "task_net", APP_STACK_NET, NULL, APP_PRIO_NET,
                &g_task_handles[APP_TASK_NET]);
}
