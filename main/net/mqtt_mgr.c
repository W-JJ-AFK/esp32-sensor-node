#include "mqtt_mgr.h"

#include <stdio.h>
#include <string.h>

#include "common.h"
#include "esp_check.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "time_sync.h"
#include "topic.h"

static const char *TAG = "MQTT";

#define TOPIC_MAX   TOPIC_BUF_LEN
#define PAYLOAD_MAX 384
#define CLIENT_COUNT 2

typedef struct {
    const char *device_id;
    esp_mqtt_client_handle_t client;
    /* 遗嘱的 topic/payload 必须活到客户端销毁，所以放静态区 */
    char lwt_topic[TOPIC_MAX];
    char lwt_payload[PAYLOAD_MAX];
    volatile bool connected;
} node_client_t;

static node_client_t s_clients[CLIENT_COUNT] = {
    {.device_id = APP_TEMP_DEVICE_ID},
    {.device_id = APP_VIB_DEVICE_ID},
};

static app_cmd_handler_t s_handler;

/* ---------------- 索引与查找 ---------------- */

static node_client_t *find_by_device(const char *device_id)
{
    if (device_id == NULL) {
        return NULL;
    }
    for (int i = 0; i < CLIENT_COUNT; i++) {
        if (strcmp(s_clients[i].device_id, device_id) == 0) {
            return &s_clients[i];
        }
    }
    return NULL;
}

/** 从主题中取出 device_id（主题的构造/解析统一由 topic.c 负责）。 */
static bool extract_device_id(const char *topic, char *out, size_t out_len)
{
    return topic_parse_device_id(topic, out, out_len);
}

/* ---------------- 发布 ---------------- */

static void publish_status(node_client_t *nc, bool online)
{
    char topic[TOPIC_MAX];
    char payload[PAYLOAD_MAX];
    if (nc == NULL || nc->client == NULL) {
        return;
    }
    if (topic_build(topic, sizeof(topic), nc->device_id, TOPIC_STATUS) < 0) {
        return;
    }
    if (protocol_build_status(payload, sizeof(payload), nc->device_id, online,
                              time_sync_epoch_s(), time_sync_uptime_s()) > 0) {
        /* retain=1：后启动的 Agent/Dashboard 立刻能拿到在线状态 */
        esp_mqtt_client_publish(nc->client, topic, payload, 0, 1, 1);
    }
}

esp_err_t mqtt_mgr_publish(const char *topic, const char *payload, int qos,
                           bool retain)
{
    char device_id[32];
    node_client_t *nc = NULL;

    if (extract_device_id(topic, device_id, sizeof(device_id))) {
        nc = find_by_device(device_id);
    }
    if (nc == NULL) {
        /* 非 devices/{id}/ 主题：随便找一条可用连接 */
        for (int i = 0; i < CLIENT_COUNT; i++) {
            if (s_clients[i].client != NULL && s_clients[i].connected) {
                nc = &s_clients[i];
                break;
            }
        }
    }
    if (nc == NULL || nc->client == NULL || !nc->connected) {
        return ESP_ERR_INVALID_STATE;
    }

    const int msg_id = esp_mqtt_client_publish(nc->client, topic, payload, 0, qos,
                                               retain ? 1 : 0);
    return (msg_id < 0) ? ESP_FAIL : ESP_OK;
}

/* ---------------- 下行指令 ---------------- */

static void handle_command_message(node_client_t *nc, const char *topic,
                                   const char *data, int data_len)
{
    char device_id[32];
    if (!extract_device_id(topic, device_id, sizeof(device_id))) {
        return;
    }
    /* 再确认一次"这条主题确实是给这台设备的指令"：
     * topic_parse_device_id 只回答"形态是否认识"，devices/{id}/cmd/ack 也认识；
     * 若有人把回执主题误订阅进来，这里能挡住"把 ack 当 cmd 执行"。 */
    if (!topic_match(topic, device_id, TOPIC_CMD)) {
        ESP_LOGW(TAG, "ignore non-command topic %s", topic);
        return;
    }

    char json[PAYLOAD_MAX];
    const int copy_len =
        (data_len < (int)sizeof(json) - 1) ? data_len : (int)sizeof(json) - 1;
    memcpy(json, data, (size_t)copy_len);
    json[copy_len] = '\0';

    app_command_t cmd;
    char ack_payload[PAYLOAD_MAX];
    char ack_topic[TOPIC_MAX];
    char detail[96] = "invalid payload";

    if (!protocol_parse_command(json, &cmd)) {
        /* 连 command_id 都没有时无法回执，只能记日志 */
        ESP_LOGW(TAG, "unparsable command on %s", topic);
        return;
    }

    const char *result;
    /* 时效校验：只有在两端都有可信墙上时间时才做。
     * 设备没同步时间时无法判断"过期"，此时放行并依赖 QoS1 + 在线状态。 */
    const int64_t now = time_sync_epoch_s();
    if (cmd.has_expires_at && now > 0 && now > cmd.expires_at) {
        result = "error";
        snprintf(detail, sizeof(detail), "command expired at %lld (now %lld)",
                 (long long)cmd.expires_at, (long long)now);
    } else {
        result = (s_handler != NULL)
                     ? s_handler(device_id, &cmd, detail, sizeof(detail))
                     : "error";
    }

    ESP_LOGI(TAG, "cmd %s -> %s (%s)", cmd.action, result, detail);

    if (protocol_build_ack(ack_payload, sizeof(ack_payload), cmd.command_id, result,
                           detail, time_sync_epoch_s(), time_sync_uptime_s()) > 0) {
        topic_build(ack_topic, sizeof(ack_topic), device_id, TOPIC_ACK);
        esp_mqtt_client_publish(nc->client, ack_topic, ack_payload, 0, 1, 0);
    }
}

/* ---------------- 事件 ---------------- */

/**
 * 事件回调的 arg 是注册时传入的 node_client_t*，
 * 这样同一个处理函数能区分"哪条连接"的事件。
 */
static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t event_id,
                               void *event_data)
{
    (void)base;
    node_client_t *nc = (node_client_t *)arg;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    if (nc == NULL) {
        return;
    }

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED: {
        nc->connected = true;
        ESP_LOGI(TAG, "%s connected to broker", nc->device_id);

        publish_status(nc, true);

        char topic[TOPIC_MAX];
        if (topic_build(topic, sizeof(topic), nc->device_id, TOPIC_CMD) > 0) {
            esp_mqtt_client_subscribe(nc->client, topic, 1);
        }
        break;
    }

    case MQTT_EVENT_DISCONNECTED:
        nc->connected = false;
        ESP_LOGW(TAG, "%s disconnected, esp-mqtt will retry automatically",
                 nc->device_id);
        break;

    case MQTT_EVENT_DATA: {
        if (event->topic == NULL || event->data == NULL) {
            break;
        }
        /* event->topic **不是** NUL 结尾的！
         * esp-mqtt 只给 (topic, topic_len)，后面紧跟着的还是接收缓冲里的其他字节。
         * 直接把 event->topic 当 C 字符串传给 strcmp/strstr，等于读到缓冲区尾部之外：
         * 轻则匹配不上 devices/{id}/cmd 导致指令被静默丢弃，重则越界读。
         * 统一拷贝到本地缓冲再补 '\0'。 */
        char topic_buf[TOPIC_MAX];
        const int topic_len = (event->topic_len < (int)sizeof(topic_buf) - 1)
                                  ? event->topic_len
                                  : (int)sizeof(topic_buf) - 1;
        memcpy(topic_buf, event->topic, (size_t)topic_len);
        topic_buf[topic_len] = '\0';

        /* 指令报文很短，这里只处理未分片的情况 */
        if (event->total_data_len == event->data_len) {
            handle_command_message(nc, topic_buf, event->data, event->data_len);
        } else {
            ESP_LOGW(TAG, "fragmented command ignored (len=%d)", event->total_data_len);
        }
        break;
    }

    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "%s mqtt error, check broker uri / credentials",
                 nc->device_id);
        break;

    default:
        break;
    }
}

esp_err_t mqtt_mgr_start(app_cmd_handler_t handler)
{
    s_handler = handler;

    int started = 0;
    esp_err_t last_err = ESP_FAIL;

    for (int i = 0; i < CLIENT_COUNT; i++) {
        node_client_t *nc = &s_clients[i];

        topic_build(nc->lwt_topic, sizeof(nc->lwt_topic), nc->device_id, TOPIC_STATUS);
        protocol_build_status(nc->lwt_payload, sizeof(nc->lwt_payload),
                              nc->device_id, false, 0, 0);

        const esp_mqtt_client_config_t cfg = {
            .broker.address.uri = CONFIG_APP_MQTT_BROKER_URI,
            /* client_id 必须唯一，否则两条连接会互相踢下线 */
            .credentials.client_id = nc->device_id,
            .session = {
                .keepalive = 30,
                .last_will = {
                    .topic = nc->lwt_topic,
                    .msg = nc->lwt_payload,
                    .msg_len = 0,
                    .qos = 1,
                    .retain = 1,
                },
            },
            .network = {
                .reconnect_timeout_ms = 2000,
                .disable_auto_reconnect = false,
            },
        };

        nc->client = esp_mqtt_client_init(&cfg);
        if (nc->client == NULL) {
            last_err = ESP_ERR_NO_MEM;
            continue;
        }
        if (esp_mqtt_client_register_event(nc->client, ESP_EVENT_ANY_ID,
                                           mqtt_event_handler, nc) != ESP_OK) {
            last_err = ESP_FAIL;
            continue;
        }
        if (esp_mqtt_client_start(nc->client) != ESP_OK) {
            last_err = ESP_FAIL;
            continue;
        }
        started++;
    }

    if (started == 0) {
        ESP_LOGE(TAG, "no mqtt client started");
        return last_err;
    }
    ESP_LOGI(TAG, "%d mqtt clients started, broker=%s", started,
             CONFIG_APP_MQTT_BROKER_URI);
    return ESP_OK;
}

bool mqtt_mgr_connected(void)
{
    for (int i = 0; i < CLIENT_COUNT; i++) {
        if (s_clients[i].connected) {
            return true;
        }
    }
    return false;
}

bool mqtt_mgr_device_connected(const char *device_id)
{
    const node_client_t *nc = find_by_device(device_id);
    return (nc != NULL) && nc->connected;
}
