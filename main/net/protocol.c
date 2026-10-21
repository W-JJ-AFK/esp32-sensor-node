#include "protocol.h"

#include <string.h>

#include "cJSON.h"
#include "esp_log.h"

static const char *TAG = "PROTOCOL";

#define CMD_ID_MAX_LEN  47
#define CMD_ACTION_MAX_LEN 23

static int finish(cJSON *root, char *buf, size_t len)
{
    if (root == NULL || buf == NULL || len == 0) {
        cJSON_Delete(root);
        return -1;
    }
    /* PrintPreallocated 不分配内存，适合 6KB 栈的任务 */
    const int ok = cJSON_PrintPreallocated(root, buf, (int)len, 0);
    const int written = ok ? (int)strlen(buf) : -1;
    cJSON_Delete(root);
    if (!ok) {
        ESP_LOGE(TAG, "serialize failed, buffer too small (%u)", (unsigned)len);
    }
    return written;
}

/**
 * 公共字段：device_id / device_type / timestamp(可选) / uptime_s。
 *
 * timestamp 只在拿到可信墙上时间时写入；缺失即"设备未同步"，
 * 由 Agent 回退到本地接收时间（见 agent/protocol.py:resolve_timestamp）。
 */
static void add_common(cJSON *root, const char *device_id, const char *device_type,
                       int64_t ts_epoch, uint32_t uptime_s)
{
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "device_type", device_type);
    if (ts_epoch > 0) {
        cJSON_AddNumberToObject(root, "timestamp", (double)ts_epoch);
    }
    cJSON_AddNumberToObject(root, "uptime_s", (double)uptime_s);
}

int protocol_build_telemetry_env(char *buf, size_t len, const char *device_id,
                                 const char *device_type, int64_t ts_epoch,
                                 uint32_t uptime_s, bool valid,
                                 float temperature, float humidity)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return -1;
    }
    add_common(root, device_id, device_type, ts_epoch, uptime_s);

    cJSON *data = cJSON_AddObjectToObject(root, "data");
    if (valid) {
        cJSON_AddNumberToObject(data, "temperature", temperature);
        cJSON_AddNumberToObject(data, "humidity", humidity);
    } else {
        /* 传感器失效时**不发**数值：0°C / 0%RH 会被上层当成真实读数，
         * 0%RH 还是物理上不可能的值。显式 valid=false 才能让运维看出来。 */
        cJSON_AddFalseToObject(data, "valid");
    }

    return finish(root, buf, len);
}

int protocol_build_telemetry_vib(char *buf, size_t len, const char *device_id,
                                 const char *device_type, int64_t ts_epoch,
                                 uint32_t uptime_s, bool valid, uint32_t seq,
                                 float rms, float peak, const char *state)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return -1;
    }
    add_common(root, device_id, device_type, ts_epoch, uptime_s);
    cJSON_AddNumberToObject(root, "seq", (double)seq);

    cJSON *data = cJSON_AddObjectToObject(root, "data");
    if (valid) {
        cJSON_AddNumberToObject(data, "rms", rms);
        cJSON_AddNumberToObject(data, "peak", peak);
        cJSON_AddStringToObject(data, "state", state);
    } else {
        /* IMU 没起来时不能发 rms=0/state=NORMAL —— 那看起来是"全车间最健康的设备" */
        cJSON_AddFalseToObject(data, "valid");
        cJSON_AddStringToObject(data, "state", "UNKNOWN");
    }

    return finish(root, buf, len);
}

int protocol_build_alert(char *buf, size_t len, const char *device_id,
                         const char *alert_id, const char *alert_type,
                         float value, float threshold, const char *severity,
                         int64_t ts_epoch, uint32_t uptime_s)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return -1;
    }
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "alert_id", alert_id);
    cJSON_AddStringToObject(root, "alert_type", alert_type);
    cJSON_AddNumberToObject(root, "value", value);
    cJSON_AddNumberToObject(root, "threshold", threshold);
    cJSON_AddStringToObject(root, "severity", severity);
    if (ts_epoch > 0) {
        cJSON_AddNumberToObject(root, "timestamp", (double)ts_epoch);
    }
    cJSON_AddNumberToObject(root, "uptime_s", (double)uptime_s);

    return finish(root, buf, len);
}

int protocol_build_status(char *buf, size_t len, const char *device_id,
                          bool online, int64_t ts_epoch, uint32_t uptime_s)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return -1;
    }
    cJSON_AddStringToObject(root, "device_id", device_id);
    cJSON_AddStringToObject(root, "state", online ? "online" : "offline");
    if (ts_epoch > 0) {
        cJSON_AddNumberToObject(root, "timestamp", (double)ts_epoch);
    }
    cJSON_AddNumberToObject(root, "uptime_s", (double)uptime_s);
    return finish(root, buf, len);
}

int protocol_build_ack(char *buf, size_t len, const char *command_id,
                       const char *result, const char *detail,
                       int64_t ts_epoch, uint32_t uptime_s)
{
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        return -1;
    }
    cJSON_AddStringToObject(root, "command_id", command_id);
    cJSON_AddStringToObject(root, "result", result);
    if (detail != NULL && detail[0] != '\0') {
        cJSON_AddStringToObject(root, "detail", detail);
    }
    if (ts_epoch > 0) {
        cJSON_AddNumberToObject(root, "timestamp", (double)ts_epoch);
    }
    cJSON_AddNumberToObject(root, "uptime_s", (double)uptime_s);
    return finish(root, buf, len);
}

bool protocol_parse_command(const char *json, app_command_t *out)
{
    if (json == NULL || out == NULL) {
        return false;
    }

    cJSON *root = cJSON_Parse(json);
    if (root == NULL) {
        ESP_LOGW(TAG, "command is not valid JSON");
        return false;
    }

    bool ok = false;
    const cJSON *cid = cJSON_GetObjectItemCaseSensitive(root, "command_id");
    const cJSON *action = cJSON_GetObjectItemCaseSensitive(root, "action");
    const cJSON *value = cJSON_GetObjectItemCaseSensitive(root, "value");
    const cJSON *expires = cJSON_GetObjectItemCaseSensitive(root, "expires_at");

    const bool cid_ok = cJSON_IsString(cid) && cid->valuestring != NULL &&
                        cid->valuestring[0] != '\0' &&
                        strlen(cid->valuestring) <= CMD_ID_MAX_LEN;
    const bool action_ok = cJSON_IsString(action) && action->valuestring != NULL &&
                           action->valuestring[0] != '\0' &&
                           strlen(action->valuestring) <= CMD_ACTION_MAX_LEN;
    /* value 存在时必须是数字：类型错误不该被静默当成"无参数"，
     * 否则 "value": "800"（字符串）会走到"缺少 value"分支，误导排查方向。 */
    const bool value_ok = (value == NULL) || cJSON_IsNumber(value);

    if (cid_ok && action_ok && value_ok) {
        memset(out, 0, sizeof(*out));
        strncpy(out->command_id, cid->valuestring, sizeof(out->command_id) - 1);
        strncpy(out->action, action->valuestring, sizeof(out->action) - 1);
        if (cJSON_IsNumber(value)) {
            out->has_value = true;
            out->value = (float)value->valuedouble;
        }
        if (cJSON_IsNumber(expires)) {
            out->has_expires_at = true;
            out->expires_at = (int64_t)expires->valuedouble;
        }
        ok = true;
    } else {
        ESP_LOGW(TAG, "command structure invalid (cid=%d action=%d value=%d)",
                 (int)cid_ok, (int)action_ok, (int)value_ok);
    }

    cJSON_Delete(root);
    return ok;
}
