#include "topic.h"

#include <string.h>

#define TOPIC_PREFIX      "devices/"
#define TOPIC_PREFIX_LEN  (sizeof(TOPIC_PREFIX) - 1)

static const char *const k_suffix[TOPIC_KIND_COUNT] = {
    [TOPIC_TELEMETRY] = "telemetry",
    [TOPIC_STATUS] = "status",
    [TOPIC_ALERT] = "alert",
    [TOPIC_CMD] = "cmd",
    [TOPIC_ACK] = "cmd/ack",
};

const char *topic_kind_suffix(topic_kind_t kind)
{
    if ((int)kind < 0 || kind >= TOPIC_KIND_COUNT) {
        return NULL;
    }
    return k_suffix[kind];
}

int topic_build(char *buf, size_t len, const char *device_id, topic_kind_t kind)
{
    const char *suffix = topic_kind_suffix(kind);
    if (buf == NULL || len == 0 || suffix == NULL || device_id == NULL) {
        return -1;
    }
    /* 空 ID 或含 '/' 的 ID 会让主题变成 devices//cmd 这种谁都不匹配的形态，
     * 静默发布出去只会白耗流量，这里直接拒绝。 */
    if (device_id[0] == '\0' || strchr(device_id, '/') != NULL) {
        return -1;
    }

    const size_t id_len = strlen(device_id);
    const size_t need = TOPIC_PREFIX_LEN + id_len + 1 + strlen(suffix) + 1;
    if (need > len) {
        return -1;
    }

    char *p = buf;
    memcpy(p, TOPIC_PREFIX, TOPIC_PREFIX_LEN);
    p += TOPIC_PREFIX_LEN;
    memcpy(p, device_id, id_len);
    p += id_len;
    *p++ = '/';
    const size_t suffix_len = strlen(suffix);
    memcpy(p, suffix, suffix_len);
    p += suffix_len;
    *p = '\0';

    return (int)(p - buf);
}

bool topic_parse_device_id(const char *topic, char *out, size_t out_len)
{
    if (topic == NULL || out == NULL || out_len == 0) {
        return false;
    }
    if (strncmp(topic, TOPIC_PREFIX, TOPIC_PREFIX_LEN) != 0) {
        return false;
    }

    const char *start = topic + TOPIC_PREFIX_LEN;
    const char *slash = strchr(start, '/');
    if (slash == NULL || slash == start) {
        return false; /* "devices/" 后面没东西，或 "devices//..." */
    }

    /* 斜杠之后必须是已知后缀，且**整段相等**（strcmp 而非 strstr）：
     * devices/a/cmd/ack 的尾部是 "cmd/ack"，与 "cmd" 不相等，
     * 因此它不会被当成设备 a 的指令主题；反过来 ack 主题能正确匹配 ack。 */
    const char *suffix = slash + 1;
    bool matched = false;
    for (int i = 0; i < (int)TOPIC_KIND_COUNT; i++) {
        if (strcmp(suffix, k_suffix[i]) == 0) {
            matched = true;
            break;
        }
    }
    if (!matched) {
        return false;
    }

    const size_t id_len = (size_t)(slash - start);
    if (id_len >= out_len) {
        return false;
    }
    memcpy(out, start, id_len);
    out[id_len] = '\0';
    return true;
}

bool topic_match(const char *topic, const char *device_id, topic_kind_t kind)
{
    const char *suffix = topic_kind_suffix(kind);
    if (topic == NULL || device_id == NULL || suffix == NULL || device_id[0] == '\0') {
        return false;
    }
    if (strncmp(topic, TOPIC_PREFIX, TOPIC_PREFIX_LEN) != 0) {
        return false;
    }

    const char *p = topic + TOPIC_PREFIX_LEN;
    const size_t id_len = strlen(device_id);
    if (strncmp(p, device_id, id_len) != 0) {
        return false;
    }
    if (p[id_len] != '/') {
        return false;
    }
    /* 后缀必须整段相等：这样 "cmd/ack" 不会被当成 "cmd" 的前缀匹配成功 */
    return strcmp(p + id_len + 1, suffix) == 0;
}
