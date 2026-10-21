/**
 * @file protocol.h
 * @brief MQTT 报文契约的编解码实现（与 agent/protocol.py 严格对应）。
 *
 * 主题约定：
 *   devices/{id}/telemetry   QoS0  遥测
 *   devices/{id}/status      QoS1  retain  在线状态（LWT）
 *   devices/{id}/alert       QoS1  告警
 *   devices/{id}/cmd         QoS1  指令（下行）
 *   devices/{id}/cmd/ack     QoS1  执行回执
 *
 * 字段契约（改这里必须同步改 agent/protocol.py 的同一张表）：
 *
 *   | 字段      | 时基 / 取值           | 缺省含义                     |
 *   |-----------|-----------------------|------------------------------|
 *   | timestamp | Unix epoch 秒         | **字段缺失** = 设备未同步时间 |
 *   | uptime_s  | 开机秒数（单调）      | 必定存在，仅供排查时序        |
 *   | valid     | bool                  | 字段缺失 = 该帧数据有效        |
 *   | expires_at| Unix epoch 秒（指令） | 字段缺失 = 不做时效校验        |
 *   | state     | NORMAL/WARNING/ALARM/UNKNOWN | UNKNOWN = 传感器不可用 |
 *
 * 为什么 timestamp 缺失而不是填 0：Agent 无法区分"1970 年的告警"和"设备没同步"，
 * 一旦把 0 当时间戳落库，索引和展示全错。缺失是明确信号，回退到本地接收时间。
 *
 * 所有 build 函数返回写入的字节数（不含结尾 0），失败返回 -1。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    char    command_id[48];
    char    action[24];
    float   value;
    bool    has_value;
    bool    has_expires_at;
    int64_t expires_at; /* epoch 秒；has_expires_at 为 false 时不校验 */
} app_command_t;

int protocol_build_telemetry_env(char *buf, size_t len, const char *device_id,
                                 const char *device_type, int64_t ts_epoch,
                                 uint32_t uptime_s, bool valid,
                                 float temperature, float humidity);

int protocol_build_telemetry_vib(char *buf, size_t len, const char *device_id,
                                 const char *device_type, int64_t ts_epoch,
                                 uint32_t uptime_s, bool valid, uint32_t seq,
                                 float rms, float peak, const char *state);

int protocol_build_alert(char *buf, size_t len, const char *device_id,
                         const char *alert_id, const char *alert_type,
                         float value, float threshold, const char *severity,
                         int64_t ts_epoch, uint32_t uptime_s);

int protocol_build_status(char *buf, size_t len, const char *device_id,
                          bool online, int64_t ts_epoch, uint32_t uptime_s);

int protocol_build_ack(char *buf, size_t len, const char *command_id,
                       const char *result, const char *detail,
                       int64_t ts_epoch, uint32_t uptime_s);

bool protocol_parse_command(const char *json, app_command_t *out);
