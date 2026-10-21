/**
 * @file topic.h
 * @brief MQTT 主题的**唯一**构造与解析入口。
 *
 * 主题字符串是跨仓库契约里最容易出错、又最难在真机上发现的一环：
 * 「devices/{id}/cmd/ack」少一个字符、多一个斜杠，订阅照样成功、
 * 发布照样成功，只是对面永远收不到——没有任何报错。
 *
 * 因此这里把五个主题收敛成一组纯 C 函数：
 *   - 写入侧只用 topic_build()，不再到处 snprintf("devices/%s/...")；
 *   - 解析侧只用 topic_parse_device_id()，它要求**整段精确匹配**，
 *     devices/a/cmd/ack 不会被误判成 devices/a/cmd；
 *   - 因为是纯 C（不依赖 ESP-IDF），可以直接在主机上跑断言，
 *     并与 agent/protocol.py 的同一张表交叉核对（见 test/test_edge_detect.c）。
 *
 * 主题矩阵（改这里必须同步改 agent/protocol.py）：
 *
 *   devices/{id}/telemetry   QoS0        遥测
 *   devices/{id}/status      QoS1 retain 在线状态（含 LWT 遗嘱）
 *   devices/{id}/alert       QoS1        告警
 *   devices/{id}/cmd         QoS1        指令（下行）
 *   devices/{id}/cmd/ack     QoS1        执行回执
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

/** 主题种类。数值仅用于数组下标，不要持久化。 */
typedef enum {
    TOPIC_TELEMETRY = 0,
    TOPIC_STATUS,
    TOPIC_ALERT,
    TOPIC_CMD,
    TOPIC_ACK,
    TOPIC_KIND_COUNT,
} topic_kind_t;

/**
 * @brief 存放一条完整主题所需的缓冲长度（含结尾 0）。
 *
 * 唯一出处。以前 mqtt_mgr.c 里叫 TOPIC_MAX、task_net.c 里叫 TOPIC_LEN，
 * 两个 96 各写一遍——哪天要放宽 device_id 长度、只改了其中一处，
 * 另一处就会在 topic_build 里静默返回 -1，主题留空。
 */
#define TOPIC_BUF_LEN 96

/** @brief 种类的后缀名，例如 TOPIC_ACK -> "cmd/ack"。 */
const char *topic_kind_suffix(topic_kind_t kind);

/**
 * @brief 构造 "devices/{device_id}/{suffix}"。
 *
 * @param buf       输出缓冲
 * @param len       缓冲容量（含结尾 0）
 * @param device_id 设备 ID，不能为空且不能含 '/'
 * @param kind      主题种类
 * @return 写入的字符数（不含结尾 0）；参数非法或缓冲不足返回 -1
 */
int topic_build(char *buf, size_t len, const char *device_id, topic_kind_t kind);

/**
 * @brief 从主题里取出 device_id。
 *
 * 只接受完整匹配五种已知形态的主题；其它一律返回 false。
 * 这样 "devices/a/cmd/ack" 不会被当成设备 a 的指令主题，
 * 也不会把超长 device_id 截断后当成"另一台设备"。
 *
 * @return true 表示解析成功
 */
bool topic_parse_device_id(const char *topic, char *out, size_t out_len);

/**
 * @brief 判断主题是否**恰好**等于 devices/{device_id}/{kind}。
 *
 * 与 topic_parse_device_id 的区别：后者只回答"是不是我们认识的形态"，
 * 前者还要求设备与种类都对得上。收指令前用它再确认一次，
 * 可以保证 ack 主题、别的设备的主题都不会被误执行。
 */
bool topic_match(const char *topic, const char *device_id, topic_kind_t kind);
