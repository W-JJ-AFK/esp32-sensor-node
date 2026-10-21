/**
 * @file mqtt_mgr.h
 * @brief MQTT 连接管理：每设备一条连接、遗嘱、在线状态、指令订阅与回执。
 *
 * 为什么一个节点要开两条连接：
 * MQTT 3.1.1 的遗嘱（LWT）是**绑定在连接上**的，一个连接只能挂一条遗嘱。
 * 早期实现用单连接代理 temp_sensor_01 与 vibration_01 两个 device_id，
 * 结果节点掉电时只有 temp_sensor_01 被标记 offline，vibration_01 的 retained
 * 状态永远停在 online —— 任何直接订阅 status 的消费者（Grafana、告警转发）
 * 都会看到一台其实已经断电的"在线"设备。模拟器给每台设备都设了独立 LWT，
 * 所以这种不一致只在真机上出现，演示时看不出来。
 *
 * 代价是两条连接（2 个 socket + 2 个 esp-mqtt 任务）。对 S3 的 512KB SRAM
 * 来说完全可接受，换来的是"硬件与模拟器行为一致"这个更重要的属性。
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"
#include "protocol.h"

/**
 * 指令处理回调（在 MQTT 事件任务上下文中调用，必须快速返回）。
 *
 * @param device_id 指令目标设备
 * @param cmd       解析后的指令
 * @param detail    输出缓冲，用于回执中的人类可读说明
 * @param detail_len detail 容量
 * @return "ok" 或 "error"
 */
typedef const char *(*app_cmd_handler_t)(const char *device_id,
                                         const app_command_t *cmd,
                                         char *detail, size_t detail_len);

/** @brief 为节点上的每台设备建一条连接（异步，不阻塞等待连上）。 */
esp_err_t mqtt_mgr_start(app_cmd_handler_t handler);

/** @brief 至少一条连接可用。 */
bool mqtt_mgr_connected(void);

/** @brief 指定设备的连接是否可用。 */
bool mqtt_mgr_device_connected(const char *device_id);

/** @brief 按主题里的 device_id 选择连接发布；该设备离线时返回 INVALID_STATE。 */
esp_err_t mqtt_mgr_publish(const char *topic, const char *payload, int qos,
                           bool retain);
