/**
 * @file time_sync.h
 * @brief 墙上时间（Unix epoch）来源。
 *
 * 解决什么问题：ESP32 没有 RTC 电池，复位后 `time(NULL)` 从 1970 开始。
 * 早期固件把「开机秒数」直接塞进 `timestamp` 字段，Agent 又按 epoch 落库，
 * 于是真机告警在数据库里全变成 1970 年——模拟器演示正常，接上硬件才炸。
 *
 * 现在的约定（与 agent/protocol.py 的契约表一致）：
 *   - `timestamp` 只在 SNTP 同步成功后出现，语义严格是 Unix epoch 秒；
 *   - `uptime_s`  永远存在，语义是开机秒数，用于排查时序，不可当 epoch 用。
 *
 * 未同步时 `time_sync_epoch_s()` 返回 0，上报层据此**不写** timestamp 字段，
 * 由 Agent 回退到本地接收时间。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/** @brief 初始化 SNTP（不阻塞、不同步，等联网后由 notify_connected 启动）。 */
esp_err_t time_sync_start(void);

/** @brief 拿到 IP 后调用；幂等，重复调用无副作用。 */
void time_sync_notify_connected(void);

/** @brief 是否已同步到可信的墙上时间。 */
bool time_sync_valid(void);

/** @brief 当前 epoch 秒；未同步返回 0。 */
int64_t time_sync_epoch_s(void);

/** @brief 开机秒数（单调，永不为负）。 */
uint32_t time_sync_uptime_s(void);
