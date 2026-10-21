/**
 * @file wifi_mgr.h
 * @brief WiFi STA 连接管理（事件驱动 + 指数退避，永不放弃重连）。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief 初始化并异步发起连接。
 *
 * 返回 ESP_OK 只代表"已经开始连接"，不代表已联网。
 * 这样 app_main 可以先把采样/检测/显示任务拉起来，网络好坏不影响本地功能。
 */
esp_err_t wifi_mgr_start(void);

/** @brief 阻塞等待拿到 IP；仅调试或必须联网的路径使用。 */
bool wifi_mgr_wait_connected(uint32_t timeout_ms);

/** @brief 当前是否已连接并持有 IP。 */
bool wifi_mgr_is_connected(void);
