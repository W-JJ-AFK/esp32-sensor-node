/**
 * @file common.h
 * @brief 全工程共享的引脚映射、任务参数与数据快照类型。
 *
 * 这里只放与硬件无关的常量与 POD 结构，便于 algorithm 层在主机上单独测试。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "edge_detect.h"

/* ---------------- 引脚映射（与 README 接线表一致） ---------------- */
#define APP_I2C_SDA_GPIO      8
#define APP_I2C_SCL_GPIO      9
#define APP_DHT_GPIO          4
#define APP_OLED_I2C_ADDR     0x3C
#define APP_MPU_I2C_ADDR      0x68
#define APP_I2C_FREQ_HZ       400000

/* ---------------- 任务周期（毫秒） ---------------- */
#define APP_IMU_PERIOD_MS     10    /* 100 Hz 硬实时采样 */
#define APP_DETECT_PERIOD_MS  50    /* 每 50ms 批量消费 5 个样本，实际 100Hz 全量处理 */
#define APP_DHT_PERIOD_MS     2000  /* DHT22 最小采样间隔 */
#define APP_DISPLAY_PERIOD_MS 200
#define APP_MONITOR_PERIOD_MS 2000

/* ---------------- 任务优先级（数值越大优先级越高） ---------------- */
#define APP_PRIO_IMU      5
#define APP_PRIO_DETECT   4
#define APP_PRIO_NET      3
#define APP_PRIO_DHT      3
#define APP_PRIO_DISPLAY  2
#define APP_PRIO_MONITOR  1

/* ---------------- 任务栈（字节，ESP32 上 1 word = 4 字节） ---------------- */
#define APP_STACK_IMU      3072
#define APP_STACK_DETECT   3072
#define APP_STACK_NET      6144  /* JSON 组包 + TLS 预留 */
#define APP_STACK_DHT      4096  /* 内部使用 printf/ESP_LOG */
#define APP_STACK_DISPLAY  3072
#define APP_STACK_MONITOR  2048

/* ---------------- 设备身份（来自 Kconfig） ---------------- */
#define APP_TEMP_DEVICE_ID  CONFIG_APP_DEVICE_TEMP_ID
#define APP_VIB_DEVICE_ID   CONFIG_APP_DEVICE_VIB_ID
#define APP_TEMP_DEVICE_TYPE "temperature_sensor"
#define APP_VIB_DEVICE_TYPE  "vibration_sensor"

/* ---------------- 共享数据快照 ---------------- */

/** DHT22 最近一次成功读取的结果，由 task_dht 写、task_net/display 读。 */
typedef struct {
    float   temperature_c;
    float   humidity_pct;
    bool    valid;
    int64_t ts_ms;
} dht_snapshot_t;

/** 边缘检测器最近一次判定结果，由 task_detect 写、task_net/display 读。 */
typedef struct {
    float        rms;
    float        peak;
    edge_state_t state;
    int64_t      ts_ms;
} edge_snapshot_t;

/** 待上报的告警事件（定长，可直接进 FreeRTOS 队列）。 */
typedef struct {
    float        rms;
    float        threshold;
    edge_state_t state;
    int64_t      ts_ms;
} edge_alert_t;
