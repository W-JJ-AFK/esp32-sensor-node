/**
 * @file tasks.h
 * @brief 任务启动声明与跨任务共享状态。
 *
 * 共享状态约定：
 *   - g_dht / g_edge 由单一写者更新、多读者读取，访问必须持 g_state_mutex
 *   - g_imu_ring 是单生产者单消费者无锁环（task_imu -> task_detect）
 *   - g_alert_queue 是 task_detect -> task_net 的告警通道
 */

#pragma once

#include "algo/edge_detect.h"
#include "algo/imu_ring.h"
#include "common.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

typedef enum {
    APP_TASK_IMU = 0,
    APP_TASK_DETECT,
    APP_TASK_DHT,
    APP_TASK_NET,
    APP_TASK_DISPLAY,
    APP_TASK_MONITOR,
    APP_TASK_COUNT,
} app_task_id_t;

typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    /** IMU 初始化是否成功。失败时不启动采样任务，并把状态置为 EDGE_STATE_UNKNOWN，
     *  避免上报 rms=0/state=NORMAL 这种"看起来最健康"的假数据。 */
    bool imu_ok;
} app_context_t;

extern app_context_t    g_app;
extern SemaphoreHandle_t g_state_mutex;
extern dht_snapshot_t   g_dht;
extern edge_snapshot_t  g_edge;
extern imu_ring_t       g_imu_ring;
extern QueueHandle_t    g_alert_queue;
extern edge_detector_t  g_detector;
extern volatile uint32_t g_telemetry_period_ms;
extern TaskHandle_t     g_task_handles[APP_TASK_COUNT];

void task_imu_start(void);
void task_detect_start(void);
void task_dht_start(void);
void task_net_start(void);
void task_display_start(void);
void task_monitor_start(void);

/** @brief 断网期间缓存的待补发告警条数 / 因缓存满被淘汰的条数。 */
uint32_t task_net_backlog_depth(void);
uint32_t task_net_backlog_dropped(void);
