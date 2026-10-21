/**
 * @file edge_detect.h
 * @brief 边缘异常检测：去重力 + 滑窗 RMS + 迟滞三态状态机。
 *
 * 纯 C 实现，不依赖 ESP-IDF，可在主机上用 gcc 直接编译测试（见 test/）。
 *
 * 处理链：
 *   原始加速度(100Hz) -> IIR 去重力 -> 20 点滑窗 RMS -> NORMAL/WARNING/ALARM
 *
 * 状态机语义：
 *   NORMAL  --rms > on------------> WARNING（首次越过阈值，尚未确认）
 *   WARNING --连续 confirm 次超限--> ALARM（触发告警）
 *   WARNING --rms <= on----------> NORMAL（未确认就回落，不告警）
 *   ALARM   --连续 release 次低于 off--> NORMAL（迟滞退出）
 *
 * 为什么 WARNING 单独存在：把"越过阈值"和"确认故障"分开，告警只在确认后发出，
 * 既保留了对瞬时冲击的敏感性，又不会因为单点毛刺打扰运维。
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define EDGE_WINDOW_SIZE 20

typedef enum {
    EDGE_STATE_NORMAL = 0,
    EDGE_STATE_WARNING = 1,
    EDGE_STATE_ALARM = 2,
    /* 传感器不可用（初始化失败、连续读失败）。
     * 必须有这个显式状态：曾经的做法是让 g_edge 保持初值，
     * 于是 IMU 焊虚了也会每 2s 上报 rms=0 / state=NORMAL，
     * 在仪表盘上看起来是"全车间最健康的设备"。 */
    EDGE_STATE_UNKNOWN = 3,
} edge_state_t;

typedef struct {
    /* ---- 判定参数 ---- */
    float   threshold_on;    /* 进入 WARNING 的阈值 (g) */
    float   threshold_off;   /* 退出 ALARM 的阈值 (g)，必须小于 on */
    int     confirm_samples; /* 连续超限多少次确认告警 */
    int     release_samples; /* 连续低于 off 多少次解除告警 */
    int64_t cooldown_us;     /* 两次告警之间的最小间隔（抑制告警风暴） */
    int64_t realert_us;      /* ALARM 持续期间的重发周期（防单次丢失） */

    /* ---- 去重力滤波器 ---- */
    float alpha;             /* IIR 系数，越小重力估计越稳、跟随倾斜越慢 */
    float gx, gy, gz;        /* 重力估计向量 */
    bool  primed;            /* 是否已用首帧初始化重力 */

    /* ---- 滑窗 ---- */
    float window[EDGE_WINDOW_SIZE];
    int   win_idx;
    int   win_count;

    /* ---- 状态机 ---- */
    edge_state_t state;
    int          over_count;
    int          under_count;
    float        rms;
    int64_t      last_alert_us;
} edge_detector_t;

/**
 * @brief 初始化检测器。
 *
 * @param alpha         去重力 IIR 系数，建议 dt/(tau+dt)，dt=10ms、tau=500ms -> 0.02
 * @param threshold_on  告警进入阈值 (g)
 * @param threshold_off 告警退出阈值 (g)，必须 < threshold_on
 */
void edge_detector_init(edge_detector_t *d, float alpha,
                        float threshold_on, float threshold_off);

/** @brief 运行期调整阈值（例如来自远程指令），保留当前状态与窗口。 */
void edge_detector_set_thresholds(edge_detector_t *d, float on, float off);

/** @brief 清空窗口与状态机，保留配置参数。 */
void edge_detector_reset(edge_detector_t *d);

/**
 * @brief 喂入一个加速度样本。
 *
 * @param now_us  单调时间戳（微秒）
 * @param out_rms 输出当前 RMS（可为 NULL）；窗口未填满时返回 0
 * @return true 表示本次应当发出一帧告警（进入 ALARM 或 ALARM 期间重发）
 */
bool edge_detector_feed(edge_detector_t *d, float ax, float ay, float az,
                        int64_t now_us, float *out_rms);

/** @brief 状态名，用于日志与 MQTT 上报（大写，配合 5x7 字模）。 */
const char *edge_state_name(edge_state_t s);
