#include "edge_detect.h"

#include <math.h>
#include <string.h>

#define EDGE_DEFAULT_CONFIRM 5
#define EDGE_DEFAULT_RELEASE 20
#define EDGE_COOLDOWN_US     (5LL * 1000 * 1000)
#define EDGE_REALERT_US      (30LL * 1000 * 1000)

static float window_rms(const edge_detector_t *d)
{
    if (d->win_count < EDGE_WINDOW_SIZE) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (int i = 0; i < EDGE_WINDOW_SIZE; i++) {
        sum += d->window[i] * d->window[i];
    }
    return sqrtf(sum / (float)EDGE_WINDOW_SIZE);
}

void edge_detector_init(edge_detector_t *d, float alpha,
                        float threshold_on, float threshold_off)
{
    if (d == NULL) {
        return;
    }
    memset(d, 0, sizeof(*d));
    d->alpha = alpha;
    d->threshold_on = threshold_on;
    d->threshold_off = (threshold_off < threshold_on) ? threshold_off : threshold_on * 0.6f;
    d->confirm_samples = EDGE_DEFAULT_CONFIRM;
    d->release_samples = EDGE_DEFAULT_RELEASE;
    d->cooldown_us = EDGE_COOLDOWN_US;
    d->realert_us = EDGE_REALERT_US;
    d->state = EDGE_STATE_NORMAL;
    /* 取一个足够早的时间，保证首帧告警不会被冷却期吞掉 */
    d->last_alert_us = INT64_MIN / 4;
}

void edge_detector_set_thresholds(edge_detector_t *d, float on, float off)
{
    if (d == NULL || on <= 0.0f) {
        return;
    }
    d->threshold_on = on;
    d->threshold_off = (off < on && off > 0.0f) ? off : on * 0.6f;
}

void edge_detector_reset(edge_detector_t *d)
{
    if (d == NULL) {
        return;
    }
    memset(d->window, 0, sizeof(d->window));
    d->win_idx = 0;
    d->win_count = 0;
    d->gx = d->gy = d->gz = 0.0f;
    d->primed = false;
    d->state = EDGE_STATE_NORMAL;
    d->over_count = 0;
    d->under_count = 0;
    d->rms = 0.0f;
    d->last_alert_us = INT64_MIN / 4;
}

bool edge_detector_feed(edge_detector_t *d, float ax, float ay, float az,
                        int64_t now_us, float *out_rms)
{
    if (d == NULL) {
        return false;
    }

    /* 1) 去重力：一阶 IIR 估计重力向量，首帧直接用当前值初始化以免长瞬态 */
    if (!d->primed) {
        d->gx = ax;
        d->gy = ay;
        d->gz = az;
        d->primed = true;
    } else {
        d->gx += d->alpha * (ax - d->gx);
        d->gy += d->alpha * (ay - d->gy);
        d->gz += d->alpha * (az - d->gz);
    }

    const float dx = ax - d->gx;
    const float dy = ay - d->gy;
    const float dz = az - d->gz;
    const float dyn_mag = sqrtf(dx * dx + dy * dy + dz * dz);

    /* 2) 滑窗累积 */
    d->window[d->win_idx] = dyn_mag;
    d->win_idx = (d->win_idx + 1) % EDGE_WINDOW_SIZE;
    if (d->win_count < EDGE_WINDOW_SIZE) {
        d->win_count++;
    }

    const float rms = window_rms(d);
    d->rms = rms;
    if (out_rms != NULL) {
        *out_rms = rms;
    }

    /* 3) 窗口未填满前不判定，避免冷启动误报 */
    if (d->win_count < EDGE_WINDOW_SIZE) {
        return false;
    }

    bool fire = false;

    switch (d->state) {
    case EDGE_STATE_NORMAL:
        if (rms > d->threshold_on) {
            d->state = EDGE_STATE_WARNING;
            d->over_count = 1;
            d->under_count = 0;
        }
        break;

    case EDGE_STATE_WARNING:
        if (rms > d->threshold_on) {
            d->over_count++;
            if (d->over_count >= d->confirm_samples) {
                d->state = EDGE_STATE_ALARM;
                d->under_count = 0;
                if (now_us - d->last_alert_us >= d->cooldown_us) {
                    d->last_alert_us = now_us;
                    fire = true;
                }
            }
        } else {
            /* 未确认即回落，视为瞬态干扰 */
            d->state = EDGE_STATE_NORMAL;
            d->over_count = 0;
        }
        break;

    case EDGE_STATE_ALARM:
        if (rms < d->threshold_off) {
            d->under_count++;
            if (d->under_count >= d->release_samples) {
                d->state = EDGE_STATE_NORMAL;
                d->over_count = 0;
                d->under_count = 0;
            }
        } else {
            d->under_count = 0;
        }

        /* 故障持续时按 realert 周期重发，避免单帧告警在网络抖动中丢失 */
        if (d->state == EDGE_STATE_ALARM &&
            now_us - d->last_alert_us >= d->realert_us) {
            d->last_alert_us = now_us;
            fire = true;
        }
        break;

    case EDGE_STATE_UNKNOWN:
        /* 传感器不可用：不做判定，等上层把状态改回来。
         * 这里必须显式处理，否则会掉到 default 被当成 NORMAL。 */
        return false;

    default:
        d->state = EDGE_STATE_NORMAL;
        break;
    }

    return fire;
}

const char *edge_state_name(edge_state_t s)
{
    switch (s) {
    case EDGE_STATE_NORMAL:
        return "NORMAL";
    case EDGE_STATE_WARNING:
        return "WARNING";
    case EDGE_STATE_ALARM:
        return "ALARM";
    case EDGE_STATE_UNKNOWN:
        return "UNKNOWN";
    default:
        return "UNKNOWN";
    }
}
