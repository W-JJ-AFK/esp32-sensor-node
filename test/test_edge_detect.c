/**
 * @file test_edge_detect.c
 * @brief 边缘检测算法与环形缓冲的主机端单元测试。
 *
 * 纯 C、无 ESP-IDF 依赖，可直接在 PC 上编译运行：
 *   cd test && make
 *
 * 覆盖：静态不误报 / 逐级确认 / 阈值严格比较 / 迟滞退出 / 冷却抑制 /
 *       RMS 数值正确性 / 传感器失效态 / 环形缓冲 FIFO 与溢出 /
 *       MQTT 主题契约（与 agent/protocol.py 交叉核对）。
 */

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "edge_detect.h"
#include "imu_ring.h"
#include "topic.h"

#define SAMPLE_PERIOD_US (10 * 1000) /* 100 Hz */
#define ALPHA            0.02f
#define TH_ON            0.55f
#define TH_OFF           0.35f

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, fmt, ...)                                            \
    do {                                                                 \
        g_checks++;                                                      \
        if (!(cond)) {                                                   \
            g_failures++;                                                \
            printf("  [FAIL] %s:%d  " fmt "\n", __FILE__, __LINE__,      \
                   ##__VA_ARGS__);                                       \
        }                                                                \
    } while (0)

/* 生成与索引绑定的确定性振动样本：x 轴正弦摆动，z 轴保持重力 */
static void make_sample(int i, float amplitude, float *ax, float *ay, float *az)
{
    *ax = amplitude * sinf((float)i * 0.7f);
    *ay = 0.0f;
    *az = 1.0f;
}

static void test_static_no_alarm(void)
{
    printf("test_static_no_alarm\n");
    edge_detector_t d;
    edge_detector_init(&d, ALPHA, TH_ON, TH_OFF);

    bool fired = false;
    for (int i = 0; i < 300; i++) {
        if (edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, (int64_t)i * SAMPLE_PERIOD_US, NULL)) {
            fired = true;
        }
    }
    CHECK(!fired, "static device must not raise alerts");
    CHECK(d.state == EDGE_STATE_NORMAL, "state should stay NORMAL, got %s",
          edge_state_name(d.state));
    CHECK(d.rms < 0.01f, "static rms should be ~0, got %.4f", d.rms);
}

static void test_gradual_confirm(void)
{
    printf("test_gradual_confirm\n");
    edge_detector_t d;
    edge_detector_init(&d, ALPHA, TH_ON, TH_OFF);

    /* 先建立重力基准，再叠加振动 */
    for (int i = 0; i < 100; i++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, (int64_t)i * SAMPLE_PERIOD_US, NULL);
    }
    CHECK(d.state == EDGE_STATE_NORMAL, "baseline should be NORMAL");

    int fired = 0;
    for (int i = 100; i < 400; i++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        if (edge_detector_feed(&d, ax, ay, az, (int64_t)i * SAMPLE_PERIOD_US, NULL)) {
            fired++;
        }
    }
    CHECK(fired >= 1, "sustained vibration must raise at least one alert");
    CHECK(d.state == EDGE_STATE_ALARM, "state should be ALARM, got %s",
          edge_state_name(d.state));
    /* 冷却窗口 5s = 500 个样本，这里只跑了 300 个样本，因此最多 1 次 */
    CHECK(fired == 1, "cooldown should suppress repeats, fired=%d", fired);
}

static void test_confirm_count(void)
{
    printf("test_confirm_count\n");
    edge_detector_t d;
    /* 把阈值压到极低：任何振动都会立刻越阈，从而单独考察「确认计数」这一个维度。
     * 如果沿用 0.55g 阈值，RMS 需要十几个样本才爬过阈值，就分不清"没告警"是因为
     * 确认次数没到，还是因为 RMS 还没涨上来。 */
    edge_detector_init(&d, ALPHA, 0.01f, 0.005f);

    for (int i = 0; i < 40; i++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, (int64_t)i * SAMPLE_PERIOD_US, NULL);
    }
    CHECK(d.state == EDGE_STATE_NORMAL, "baseline must be NORMAL");

    /* 前 confirm_samples-1 个超限样本：只进 WARNING，不允许告警 */
    bool fired_early = false;
    for (int i = 1; i < d.confirm_samples; i++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        if (edge_detector_feed(&d, ax, ay, az, (int64_t)(40 + i) * SAMPLE_PERIOD_US, NULL)) {
            fired_early = true;
        }
    }
    CHECK(!fired_early, "must not fire before %d confirm samples", d.confirm_samples);
    CHECK(d.state == EDGE_STATE_WARNING, "should stay WARNING before confirm, got %s",
          edge_state_name(d.state));

    /* 第 confirm_samples 个超限样本：确认故障并告警 */
    float ax, ay, az;
    make_sample(d.confirm_samples, 2.0f, &ax, &ay, &az);
    const bool fired = edge_detector_feed(&d, ax, ay, az,
                                          (int64_t)(40 + d.confirm_samples) * SAMPLE_PERIOD_US, NULL);
    CHECK(fired, "should fire on the %d-th over-threshold sample", d.confirm_samples);
    CHECK(d.state == EDGE_STATE_ALARM, "should be ALARM, got %s", edge_state_name(d.state));
}

static void test_threshold_is_strict(void)
{
    printf("test_threshold_is_strict\n");
    edge_detector_t probe;
    edge_detector_init(&probe, ALPHA, TH_ON, TH_OFF);

    float rms = 0.0f;
    for (int i = 0; i < 120; i++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        edge_detector_feed(&probe, ax, ay, az, (int64_t)i * SAMPLE_PERIOD_US, &rms);
    }

    /* 用实测 RMS 作为阈值：由于计算路径完全一致，比较应为 rms > rms == false */
    edge_detector_t exact;
    edge_detector_init(&exact, ALPHA, rms, rms * 0.6f);
    bool fired = false;
    for (int i = 0; i < 120; i++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        if (edge_detector_feed(&exact, ax, ay, az, (int64_t)i * SAMPLE_PERIOD_US, NULL)) {
            fired = true;
        }
    }
    CHECK(!fired, "rms == threshold must not trigger (strict greater-than)");
    CHECK(exact.state == EDGE_STATE_NORMAL, "state should be NORMAL at threshold, got %s",
          edge_state_name(exact.state));
}

static void test_hysteresis(void)
{
    printf("test_hysteresis\n");
    edge_detector_t d;
    edge_detector_init(&d, ALPHA, TH_ON, TH_OFF);

    for (int i = 0; i < 60; i++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, (int64_t)i * SAMPLE_PERIOD_US, NULL);
    }

    int64_t t = 60;
    for (int i = 0; i < 200; i++, t++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        edge_detector_feed(&d, ax, ay, az, t * SAMPLE_PERIOD_US, NULL);
    }
    CHECK(d.state == EDGE_STATE_ALARM, "should be in ALARM before hysteresis test");

    /* 中等幅度：rms 处于 off~on 之间，应维持 ALARM 而不是抖动 */
    bool stayed_alarm = true;
    for (int i = 0; i < 200; i++, t++) {
        float ax, ay, az;
        make_sample(i, 0.7f, &ax, &ay, &az);
        edge_detector_feed(&d, ax, ay, az, t * SAMPLE_PERIOD_US, NULL);
        if (d.state != EDGE_STATE_ALARM) {
            stayed_alarm = false;
            break;
        }
    }
    CHECK(stayed_alarm, "must stay ALARM inside hysteresis band (got %s)",
          edge_state_name(d.state));

    /* 降到 off 以下并持续 release_samples 次才允许退出 */
    for (int i = 0; i < 40; i++, t++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, t * SAMPLE_PERIOD_US, NULL);
    }
    CHECK(d.state == EDGE_STATE_NORMAL, "should return to NORMAL after release window");
}

static void test_cooldown(void)
{
    printf("test_cooldown\n");
    edge_detector_t d;
    edge_detector_init(&d, ALPHA, TH_ON, TH_OFF);

    int64_t t = 0;
    for (int i = 0; i < 40; i++, t++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, t * SAMPLE_PERIOD_US, NULL);
    }

    int first = 0;
    for (int i = 0; i < 120; i++, t++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        if (edge_detector_feed(&d, ax, ay, az, t * SAMPLE_PERIOD_US, NULL)) {
            first++;
        }
    }
    CHECK(first == 1, "first alarm should fire once, got %d", first);

    /* 回落复位 */
    for (int i = 0; i < 40; i++, t++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 1.0f, t * SAMPLE_PERIOD_US, NULL);
    }

    /* 冷却窗口内再次超限：不应告警 */
    int second = 0;
    for (int i = 0; i < 120; i++, t++) {
        float ax, ay, az;
        make_sample(i, 2.0f, &ax, &ay, &az);
        if (edge_detector_feed(&d, ax, ay, az, t * SAMPLE_PERIOD_US, NULL)) {
            second++;
        }
    }
    CHECK(second == 0, "alarm inside cooldown must be suppressed, got %d", second);
    CHECK(d.state == EDGE_STATE_ALARM, "state should still escalate to ALARM");
}

static void test_rms_value(void)
{
    printf("test_rms_value\n");
    edge_detector_t d;
    edge_detector_init(&d, ALPHA, TH_ON, TH_OFF);

    /* 让重力收敛到 0（零加速度），之后 dyn 幅度即为注入值 */
    for (int i = 0; i < 500; i++) {
        edge_detector_feed(&d, 0.0f, 0.0f, 0.0f, (int64_t)i * SAMPLE_PERIOD_US, NULL);
    }

    /* 恒定幅值 0.4g 的方波式振动：|dyn| 约等于 0.4，RMS 也应约等于 0.4 */
    float rms = 0.0f;
    for (int i = 0; i < 40; i++) {
        float v = (i % 2 == 0) ? 0.4f : -0.4f;
        edge_detector_feed(&d, v, 0.0f, 0.0f, (int64_t)(500 + i) * SAMPLE_PERIOD_US, &rms);
    }
    CHECK(fabsf(rms - 0.4f) < 0.05f, "rms should be ~0.4, got %.4f", rms);
}

static void test_sensor_unknown_state(void)
{
    printf("test_sensor_unknown_state\n");

    /* 传感器初始化失败时固件把状态置为 UNKNOWN。这个状态必须：
     *   1) 永不触发告警；
     *   2) 不被状态机的 default 分支悄悄改回 NORMAL。
     * 反例就是修复前的行为：IMU 焊虚了仍每 2s 上报 rms=0 / state=NORMAL，
     * 在仪表盘上看起来是"全车间最健康的设备"。 */
    edge_detector_t d;
    edge_detector_init(&d, ALPHA, TH_ON, TH_OFF);
    d.state = EDGE_STATE_UNKNOWN;

    int fired = 0;
    for (int i = 0; i < 200; i++) {
        float ax, ay, az;
        make_sample(i, 3.0f, &ax, &ay, &az); /* 幅值远超阈值 */
        float rms = 0.0f;
        if (edge_detector_feed(&d, ax, ay, az, (int64_t)i * SAMPLE_PERIOD_US, &rms)) {
            fired++;
        }
    }
    CHECK(fired == 0, "UNKNOWN state must never fire, got %d events", fired);
    CHECK(d.state == EDGE_STATE_UNKNOWN, "UNKNOWN must not decay to NORMAL");

    CHECK(strcmp(edge_state_name(EDGE_STATE_UNKNOWN), "UNKNOWN") == 0,
          "UNKNOWN name mismatch: %s", edge_state_name(EDGE_STATE_UNKNOWN));
    CHECK(strcmp(edge_state_name(EDGE_STATE_ALARM), "ALARM") == 0,
          "ALARM name mismatch: %s", edge_state_name(EDGE_STATE_ALARM));
}

static void test_ring_buffer(void)
{
    printf("test_ring_buffer\n");
    imu_ring_t ring;
    imu_ring_init(&ring);

    int pushed = 0;
    for (int i = 0; i < IMU_RING_CAPACITY + 10; i++) {
        imu_sample_t s = {.ax = (float)i, .ay = 0.0f, .az = 0.0f, .ts_us = i};
        if (!imu_ring_push(&ring, &s)) {
            break;
        }
        pushed++;
    }
    CHECK(pushed == IMU_RING_CAPACITY - 1, "ring holds capacity-1 samples, got %d", pushed);
    CHECK(imu_ring_dropped(&ring) > 0, "overflow must be counted");
    CHECK(imu_ring_count(&ring) == (uint32_t)pushed, "count mismatch");

    imu_sample_t out;
    int popped = 0;
    float expect = 0.0f;
    while (imu_ring_pop(&ring, &out)) {
        CHECK(out.ax == expect, "FIFO order broken at %d", popped);
        expect += 1.0f;
        popped++;
    }
    CHECK(popped == pushed, "popped %d != pushed %d", popped, pushed);
    CHECK(!imu_ring_pop(&ring, &out), "pop on empty ring must fail");
}


/* ---------------- MQTT 主题契约 ----------------
 *
 * 主题是跨仓库合同里最脆弱的一环：写错一个字符，订阅和发布都会"成功"，
 * 只是对端永远收不到，真机上没有任何报错。这里把五个主题的字面值
 * 硬编码进断言，并与 agent/protocol.py 的同一张表逐字对齐：
 *
 *   tests/test_protocol.py::test_topic_builders
 *   tests/test_protocol.py::test_parse_topic
 *
 * 任何一端的字符串被改动，两端测试必然有一个变红。
 */
static void test_topic_contract(void)
{
    printf("test_topic_contract\n");

    const struct {
        topic_kind_t kind;
        const char  *expect;
    } cases[] = {
        {TOPIC_TELEMETRY, "devices/vibration_01/telemetry"},
        {TOPIC_STATUS,    "devices/vibration_01/status"},
        {TOPIC_ALERT,     "devices/vibration_01/alert"},
        {TOPIC_CMD,       "devices/vibration_01/cmd"},
        {TOPIC_ACK,       "devices/vibration_01/cmd/ack"},
    };

    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        char buf[TOPIC_BUF_LEN];
        const int n = topic_build(buf, sizeof(buf), "vibration_01", cases[i].kind);
        CHECK(n == (int)strlen(cases[i].expect), "kind %d: length %d != %d",
              (int)cases[i].kind, n, (int)strlen(cases[i].expect));
        CHECK(strcmp(buf, cases[i].expect) == 0, "kind %d: got '%s', want '%s'",
              (int)cases[i].kind, buf, cases[i].expect);

        /* 造出来的主题，必须能被自己解析回同一个 device_id */
        char id[32];
        CHECK(topic_parse_device_id(buf, id, sizeof(id)), "kind %d: self-parse failed",
              (int)cases[i].kind);
        CHECK(strcmp(id, "vibration_01") == 0, "kind %d: parsed id '%s'",
              (int)cases[i].kind, id);
    }

    /* 缓冲不足必须失败，而不是产生被截断的主题 */
    char small[20];
    CHECK(topic_build(small, sizeof(small), "vibration_01", TOPIC_TELEMETRY) < 0,
          "undersized buffer must fail");

    /* 空 ID / 含斜杠的 ID 直接拒绝 */
    char buf[TOPIC_BUF_LEN];
    CHECK(topic_build(buf, sizeof(buf), "", TOPIC_CMD) < 0, "empty id must fail");
    CHECK(topic_build(buf, sizeof(buf), "a/b", TOPIC_CMD) < 0, "id with '/' must fail");

    /* TOPIC_BUF_LEN 必须容得下最长的真实组合：
     * 之前 mqtt_mgr.c 写 96、task_net.c 也写 96，两个魔数各写一遍，
     * 放宽 device_id 时很容易只改一处。这里用真实前缀 + 最长后缀钉住它。 */
    CHECK(topic_build(buf, sizeof(buf), "vibration_01", TOPIC_ACK) > 0,
          "TOPIC_BUF_LEN too small for longest real topic");

    /* cmd/ack 不能被当成 cmd：这正是"回执被误执行"的入口 */
    CHECK(strcmp(topic_kind_suffix(TOPIC_ACK), "cmd/ack") == 0, "ack suffix wrong");
    CHECK(topic_match("devices/a/cmd", "a", TOPIC_CMD), "plain cmd should match");
    CHECK(!topic_match("devices/a/cmd/ack", "a", TOPIC_CMD),
          "ack topic must not match CMD");
    CHECK(topic_match("devices/a/cmd/ack", "a", TOPIC_ACK), "ack topic should match ACK");
    CHECK(!topic_match("devices/ab/cmd", "a", TOPIC_CMD), "prefix of id must not match");
    CHECK(!topic_match("devices/a/telemetry", "a", TOPIC_CMD), "wrong kind must not match");

    /* 不认识的主题形态一律解析失败 */
    char id[32];
    CHECK(!topic_parse_device_id("devices//cmd", id, sizeof(id)), "empty id rejected");
    CHECK(!topic_parse_device_id("devices/a/unknown", id, sizeof(id)), "unknown kind rejected");
    CHECK(!topic_parse_device_id("other/a/cmd", id, sizeof(id)), "wrong prefix rejected");
    CHECK(!topic_parse_device_id("devices/a", id, sizeof(id)), "missing suffix rejected");
    CHECK(!topic_parse_device_id(NULL, id, sizeof(id)), "NULL topic rejected");

    /* 超长 device_id 必须失败，不能被截断后当成另一台设备 */
    char tiny[4];
    CHECK(!topic_parse_device_id("devices/vibration_01/cmd", tiny, sizeof(tiny)),
          "truncated id must be rejected");
}

int main(void)
{
    printf("=== edge_detect / imu_ring / topic host tests ===\n");
    test_static_no_alarm();
    test_gradual_confirm();
    test_confirm_count();
    test_threshold_is_strict();
    test_hysteresis();
    test_cooldown();
    test_rms_value();
    test_sensor_unknown_state();
    test_ring_buffer();
    test_topic_contract();

    printf("\n%d checks, %d failure(s)\n", g_checks, g_failures);
    printf("%s\n", g_failures == 0 ? "PASS" : "FAIL");
    return g_failures == 0 ? 0 : 1;
}
