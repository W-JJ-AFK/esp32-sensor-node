# ESP32-S3 边缘振动监测节点

[![build](https://github.com/W-JJ-AFK/esp32-sensor-node/actions/workflows/build.yml/badge.svg)](https://github.com/W-JJ-AFK/esp32-sensor-node/actions/workflows/build.yml)
![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.3.2-000000)
![target](https://img.shields.io/badge/target-ESP32--S3-E7352C)
![lang](https://img.shields.io/badge/C-11-00599C)

一块自己就能干活的振动监测节点：**100 Hz 采样 → 本地判定 → 200 ms 级告警 → 断网不丢、联网可调参**。

网络挂了本地检测照常跑；网络回来自动重连、重新上报在线状态、补发断网期间的告警。
上位机（[edge-agent-system](https://github.com/W-JJ-AFK/edge-agent-system)）只负责解释和下发指令，**判定永远在设备侧**。

| 指标 | 实测值 |
|---|---|
| 固件体积 | `iot_edge_node.bin` = 990,272 B ≈ 967 KB（app 分区 4 MB，余量 76%） |
| 编译 | ESP-IDF **v5.3.2** + `xtensa-esp-elf`，**0 error / 0 warning** |
| 主机侧测试 | **129 条断言 / 10 个测试函数，全绿**（不需要硬件，秒级） |
| 代码规模 | `main/` 共 30 个 `.c/.h`，3,009 行 |
| 检测延迟 | 信号 160 ms + 调度 ≤50 ms，**约 200 ms 量级**（推导见 [6.5](#65-检测延迟到底是多少)） |

> 这个仓库只有一条规矩：**README 里每一句"它支持 X"，都必须能指到一行代码，或者一条你自己能跑出来的命令。**
> 做不到的，全部写在 [11. 明确的边界](#11-明确的边界)里。

---

## 目录

- [1. 它解决什么问题](#1-它解决什么问题)
- [2. 能力清单](#2-能力清单能力--代码--怎么自己验)
- [3. 硬件与接线](#3-硬件与接线)
- [4. 快速开始](#4-快速开始)
- [5. 架构与任务划分](#5-架构与任务划分)
- [6. 检测算法](#6-检测算法)
- [7. 通信契约](#7-通信契约)
- [8. 远程指令](#8-远程指令)
- [9. 验证：四个层次](#9-验证四个层次)
- [10. 上板检查清单](#10-上板检查清单)
- [11. 明确的边界](#11-明确的边界)
- [12. 目录结构](#12-目录结构)
- [13. 关键设计取舍](#13-关键设计取舍)
- [14. 面试会追问什么](#14-面试会追问什么)

---

## 1. 它解决什么问题

工业现场的振动监测通常有三种做法，代价各不相同：

| 做法 | 问题 |
|---|---|
| 只采原始数据，全部传云端判定 | 带宽和成本不划算；断网就完全失效，而现场网络恰恰不可靠 |
| 采完本地存 Flash，定期批量回传 | 告警延迟以小时计，故障已经发生了才知道 |
| **本地判定 + 只上报事件与统计量** | 需要把算法、时序、可靠性都做扎实 —— 这就是本仓库 |

所以这个节点的设计目标是三句话：

1. **判定必须在本地闭环。** 断网期间检测不停、告警不丢（RAM 环形积压 64 条）。
2. **上报的必须是"结论 + 可解释的量"**，不是原始波形。运维要看到 `rms=0.72 g / threshold=0.55 g`，而不是 2000 个浮点数。
3. **每个参数都要说得出物理理由。** 为什么阈值 0.55 g、为什么窗口 20 点、为什么去重力时间常数 500 ms —— 见 [6](#6-检测算法)。

---

## 2. 能力清单（能力 → 代码 → 怎么自己验）

| 能力 | 实现位置 | 怎么自己验 |
|---|---|---|
| 100 Hz 加速度采样 | `main/tasks/task_imu.c`（10 ms 周期，优先级 5） | 真机看 `TASK_MONITOR` 里 `imu_ring used` 的增长速率 |
| IIR 去重力 | `main/algo/edge_detect.c` → `edge_detector_feed()` | `test_static_no_alarm`：静止 300 点，断言无告警且 `rms < 0.01 g` |
| 20 点滑窗 RMS | 同上，`EDGE_WINDOW_SIZE 20` | `test_rms_value`：注入 ±0.4 g 方波，断言 RMS = 0.4 ± 0.05 |
| 迟滞三态状态机 | 同上，`EDGE_STATE_NORMAL/WARNING/ALARM` | `test_hysteresis`：在进/出阈值之间来回抖动，断言保持 ALARM 不抖动 |
| 第四态 `UNKNOWN`（传感器不可信） | `main/algo/edge_detect.h`，`feed()` 直接短路不产告警 | `test_sensor_unknown_state`：IMU 初始化失败后灌 200 个超阈样本，断言 0 次告警 |
| 连续 5 次确认才告警 | `edge_detector_t.confirm_samples`（默认 5） | `test_confirm_count`：前 4 个样本只进 WARNING，第 5 个才 ALARM |
| 严格大于才越阈 | 同上 | `test_threshold_is_strict`：把阈值设成实测 RMS 本身，断言**不**触发 |
| 告警冷却 + 重发 | `cooldown_us` / `realert_us` | `test_cooldown`：冷却窗口内二次超限被吞掉，但状态仍升到 ALARM |
| 无锁环形缓冲 | `main/algo/imu_ring.c`（`__atomic_*` release/acquire） | `test_ring_buffer`：容量、溢出计数、FIFO 顺序、空读失败 |
| 低干扰 DHT22 读取 | `main/drivers/dht22.c`（临界区只包住**单 bit**，约 130 µs；3 次重试 + 退避） | 连续读 100 次无 `ESP_ERR_TIMEOUT`，期间采样任务不掉点 |
| 传感器失效显式化 | `main/tasks/task_dht.c`（连续 3 次失败置 `valid=false`） | 拔掉数据线，OLED 显示 `T --.-C H --%`，遥测带 `data.valid=false` |
| OLED 显示 | `main/drivers/ssd1306.c` + `main/tasks/task_display.c` | 四行：温度 / 湿度 / RMS / 状态；失效显示 `DHT22 FAIL`、`RMS --- G` |
| WiFi 断线重连（永不休止） | `main/net/wifi_mgr.c`，指数退避 0.5 → 60 s 封顶 | 拔路由器电源再上电，看日志退避逐渐拉长并最终恢复 |
| MQTT 遗嘱消息（LWT） | `main/net/mqtt_mgr.c`，**每个 device_id 一条独立连接** | `mosquitto_sub -t 'devices/+/status'` 后直接拔电，两个设备各自收到 `offline` |
| 主题契约单一入口 | `main/net/topic.[ch]`，`cmd/ack` 不会被误判成 `cmd` | `test_topic_contract`：五个主题逐字断言 |
| 告警优先上报 | `main/tasks/task_net.c`（队列阻塞 200 ms，告警不走周期） | 制造一次振动，对比告警与遥测的时间戳先后 |
| 断网告警积压 | `main/tasks/task_net.c`，RAM 环形 64 条，重连后每轮补发 4 条 | 断网制造 10 次告警 → 恢复网络，`alert_backlog` 逐轮归零 |
| 远程调上报周期 | `main/app_main.c` → `set_interval` | 下发后遥测间隔变化，回执 `result=ok` |
| 远程调振动阈值 | `main/app_main.c` → `set_threshold` | 回执 `detail` 返回 `threshold=0.80g (release 0.48g)` |
| 远程重启（先回执后重启） | `main/app_main.c`，单次 `esp_timer` 延迟 500 ms | 先收到 ack，再看到重启日志 |
| 运行时监控 | `main/tasks/task_monitor.c`：堆 + 每任务栈高水位 + 队列深度 + 积压深度 | 串口每 2 s 打印一次 |

---

## 3. 硬件与接线

| 器件 | 接口 | 引脚 | 备注 |
|---|---|---|---|
| ESP32-S3 N16R8 | — | — | 16 MB Flash + 8 MB Octal PSRAM |
| SSD1306 OLED 128×64 | I2C | SDA=GPIO8, SCL=GPIO9 | 地址 `0x3C`，400 kHz |
| MPU6050 | I2C | 同上（共用总线） | 地址 `0x68` |
| DHT22 / AM2302 | 单总线 | GPIO4 | **必须外接 4.7–10 kΩ 上拉到 3.3 V** |

引脚定义集中在 `main/common.h:16-21`，改接线只改这一处。

**两个容易踩的硬件细节：**

- **DHT22 的上拉不能省。** 模块内部虽有上拉，但 3.3 V + 长导线时上升沿偏慢，会整帧 CRC 错。代码里也开了内部上拉，但替代不了外部电阻。
- **单总线必须用开漏，且全程不切方向。** 驱动用 `GPIO_MODE_INPUT_OUTPUT_OD`：写 0 = 拉低，写 1 = 释放（高阻）。
  如果改成推挽输出，ESP32 会在释放后主动把线拉到 3.3 V，与传感器随后的拉低响应"对着干"，
  接触稍差就整帧校验失败。见 `main/drivers/dht22.c`。

---

## 4. 快速开始

```bash
# 0) 环境：ESP-IDF v5.3.2（需 ≥ v5.2，新 i2c_master 驱动从 5.2 起提供）
git clone https://github.com/W-JJ-AFK/esp32-sensor-node && cd esp32-sensor-node

# 1) 先跑算法层测试（不需要开发板，秒级）
make -C test
#   129 checks, 0 failure(s)
#   PASS

# 2) 改成你自己的 WiFi / broker / 设备 ID
idf.py menuconfig
#   Edge Node Configuration
#     WiFi SSID / password      ← 必须 2.4 GHz；S3 不支持 5 GHz
#     MQTT broker URI           ← 跑 Agent 那台电脑的局域网 IP，不是 localhost
#     SNTP server               ← 默认 ntp.aliyun.com；内网不通外网就留空
#     device id                 ← 必须与 Agent 的 config.yaml 完全一致

# 3) 编译烧写
idf.py set-target esp32s3
idf.py build                 # 实测：967 KB，0 error / 0 warning
idf.py -p /dev/ttyUSB0 flash monitor      # Windows: -p COM3
```

机器上没有 gcc（比如只装了 ESP-IDF 工具链）时，主机测试可以借用 zig 的 C 前端：

```bash
pip install ziglang && make -C test CC="python -m ziglang cc"
```

启动日志应该长这样：

```
I (312) APP: === IoT edge node starting ===
I (315) APP: temp device: temp_sensor_01, vib device: vibration_01
I (421) MPU6050: initialized (WHO_AM_I=0x68, DLPF=44Hz, 200Hz ODR, ±2g)
I (430) APP: === 6 tasks launched, heap free=214336 ===
I (432) WIFI: connecting to "MyHome-2.4G" (async, edge detection runs regardless)
I (498) TIME_SYNC: sntp ready, server=ntp.aliyun.com
I (1520) WIFI: got ip: 192.168.1.42
I (1600) MQTT: connected to broker
I (3601) TASK_MONITOR: imu_ring used=0 dropped=0 alert_q=0 alert_backlog=0(lost 0)
```

注意顺序：**任务先起、网络后连**。采样 / 检测 / 显示从第 430 ms 就开始跑，
WiFi 连不上也不影响本地判定 —— 这正是这个节点的卖点，日志顺序要和它一致。
`timestamp` 字段要等 SNTP 同步完成后才会出现在报文里（见 [7.2](#72-时间基准timestamp-可能缺席uptime_s-永远在)）。

---

## 5. 架构与任务划分

```
        ┌──────────────┐  10ms   ┌──────────────┐
        │  task_imu    │────────▶│  imu_ring    │  无锁 SPSC 环形缓冲（64 深度）
        │  prio 5      │         │  (lock-free) │  满时丢新样本，绝不阻塞采样
        └──────────────┘         └──────┬───────┘
                                        │ 50ms 批量排空
                                 ┌──────▼───────┐
                                 │ task_detect  │  IIR 去重力 → 滑窗 RMS → 三态机
                                 │  prio 4      │
                                 └──┬────────┬──┘
                    写 g_edge       │        │ 告警事件
                 (g_state_mutex)    │        ▼
                                 ┌──▼─────┐  ┌────────────┐
        ┌──────────────┐         │ g_edge │  │ g_alert_   │ 队列深度 8
        │  task_dht    │────────▶│ 快照   │  │ queue      │
        │  prio 3      │         └──┬─────┘  └─────┬──────┘
        └──────────────┘            │              │
                                    │      ┌───────▼──────┐
                                    └─────▶│  task_net    │ 先补发积压 → 新告警立刻发 / 遥测按周期
                                           │  prio 3      │ 发不出去就进 RAM 环形积压（64 条）
                                           └──────────────┘
        ┌──────────────┐   ┌──────────────┐
        │ task_display │   │ task_monitor │  堆 / 栈水位 / 队列深度
        │  prio 2      │   │  prio 1      │
        └──────────────┘   └──────────────┘
```

| 任务 | 周期 | 优先级 | 栈 (B) | 职责 | 文件 |
|---|---|---|---|---|---|
| `task_imu` | 10 ms | 5 | 3072 | I2C 读加速度 → 入环形缓冲 | `main/tasks/task_imu.c` |
| `task_detect` | 50 ms | 4 | 3072 | 排空缓冲 → 逐样本判定 → 生产告警 | `main/tasks/task_detect.c` |
| `task_net` | 事件 + 周期 | 3 | 6144 | 积压补发 → 告警即时上报 → 遥测周期上报 | `main/tasks/task_net.c` |
| `task_dht` | 2000 ms | 3 | 4096 | 温湿度采集（3 次重试 + 连续失败置无效） | `main/tasks/task_dht.c` |
| `task_display` | 200 ms | 2 | 3072 | 组帧 → I2C 刷屏 | `main/tasks/task_display.c` |
| `task_monitor` | 2000 ms | 1 | 2048 | 运行时健康度 | `main/tasks/task_monitor.c` |

**为什么 IMU 优先级最高、显示最低：** 采样丢了补不回来（时序数据不可重建），显示晚 200 ms 人眼看不出来。
网络任务栈给到 6 KB，因为要拼 JSON 并给 TLS 预留余量。

**为什么采样和判定要拆成两个任务：** 采样必须严格 10 ms 节拍；判定要算浮点（含 `sqrt`），
放在同一个任务里会让采样抖动。中间用无锁环形缓冲解耦，采样端永不因判定慢而阻塞。

**为什么 tick 设成 1000 Hz：** 10 ms 采样周期正好 10 个 tick。若用默认 100 Hz tick，
周期就变成 1 个 tick 的整数倍约束，抖动被放大到 ±10 ms。`sdkconfig.defaults` 里
`CONFIG_FREERTOS_HZ=1000` 就是为这个设的。

---

## 6. 检测算法

四步，全部在 `main/algo/edge_detect.c`。**纯 C、不依赖 ESP-IDF**，所以能在电脑上直接跑真实断言。

### 6.1 去重力（为什么必须先做这一步）

原始加速度里永远含 1 g 重力。直接对 `|a|` 求 RMS 的话，静止读数就是 1 g ——
阈值设 0.55 g 会导致**永久告警**。这是这类项目最常见的坑。

做法是用 IIR 低通估出重力向量，再从测量值里减掉：

```
g_est  ← (1-α)·g_est + α·a          α = 0.02
a_dyn  = a - g_est
```

`α = dt/(τ+dt) = 0.01/(0.5+0.01) ≈ 0.02`，对应时间常数 τ ≈ 500 ms。

- τ 太小 → 设备倾斜时重力估计跟不上，倾斜本身被当成"剧烈振动"
- τ 太大 → 真实振动被当成重力吸收掉，告警迟钝

500 ms 正好是"人手搬运设备"和"机械振动"的分界。首帧直接令 `g_est = a`（`primed` 标志），
避免从零收敛时出现巨大瞬态。

### 6.2 滑窗 RMS

对去重力后的三轴合成幅值做 20 点滑窗均方根（100 Hz × 20 点 = **200 ms 窗**）。

用 RMS 而不是峰值：峰值对单个毛刺极敏感，RMS 反映能量，与振动的物理意义一致。

### 6.3 迟滞状态机

```
NORMAL  ──rms > on──────────▶ WARNING   （越阈，但还没确认）
WARNING ──连续 confirm 次──▶ ALARM     （确认故障 → 发告警）
WARNING ──rms ≤ on─────────▶ NORMAL    （没确认就回落，不打扰）
ALARM   ──连续 release 次──▶ NORMAL    （迟滞退出）
UNKNOWN ──（任何输入）─────▶ UNKNOWN   （传感器不可信，不产告警）
```

默认 `on = 0.55 g`、`off = 0.35 g`（`Kconfig` 可改）。**`off < on` 是关键**：
如果退出也设 0.55，RMS 在阈值附近抖动会让告警反复开关（flapping），运维最后会把整个告警通道屏蔽掉。

WARNING 单独存在的意义：把"越阈"和"确认"拆开。既保留对瞬态冲击的敏感，
又不会让一次采样毛刺触发告警。

**为什么还要第四个状态 `UNKNOWN`：** MPU6050 没初始化成功时，`g_edge.rms` 恒为 0 ——
在只看"有没有超阈"的逻辑里，这恰好是**最健康的样子**，屏幕显示 `NORMAL`、遥测报 `state:"NORMAL"`，
运维完全看不出传感器已经死了。所以初始化失败必须先置 `UNKNOWN`，`feed()` 直接短路不产告警，
上报时写 `state:"UNKNOWN"`，屏幕显示 `RMS --- G`。
**"故障要么被看见，要么被当成正常"** —— 后者是嵌入式里最贵的一类 bug。

### 6.4 告警抑制

| 参数 | 默认 | 作用 |
|---|---|---|
| `cooldown_us` | 5 s | 两次告警的最小间隔，防告警风暴 |
| `realert_us` | 30 s | ALARM 持续期间每 30 s 重发一次，防单条 QoS1 丢失导致上层无感 |

同一轮故障内 `alert_id` 保持不变，让上层做幂等去重（对端 `cache.py` 依赖这一点）。
**跨重启必须变**：`alert_id` 里带一个 `esp_random()` 生成的启动标记，
否则设备重启后时间戳从 0 重新开始，新告警会被上层当成重复直接丢掉。

### 6.5 检测延迟到底是多少

很多简历写"检测延迟 < 50 ms"，这个数字经不起追问。真实推导：

1. 采样 10 ms 一点，窗口 20 点 → 窗口被新数据完全替换需要 **200 ms**
2. 设振动从 0 阶跃到 0.72 g，阈值 0.55 g：RMS = 0.72 × √(k/20)，令其 > 0.55 需 k ≥ 12 → **12 点（120 ms）**越过阈值
3. 再连续 5 点确认 → 第 16 点，即 **160 ms** 产出判定
4. 检测任务 50 ms 一批消费 → 最坏再加 50 ms

**结论：约 200 ms 量级（160 ms 信号 + ≤50 ms 调度）。**

窗口未填满的前 200 ms 内不判警，这是有意的：宁可晚 200 ms，也不要在上电头几帧就误报。

---

## 7. 通信契约

主题与字段定义在 `main/net/protocol.c`，**与 [edge-agent-system](https://github.com/W-JJ-AFK/edge-agent-system) 的 `agent/protocol.py` 逐字段对应**。
这是两个仓库之间唯一的耦合点，改任何一边都必须同步改另一边。

| 主题 | QoS | retain | 方向 | 说明 |
|---|---|---|---|---|
| `devices/{id}/telemetry` | 0 | 否 | ↑ | 周期遥测，丢了下一帧补上 |
| `devices/{id}/status` | 1 | **是** | ↑ | online/offline，同时作为 LWT |
| `devices/{id}/alert` | 1 | 否 | ↑ | 告警，必须到达 |
| `devices/{id}/cmd` | 1 | 否 | ↓ | 下行指令 |
| `devices/{id}/cmd/ack` | 1 | 否 | ↑ | 执行回执 |

**为什么遥测用 QoS0：** 2 s 一帧的连续量，丢一帧无影响，QoS1 的重传与去重成本不划算。
**为什么 status 要 retain：** 新订阅者（比如刚启动的 Agent）需要立刻知道设备在线状态，不用等下一次变化。
**为什么 alert 用 QoS1：** 告警是稀疏事件，丢一条可能意味着漏掉一次故障。

主题字符串的构造与解析**只允许走 `main/net/topic.c`**，不允许在业务代码里手写 `"devices/%s/..."`。
原因是 `cmd/ack` 用 `strstr` 匹配 `cmd` 时会误命中 —— 这类错误在联调时会表现成
"回执被当成新指令"，排查成本极高。`topic_match()` 用精确后缀比较，并且有
`test_topic_contract` 的 35 条断言守着。

### 7.1 为什么一块板子要开两条 MQTT 连接

一块板子对外是**两个 device_id**（`temp_sensor_01` 和 `vibration_01`）。
LWT 是**连接级**属性：一个连接只能挂一条遗嘱，掉线时只能替一个 device_id 发 `offline`。

如果两个 device 共用一个连接，掉电时只有一个设备变成 `offline`，另一个会永远停留在
retained 的 `online` 上 —— 上层看到"设备还活着"，实际早已断电。
所以 `mqtt_mgr.c` 为每个 device_id 各建一个 `esp_mqtt_client`：

```c
static node_client_t s_clients[CLIENT_COUNT] = {
    { .device_id = CONFIG_APP_DEVICE_TEMP_ID, ... },
    { .device_id = CONFIG_APP_DEVICE_VIB_ID,  ... },
};
```

代价是多一个 socket（约 2 KB RAM + 一个 FD），换来的是**两个设备的在线状态都可信**。

### 7.2 时间基准：`timestamp` 可能缺席，`uptime_s` 永远在

ESP32 没有带电池的 RTC，上电时 `time(NULL)` 是 1970。把 1970 写进 `timestamp` 会让上层
按时间排序、算离线、存库全部错乱，而且错得很隐蔽。

| 字段 | 何时出现 | 含义 |
|---|---|---|
| `timestamp` | **只在 SNTP 同步成功后**写入（epoch 秒） | 可信墙上时间 |
| `uptime_s` | 永远出现 | 开机秒数，单调递增，可用于同机内排序 |
| `data.valid` | 传感器失效时出现且为 `false` | 该帧数值不可信 |
| `data.state` | 振动遥测恒有 | `NORMAL/WARNING/ALARM/UNKNOWN` |

**`timestamp` 缺席不是 bug**，它表示"本机还没对上时"。上层 `protocol.py` 会回退到本地接收时间，
并拒绝 `timestamp < 2020` 或超前 7 天以上的值。

报文体：

```jsonc
// telemetry：振动（正常）
{"device_id":"vibration_01","device_type":"vibration_sensor",
 "timestamp":1730000000,"uptime_s":137,"seq":42,
 "data":{"rms":0.062,"peak":0.187,"frequency":50.1,"state":"NORMAL"}}

// telemetry：刚上电、SNTP 未同步（没有 timestamp）；或 IMU 挂了（state=UNKNOWN）
{"device_id":"vibration_01","device_type":"vibration_sensor",
 "uptime_s":4,"seq":1,
 "data":{"valid":false,"state":"UNKNOWN"}}

// telemetry：DHT22 连续 3 次读失败
{"device_id":"temp_sensor_01","device_type":"temperature_sensor",
 "timestamp":1730000000,"uptime_s":137,"seq":42,
 "data":{"valid":false}}

// alert
{"device_id":"vibration_01","alert_id":"vibration_01-3f2a1b-1730000000123-720",
 "alert_type":"vibration_high","value":0.72,"threshold":0.55,
 "severity":"critical","timestamp":1730000000,"uptime_s":137}

// cmd（下行）
{"command_id":"cmd-1730000000-a1b2c3","action":"set_threshold",
 "value":0.8,"issued_by":"agent","expires_at":1730000030}

// ack
{"command_id":"cmd-1730000000-a1b2c3","result":"ok",
 "detail":"threshold=0.80g (release 0.48g)","timestamp":1730000000,"uptime_s":137}
```

`command_id` + `expires_at` 的作用：设备收到过期指令直接丢弃，避免网络延迟导致
"5 分钟前点的降速"在设备已经停机时才执行。**这条校验固件和 Agent 两端都实现了**，
只在一端做等于没做。

---

## 8. 远程指令

| action | 参数范围 | 作用 | 代码 |
|---|---|---|---|
| `set_interval` | 500–60000 ms | 改遥测周期（节点级，两个 device 共用） | `app_main.c` |
| `set_threshold` | 0.05–4.0 g | 改振动阈值（退出值自动取 0.6 倍） | `app_main.c` |
| `reboot` | 无 | 500 ms 后重启 | `app_main.c` |

校验是**双端做**的：设备侧拒绝越界值并回 `result=error`；Agent 侧 `CAPABILITIES` 在下发前就拦一道。
设备侧不信任上游，是嵌入式的基本假设。

IMU 没初始化成功时 `set_threshold` 返回 `imu unavailable, threshold not applied` ——
不假装设置成功，否则上层会以为阈值已经生效。

**`reboot` 为什么延迟 500 ms：** 立刻 `esp_restart()` 会让 ack 还没发出去就断电，
上层看到"指令超时"然后无限重试。用 `esp_timer_start_once` 把重启排到 500 ms 之后，先把 ack 发完。

---

## 9. 验证：四个层次

### 9.1 算法层（不需要硬件，秒级）

```bash
make -C test
# 10 个测试函数 / 129 条断言
# 129 checks, 0 failure(s)
# PASS
```

覆盖：静止不告警、持续振动确认、第 N 次超限才确认、严格大于边界、迟滞不抖动、
冷却抑制、RMS 数值正确性、传感器 `UNKNOWN` 态不产告警、环形缓冲容量与 FIFO 顺序、
主题契约（35 条）。

这一层存在的理由：嵌入式调试成本高（要烧板子、看串口）。把不依赖硬件的逻辑抽成纯 C，
就能在电脑上用 gcc + `assert` 做真实回归测试。CI 里 `host-test` job 就是这么跑的。

### 9.2 编译层（CI 门禁）

`.github/workflows/build.yml` 在 `espressif/idf:release-v5.3` 容器里真跑
`idf.py set-target esp32s3 && idf.py build`，并检查 `build/iot_edge_node.bin` 存在。

这条门禁的意义：防止"README 写了一堆功能，仓库其实编不过"。CI 固定 v5.3 而不是"最新版"，
因为 v5.3.2 是本机实际验证过的版本 —— 写"支持最新版"却没人验证过，是简历项目里最常见的虚标。

### 9.3 真机联调

```bash
# 终端 1：看所有设备状态（含 LWT）
mosquitto_sub -t 'devices/+/status' -v

# 终端 2：手动下发指令
mosquitto_pub -t 'devices/vibration_01/cmd' -q 1 -m \
  '{"command_id":"cmd-manual-1","action":"set_threshold","value":0.8}'

# 终端 3：看回执
mosquitto_sub -t 'devices/vibration_01/cmd/ack' -v
```

用对端仓库的 `make fault` 可以反向注入故障（模拟设备），验证上层链路。

> 说明：本仓库的验证停留在"主机测试 + 真实编译 + 手工联调"。
> **没有做过产线级长期运行验收**，这是它当前最大的短板，见 [11](#11-明确的边界)。

---

## 10. 上板检查清单

按这个顺序走，能少踩 90% 的坑。

**第 1 步：配置**（不改这两项，板子会一直打印 `reconnect #N` 却永远连不上）

```bash
idf.py menuconfig
# Edge Node Configuration
#   WiFi SSID / password   ← 必须 2.4 GHz
#   MQTT broker URI        ← 局域网 IP，不是 localhost
#   SNTP server            ← 默认 ntp.aliyun.com；不通外网就留空
#   device id              ← 必须与 Agent 的 config.yaml 一致
```

设备 ID 默认 `temp_sensor_01` / `vibration_01`，与 `config.yaml` 的 `devices` 键同名。
**改一边必须改另一边**，否则数据会上报但 Agent 认不出来（表现为"这台设备一直未上报"）。

**第 2 步：接线自检**（不接传感器也能上电，日志会告诉你缺什么）

| 日志 | 含义 | 处理 |
|---|---|---|
| `MPU6050 init failed, vibration chain marked UNKNOWN` | I2C 没通（地址 0x68） | 查 SDA/SCL 是否反接、是否共地、模块 AD0 是否需接 |
| `OLED init failed, continuing headless` | 地址 0x3C 无应答 | 不影响功能；查供电与上拉 |
| `DHT22 init failed` / 反复 `read failed (1/3)` | 单总线无响应 | **先接 4.7 kΩ 上拉**，再查 GPIO4 |
| `WiFi SSID is still the placeholder` | 忘了 menuconfig | 回第 1 步 |
| `got ip: 192.168.x.x` | 联网成功 | 继续 |

**第 3 步：看它有没有在说人话**（不依赖 Agent，直接订阅）

```bash
mosquitto_sub -v -t 'devices/+/telemetry'   # 每 2 s 两帧
mosquitto_sub -v -t 'devices/+/status'      # retain：online；拔电后按 keepalive 变 offline
```

**第 4 步：验证下行链路**（最容易因为主题写错而"静默失败"）

```bash
mosquitto_sub -v -t 'devices/vibration_01/cmd/ack' &
mosquitto_pub -q 1 -t 'devices/vibration_01/cmd' -m \
  '{"command_id":"cmd-t1","action":"set_threshold","value":0.8,"expires_at":4102444800}'
# 期望：{"command_id":"cmd-t1","result":"ok","detail":"threshold=0.80g (release 0.48g)",...}
```

`expires_at` 故意给未来时间。若填过去的时间戳，设备会回 `result=error, detail=command expired...`
并拒绝执行 —— 这是设计行为，不是故障。

**第 5 步：接上 Agent**

```bash
# 在 edge-agent-system 里
make agent
💬 有哪些设备      # 期望 temp_sensor_01 / vibration_01 显示"在线 + 实时值"
💬 把振动阈值调到 0.9
💬 电机重启        # 期望被护栏拒绝
```

**第 6 步：制造一次真实告警**（这一步决定项目能不能讲）

对着 MPU6050 连续敲桌面 2~3 秒（幅度 > 0.55 g）：

1. 串口出现 `ALARM rms=0.7xx g (threshold 0.55 g)`
2. OLED 的 `STATE` 从 `NORMAL` → `WARNING` → `ALARM`
3. `devices/vibration_01/alert` 收到 `severity=critical`
4. Agent 里 `查看告警` 能看到这条，`为什么振动传感器告警了？` 能给出原因

想让设备更容易触发就把阈值调**小**（`set_threshold` 只接受 0.05~4.0 g，退出阈值自动取 0.6 倍）。

---

## 11. 明确的边界

写清楚不做什么，比声称什么都做了更可信。以下都是**当前代码里没有**的：

- **没有 TLS**：`mqtt://` 明文。生产必须换 `mqtts://` + 证书，需要额外 40 KB RAM 与证书烧写流程。
- **没有 OTA**：分区表留了 4 MB app，但没实现双分区 OTA 与回滚。
- **没有本地持久化**：断网期间的**遥测**直接丢弃（只保留最新快照）；**告警**有 64 条 RAM 环形积压，但不落 Flash，掉电即失。
- **没有节点级状态主题**：两个 device_id 各一条连接、各自 LWT，没有 `devices/{node}/status` 这类把整板当实体的主题；电量、重启原因目前不上报。
- **没有多设备自注册**：device id 是编译期 Kconfig 常量，同一份固件不能批量部署到不同 ID。
- **没有硬件看门狗兜底**：开了任务看门狗（10 s），但没有外置 WDT 芯片。
- **没有陀螺仪上报**：驱动里有 `mpu6050_read_gyro()`，但没接入上报链路（`task_imu.c` 只用加速度）。
- **没有低功耗管理**：全程满速运行，约 80 mA 量级（WiFi 常连）。电池场景需要 light-sleep + 运动唤醒。
- **没有产线验收**：无长期运行（soak test）数据、无第三方测试报告。所有指标都来自主机测试与手工联调。

---

## 12. 目录结构

```
esp32-sensor-node/
├── CMakeLists.txt              # project(iot_edge_node)
├── sdkconfig.defaults          # 16MB flash / Octal PSRAM / 1ms tick
├── partitions.csv              # nvs + phy_init + factory(4M) + storage(1M)
├── main/
│   ├── common.h                # 引脚、任务周期/优先级/栈、共享快照类型
│   ├── Kconfig.projbuild       # WiFi / broker / device id / 阈值 / SNTP
│   ├── app_main.c              # 初始化 + 指令分发（唯一的业务入口）
│   ├── algo/                   # ★ 纯 C，可在主机上测试
│   │   ├── edge_detect.[ch]    #   去重力 + 滑窗 RMS + 三态机
│   │   └── imu_ring.[ch]       #   无锁 SPSC 环形缓冲
│   ├── drivers/
│   │   ├── dht22.[ch]          #   单总线，临界区只包单 bit
│   │   ├── mpu6050.[ch]        #   DLPF 44Hz / ODR 200Hz / ±2g
│   │   ├── ssd1306.[ch]        #   128x64 页寻址帧缓冲
│   │   └── ssd1306_font.h      #   5x7 字模（由 tools/gen_font.py 生成）
│   ├── net/
│   │   ├── protocol.[ch]       #   cJSON 组包 + 指令解析（契约表在头文件里）
│   │   ├── topic.[ch]          #   主题构造/解析唯一入口（纯 C，可主机测试）
│   │   ├── time_sync.[ch]      #   SNTP，给 timestamp 提供可信时间
│   │   ├── wifi_mgr.[ch]       #   STA + 指数退避重连（永不休止）
│   │   └── mqtt_mgr.[ch]       #   每 device_id 一条连接 + 各自的 LWT
│   └── tasks/                  # 6 个任务，各自一个文件
├── test/
│   ├── Makefile                # gcc，不需要 ESP-IDF
│   └── test_edge_detect.c      # 10 个测试函数 / 129 条断言
└── tools/gen_font.py           # 重新生成字模头文件
```

---

## 13. 关键设计取舍

**采样与判定解耦，用无锁环形缓冲。**
采样任务优先级最高，任何可能阻塞的操作都会造成采样抖动。SPSC 环形缓冲的生产者永不阻塞，
满了就丢新样本（保留旧数据连续性，利于 RMS 计算）。

这里有个经典误解值得单独说：**`volatile` 不能替代原子操作。**
`volatile` 只保证"不被优化掉、每次真读内存"，**不保证顺序**。数据先写 `buf[head]`、后写 `head + 1`，
编译器或双核 Xtensa 都可能让消费者先看到新的 `head`、再看到旧的 `buf`，于是读到一个"新位置上的旧样本"。
所以 `imu_ring.c` 用 release/acquire 配对：

```c
r->buf[head] = *s;                                     // 先写数据
__atomic_store_n(&r->head, next, __ATOMIC_RELEASE);    // 再放行指针

uint32_t head = __atomic_load_n(&r->head, __ATOMIC_ACQUIRE);  // 先取指针
if (head != tail) { *out = r->buf[tail]; ... }               // 再读数据
```

**DHT22 的临界区要尽可能短。**
单总线靠高低电平持续时间编码 bit：每位先 50 µs 低电平，随后高电平 **26~28 µs 表示 0、70 µs 表示 1**。
任务切换会让读到的脉宽完全错乱，所以读位必须关中断。

但"关中断"的粒度决定代价：若用 `portENTER_CRITICAL` 包住整帧（起始信号 + 80 µs 响应 + 40 bit × 130 µs），
累计关中断约 5 ms；而 100 Hz 采样每 10 ms 就要一次，等于一半时间在关中断，采样必然掉点。

现在的做法只包住**单个 bit**（上升沿→下降沿，≤130 µs），起始信号和响应等待都在临界区之外。
总共仍关中断 40 次，但每次极短。判位方式也从"数循环次数"改成**数高电平宽度**（`DHT_ONE_MIN_US 45`），
对主频和优化等级不敏感。

**WiFi 重连"永不放弃"。**
这台设备的正常寿命以年计，路由器重启、停电、改密码都会发生。如果"重试 10 次就收工"，
就必须派人到现场按复位键 —— 对无人值守节点不可接受。所以指数退避、无限重连
（0.5 s → 1 → 2 → 4 → … → 60 s 封顶，成功一次立刻重置计数），并且改成**异步**：
`wifi_mgr_start()` 只表示"已发起连接"，需要阻塞的调用方显式调 `wifi_mgr_wait_connected()`。
这样即使 WiFi 一直连不上，采样/检测/显示任务也照常运行 —— 本地价值不依赖网络。

**断网期间的告警放 RAM 环形队列。**
一次 10 分钟断网若直接丢弃告警，上层只看到"这段时间没有告警"，与"这段时间设备正常"无法区分。
所以 `task_net.c` 维护 64 条环形积压：发不出去的入队，重连后每轮先补发 4 条再处理新告警
（避免瞬时打爆 broker），环形满时**丢最旧**并记数（日志里 `alert_backlog=..(lost N)`）。

为什么不写 Flash：告警是秒级事件，掉电丢失可接受；反复写 Flash 的寿命代价更不值得。
真实产品要上 SPIFFS 环形日志，那是另一个量级的工程。

**不用 `sin()` 造假数据。**
为了让 UI 好看而用 `sin(t)` 生成"振动数据"，会让整条链路的参数失去意义，也无法验证滤波是否有效。
现在所有数据都来自真实 I2C 读取，读失败就上报 `valid:false`，让上层知道数据不可信。

---

## 14. 面试会追问什么

| 问题 | 关键点 | 代码位置 |
|---|---|---|
| RMS 为什么要去重力？ | 不去重力静止时 RMS = 1 g，阈值无法设定 | `edge_detect.c` |
| 迟滞为什么必要？ | 阈值附近抖动会导致告警 flapping，运维会关掉告警 | `edge_detect.h` |
| 为什么还要 `UNKNOWN` 状态？ | 传感器挂了 RMS 恒为 0，恰好"最健康"，必须显式区分 | `edge_detect.c` |
| `volatile` 能替代原子操作吗？ | 不能。不保证顺序，双核下可能读到新指针 + 旧数据 | `imu_ring.c` |
| `uxTaskGetStackHighWaterMark` 单位？ | **word（4 字节）**，不是字节，忘记换算会低估 4 倍 | `task_monitor.c` |
| 优先级反转怎么防？ | `g_state_mutex` 是互斥量，FreeRTOS 默认开优先级继承；持锁时间控制在微秒级 | `app_main.c` |
| QoS0 和 QoS1 怎么选？ | 连续量用 0（丢了有下一帧），稀疏事件用 1（丢了就是漏警） | `task_net.c` |
| LWT 指针为什么要放静态存储？ | ESP-MQTT 保存的是指针不是副本，栈上 buffer 出作用域就悬空 | `mqtt_mgr.c` |
| 一块板两个 device_id，LWT 怎么处理？ | LWT 是连接级属性，必须两条连接 | `mqtt_mgr.c` |
| 为什么 `reboot` 要延迟？ | 立刻重启会让 ack 发不出去，上层判超时再狂重试 | `app_main.c` |
| 设备时间不准会怎样？ | 会把 1970 写进 `timestamp`，上层排序/离线判定全错 | `time_sync.c`、`protocol.c` |
| 检测延迟到底多少？ | 200 ms 量级，推导过程见 [6.5](#65-检测延迟到底是多少) | `edge_detect.c` |

---

## 15. 相关仓库

配套上位机：[**W-JJ-AFK/edge-agent-system**](https://github.com/W-JJ-AFK/edge-agent-system)
—— 自然语言运维 Agent（意图路由 + 本地 LLM 诊断 + 指令回执闭环 + 仪表盘）。

两个仓库通过 [第 7 节](#7-通信契约)的五个主题通信，可以独立部署、独立演示：
没有硬件时用 `simulator`，没有 Agent 时固件照样在本地检测告警。

```
振动超限 → 本地判定 → MQTT alert → Agent 幂等落库 → LLM 解释
                 ↑                                        │
                 └──────── cmd/ack ←── cmd ←── 自然语言指令 ┘
```
