# ESP32-S3 多传感器边缘节点

基于 ESP-IDF 的 ESP32-S3 传感器节点，集成 OLED 显示屏、DHT22 温湿度传感器、MPU6050 六轴传感器，实现 FreeRTOS 多任务架构和边缘实时异常检测。

## 硬件

- ESP32-S3 N16R8 (16MB Flash + 8MB PSRAM)
- 0.96寸 OLED (I2C, 地址 0x3C)
- DHT22 温湿度传感器 (单总线)
- MPU6050 六轴传感器 (I2C, 地址 0x68)

## 引脚连接

| 传感器 | 引脚 | ESP32-S3 |
|--------|------|----------|
| OLED SCL | I2C SCL | GPIO 9 |
| OLED SDA | I2C SDA | GPIO 8 |
| MPU6050 SCL | I2C SCL | GPIO 9 |
| MPU6050 SDA | I2C SDA | GPIO 8 |
| DHT22 DAT | 单总线 | GPIO 4 |

## 功能

### 1. 多传感器采集
- OLED 实时显示温湿度 + 振动 RMS
- 每 2 秒通过 MQTT 上报数据到 AI Agent 运维系统

### 2. FreeRTOS 多任务架构
- 5 任务并发：生产者(100Hz) / 消费者(滤波) / 监控器 / 心跳 / 信号量演示
- 队列实现生产者-消费者模型
- 互斥信号量保护串口共享资源
- 任务栈高水位监控

### 3. 边缘实时异常检测
- 100Hz 振动采样
- 20 点滑动窗口 + RMS 计算
- 连续 5 次超阈值触发告警（抑制瞬态噪声）
- 检测延迟 < 50ms

## 编译

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COM13 flash monitor
技术栈
ESP-IDF | C | FreeRTOS | I2C | MQTT | 边缘计算
