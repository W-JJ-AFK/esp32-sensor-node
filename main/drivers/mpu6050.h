/**
 * @file mpu6050.h
 * @brief MPU6050 六轴传感器驱动（ESP-IDF v5.2+ 新版 i2c_master 驱动）。
 *
 * 初始化时已经配置好振动检测所需的采样链路：
 *   - 时钟源选 X 轴陀螺 PLL（比内部 RC 稳）
 *   - DLPF 带宽 44Hz（100Hz 采样下的抗混叠，避免机械高频折叠到低频）
 *   - 采样率 200Hz（对 100Hz 采样留 2 倍余量）
 *   - 加速度量程 ±2g（16384 LSB/g）
 */

#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus, uint8_t address);

/**
 * @brief 读取加速度，单位 g。
 * @return ESP_OK 或 I2C 错误码（调用方不应在失败时使用输出值）
 */
esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az);

/** @brief 读取角速度，单位 deg/s（±250 dps 量程）。 */
esp_err_t mpu6050_read_gyro(float *gx, float *gy, float *gz);

/** @brief 读取芯片温度，单位摄氏度。 */
esp_err_t mpu6050_read_temp(float *celsius);
