/**
 * @file dht22.h
 * @brief DHT22 温湿度传感器驱动（单总线，位操作时序实现）。
 *
 * 注意：读取期间会关闭中断约 4~5ms（DHT22 位宽只有 26~70us，被中断打断会误判），
 * 因此必须在任务上下文调用，不要在中断或临界区内部调用。
 */

#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

/**
 * @brief 初始化数据引脚（含内部上拉）。
 * @param pin 单总线数据引脚
 */
esp_err_t dht22_init(gpio_num_t pin);

/**
 * @brief 读取一次温湿度。
 *
 * 内部会强制满足 2s 最小采样间隔并最多重试 3 次；
 * 单次调用最坏耗时约 2s（等待间隔）+ 3 × 5ms。
 *
 * @param temperature_c 温度（摄氏度，已处理负温度符号位）
 * @param humidity_pct  相对湿度（百分比）
 * @return ESP_OK 成功；ESP_ERR_INVALID_CRC 校验失败；ESP_ERR_TIMEOUT 时序超时
 */
esp_err_t dht22_read(float *temperature_c, float *humidity_pct);
