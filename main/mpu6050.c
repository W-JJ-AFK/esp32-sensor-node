#include "mpu6050.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MPU6050";
static i2c_master_dev_handle_t mpu_dev;

// MPU6050 寄存器地址
#define MPU6050_REG_PWR_MGMT_1   0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_WHO_AM_I     0x75
#define MPU6050_REG_CONFIG       0x1A
#define MPU6050_REG_ACCEL_CONFIG 0x1C

#define MPU6050_ADDR 0x68  // AD0 接地时地址

void mpu6050_init(i2c_master_bus_handle_t bus) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &mpu_dev));
    
    // 读取 WHO_AM_I 验证
    uint8_t who_am_i = 0;
    uint8_t reg = MPU6050_REG_WHO_AM_I;
    i2c_master_transmit_receive(mpu_dev, &reg, 1, &who_am_i, 1, 100);
    ESP_LOGI(TAG, "WHO_AM_I = 0x%02X (期望 0x68)", who_am_i);
    
    // 唤醒 MPU6050
    uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    i2c_master_transmit(mpu_dev, wake_cmd, 2, 100);
    
    // 加速度计配置：±2g
    uint8_t accel_cfg[2] = {MPU6050_REG_ACCEL_CONFIG, 0x00};
    i2c_master_transmit(mpu_dev, accel_cfg, 2, 100);
    
    ESP_LOGI(TAG, "MPU6050 初始化完成");
}

void mpu6050_read_accel(float *ax, float *ay, float *az) {
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t data[6] = {0};
    i2c_master_transmit_receive(mpu_dev, &reg, 1, data, 6, 100);
    
    int16_t raw_ax = (data[0] << 8) | data[1];
    int16_t raw_ay = (data[2] << 8) | data[3];
    int16_t raw_az = (data[4] << 8) | data[5];
    
    // ±2g 满量程，16384 LSB/g
    *ax = raw_ax / 16384.0f;
    *ay = raw_ay / 16384.0f;
    *az = raw_az / 16384.0f;
}

float mpu6050_calc_rms(float ax, float ay, float az) {
    // 合加速度（重力分量需要减去，简化版直接算）
    float magnitude = sqrtf(ax*ax + ay*ay + az*az);
    return magnitude;
}