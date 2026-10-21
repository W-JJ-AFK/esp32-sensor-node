#include "mpu6050.h"

#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MPU6050";

#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_XOUT_H  0x3B
#define REG_TEMP_OUT_H    0x41
#define REG_GYRO_XOUT_H   0x43
#define REG_PWR_MGMT_1    0x6B
#define REG_WHO_AM_I      0x75

#define ACCEL_LSB_PER_G   16384.0f /* ±2g */
#define GYRO_LSB_PER_DPS  131.0f   /* ±250 dps */
#define I2C_TIMEOUT_MS    100

static i2c_master_dev_handle_t s_dev;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    const uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), I2C_TIMEOUT_MS);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *out, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, out, len, I2C_TIMEOUT_MS);
}

esp_err_t mpu6050_init(i2c_master_bus_handle_t bus, uint8_t address)
{
    if (bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = address,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_dev), TAG,
                        "add i2c device failed");

    uint8_t who_am_i = 0;
    esp_err_t err = read_regs(REG_WHO_AM_I, &who_am_i, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        return err;
    }
    if (who_am_i != 0x68 && who_am_i != 0x70) {
        ESP_LOGW(TAG, "unexpected WHO_AM_I=0x%02X (expect 0x68)", who_am_i);
    }

    /* 1) 唤醒并选择时钟源：0x01 = PLL with X axis gyroscope reference */
    ESP_RETURN_ON_ERROR(write_reg(REG_PWR_MGMT_1, 0x01), TAG, "wake failed");
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 2) 数字低通滤波：DLPF_CFG=3 -> 加速度带宽 44Hz、延迟 4.9ms
     *    100Hz 采样要求信号带宽 < 50Hz，否则高频振动会混叠成低频误判 */
    ESP_RETURN_ON_ERROR(write_reg(REG_CONFIG, 0x03), TAG, "DLPF config failed");

    /* 3) 采样率 = 1kHz / (1 + SMPLRT_DIV) = 200Hz */
    ESP_RETURN_ON_ERROR(write_reg(REG_SMPLRT_DIV, 0x04), TAG, "sample rate failed");

    /* 4) 量程：加速度 ±2g，陀螺 ±250dps */
    ESP_RETURN_ON_ERROR(write_reg(REG_ACCEL_CONFIG, 0x00), TAG, "accel range failed");
    ESP_RETURN_ON_ERROR(write_reg(REG_GYRO_CONFIG, 0x00), TAG, "gyro range failed");

    ESP_LOGI(TAG, "initialized (WHO_AM_I=0x%02X, DLPF=44Hz, 200Hz ODR, ±2g)", who_am_i);
    return ESP_OK;
}

esp_err_t mpu6050_read_accel(float *ax, float *ay, float *az)
{
    if (ax == NULL || ay == NULL || az == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[6] = {0};
    ESP_RETURN_ON_ERROR(read_regs(REG_ACCEL_XOUT_H, raw, sizeof(raw)), TAG,
                        "accel read failed");

    const int16_t x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    const int16_t y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    const int16_t z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    *ax = (float)x / ACCEL_LSB_PER_G;
    *ay = (float)y / ACCEL_LSB_PER_G;
    *az = (float)z / ACCEL_LSB_PER_G;
    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(float *gx, float *gy, float *gz)
{
    if (gx == NULL || gy == NULL || gz == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[6] = {0};
    ESP_RETURN_ON_ERROR(read_regs(REG_GYRO_XOUT_H, raw, sizeof(raw)), TAG,
                        "gyro read failed");

    const int16_t x = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    const int16_t y = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    const int16_t z = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    *gx = (float)x / GYRO_LSB_PER_DPS;
    *gy = (float)y / GYRO_LSB_PER_DPS;
    *gz = (float)z / GYRO_LSB_PER_DPS;
    return ESP_OK;
}

esp_err_t mpu6050_read_temp(float *celsius)
{
    if (celsius == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t raw[2] = {0};
    ESP_RETURN_ON_ERROR(read_regs(REG_TEMP_OUT_H, raw, sizeof(raw)), TAG,
                        "temp read failed");

    const int16_t value = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    *celsius = (float)value / 340.0f + 36.53f;
    return ESP_OK;
}
