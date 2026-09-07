#ifndef MPU6050_H
#define MPU6050_H

#include "driver/i2c_master.h"

void mpu6050_init(i2c_master_bus_handle_t bus);
void mpu6050_read_accel(float *ax, float *ay, float *az);
float mpu6050_calc_rms(float ax, float ay, float az);

#endif