#pragma once

#include "../core/robot-core.h"
#include <stdint.h>

#define MPU6050_ADDR 0x68
#define MPU6050_I2C_BUS 1
#define IMU_CAL_FILE "imu_cal.bin"

typedef struct {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
} imu_offsets_t;

typedef struct {
  int fd;
  imu_offsets_t offsets;
} imu_t;

status_t imu_init(imu_t *imu);
status_t imu_calibrate(imu_t *imu);
status_t imu_load_cal(imu_t *imu);
status_t imu_read_gyro_z(imu_t *imu, float *out);
void imu_cleanup(imu_t *imu);
