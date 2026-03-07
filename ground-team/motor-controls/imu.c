#include "imu.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define REG_PWR_MGMT_1 0x6B
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_ZOUT_H 0x47
#define GYRO_SCALE 131.0f
#define CAL_SAMPLES 500

static status_t mpu_write(int fd, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = {reg, val};
  if (write(fd, buf, 2) != 2)
    return ERR_WRITE_FAIL;
  return OK;
}

static status_t mpu_read_raw(int fd, uint8_t reg, int16_t *out, int count) {
  if (write(fd, &reg, 1) != 1)
    return ERR_WRITE_FAIL;
  uint8_t buf[count * 2];
  if (read(fd, buf, count * 2) != count * 2)
    return ERR_READ_FAIL;
  for (int i = 0; i < count; i++)
    out[i] = (int16_t)((buf[i * 2] << 8) | buf[i * 2 + 1]);
  return OK;
}

status_t imu_init(imu_t *imu) {
  char path[16];
  snprintf(path, sizeof(path), "/dev/i2c-%d", MPU6050_I2C_BUS);

  imu->fd = open(path, O_RDWR);
  if (imu->fd < 0) {
    perror("Failed to open I2C bus");
    return ERR_BUS_FAIL;
  }

  if (ioctl(imu->fd, I2C_SLAVE, MPU6050_ADDR) < 0) {
    perror("Failed to set I2C address");
    return ERR_ADDRESS_FAIL;
  }

  status_t rc;
  if ((rc = mpu_write(imu->fd, REG_PWR_MGMT_1, 0x00)) != OK)
    return rc;
  if ((rc = mpu_write(imu->fd, REG_GYRO_CONFIG, 0x00)) != OK)
    return rc;
  if ((rc = mpu_write(imu->fd, REG_ACCEL_CONFIG, 0x00)) != OK)
    return rc;
  usleep(100000);

  memset(&imu->offsets, 0, sizeof(imu->offsets));
  return OK;
}

status_t imu_calibrate(imu_t *imu) {
  printf("Calibrating IMU, keep robot still...\n");

  int64_t sums[6] = {0};
  int16_t raw[6];

  for (int i = 0; i < CAL_SAMPLES; i++) {
    status_t rc = mpu_read_raw(imu->fd, REG_ACCEL_XOUT_H, raw, 6);
    if (rc != OK)
      return rc;
    for (int j = 0; j < 6; j++)
      sums[j] += raw[j];
    usleep(2000);
  }

  imu->offsets.ax = sums[0] / CAL_SAMPLES;
  imu->offsets.ay = sums[1] / CAL_SAMPLES;
  imu->offsets.az = sums[2] / CAL_SAMPLES - 16384;
  imu->offsets.gx = sums[3] / CAL_SAMPLES;
  imu->offsets.gy = sums[4] / CAL_SAMPLES;
  imu->offsets.gz = sums[5] / CAL_SAMPLES;

  printf("Offsets | accel (%d %d %d) gyro (%d %d %d)\n", imu->offsets.ax,
         imu->offsets.ay, imu->offsets.az, imu->offsets.gx, imu->offsets.gy,
         imu->offsets.gz);

  FILE *f = fopen(IMU_CAL_FILE, "wb");
  if (!f) {
    perror("Failed to save calibration");
    return ERR_WRITE_FAIL;
  }
  if (fwrite(&imu->offsets, sizeof(imu->offsets), 1, f) != 1) {
    fclose(f);
    return ERR_WRITE_FAIL;
  }
  fclose(f);
  printf("Calibration saved to %s\n", IMU_CAL_FILE);
  return OK;
}

status_t imu_load_cal(imu_t *imu) {
  FILE *f = fopen(IMU_CAL_FILE, "rb");
  if (!f)
    return ERR_BUS_FAIL;

  if (fread(&imu->offsets, sizeof(imu->offsets), 1, f) != 1) {
    fclose(f);
    return ERR_READ_FAIL;
  }
  fclose(f);
  printf("Calibration loaded from %s\n", IMU_CAL_FILE);
  return OK;
}

status_t imu_read_gyro_z(imu_t *imu, float *out) {
  int16_t raw[1];
  status_t rc = mpu_read_raw(imu->fd, REG_GYRO_ZOUT_H, raw, 1);
  if (rc != OK)
    return rc;
  *out = (float)(raw[0] - imu->offsets.gz) / GYRO_SCALE;
  return OK;
}

void imu_cleanup(imu_t *imu) { close(imu->fd); }
