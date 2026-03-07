#include "../include/motor-controller.h"
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

status_t motor_init(motor_t *motor, const char *i2c_device) {
  if (!motor || !i2c_device)
    return ERR_INVALID_ARG;

  motor->i2c_addr = MOTOR_I2C_ADDR;
  motor->i2c_fd = open(i2c_device, O_RDWR);

  if (motor->i2c_fd < 0) {
    return ERR_BUS_FAIL;
  }

  if (ioctl(motor->i2c_fd, I2C_SLAVE, motor->i2c_addr) < 0) {
    close(motor->i2c_fd);
    return ERR_ADDRESS_FAIL;
  }

  return OK;
}

status_t motor_set_speed(motor_t *motor, motor_id_t id, int16_t speed) {
  if (!motor)
    return ERR_INVALID_ARG;

  uint8_t buf[5];
  buf[0] = MOTOR_SPEED_ADDR;
  buf[1] = id;
  buf[2] = (speed >> 8) & 0xFF;
  buf[3] = speed & 0xFF;
  buf[4] = 0x00;

  if (write(motor->i2c_fd, buf, 5) != 5) {
    return ERR_WRITE_FAIL;
  }

  return OK;
}

status_t motor_set_all_speeds(motor_t *motor, int16_t m1, int16_t m2) {
  if (!motor)
    return ERR_INVALID_ARG;

  uint8_t buf[5];
  buf[0] = MOTOR_SPEED_ADDR;
  buf[1] = (m1 >> 8) & 0xFF;
  buf[2] = m1 & 0xFF;
  buf[3] = (m2 >> 8) & 0xFF;
  buf[4] = m2 & 0xFF;
  if (write(motor->i2c_fd, buf, 9) != 9) {
    return ERR_WRITE_FAIL;
  }

  return OK;
}

status_t motor_stop(motor_t *motor) {
  return motor_set_all_speeds(motor, 0, 0);
}

status_t motor_get_encoder(motor_t *motor, motor_id_t id, int32_t *count) {
  if (!motor || !count)
    return ERR_INVALID_ARG;

  uint8_t reg = MOTOR_ENCODER_ADDR + (id - 1) * 4;

  if (write(motor->i2c_fd, &reg, 1) != 1) {
    return ERR_WRITE_FAIL;
  }

  uint8_t data[4];
  if (read(motor->i2c_fd, data, 4) != 4) {
    return ERR_READ_FAIL;
  }

  *count = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];

  return OK;
}

void motor_close(motor_t *motor) {
  if (motor && motor->i2c_fd >= 0) {
    close(motor->i2c_fd);
    motor->i2c_fd = -1;
  }
}
