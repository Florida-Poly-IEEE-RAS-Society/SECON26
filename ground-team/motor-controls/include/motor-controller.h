#pragma once

#include "../../core/include/robot-core.h"
#include <stdint.h>

#define MOTOR_I2C_ADDR                                                         \
  0x34 // This is the IC2 address [NOTE: this may change with electrical]
#define MOTOR_TYPE_ADDR 20 // Register address for motor type
#define MOTOR_SPEED_ADDR 51
#define MOTOR_ENCODER_ADDR 31

typedef enum { MOTOR_1 = 1, MOTOR_2 = 2 } motor_id_t;

typedef struct {
  int i2c_fd;
  uint8_t i2c_addr;
} motor_t;

status_t motor_init(motor_t *motor, const char *i2c_device);
status_t motor_set_speed(motor_t *motor, motor_id_t id, int16_t speed);
status_t motor_set_all_speeds(motor_t *motor, int16_t m1, int16_t m2);
status_t motor_stop(motor_t *motor);
status_t motor_get_encoder(motor_t *motor, motor_id_t id, int32_t *count);
void motor_close(motor_t *motor);
