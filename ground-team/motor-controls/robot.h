#pragma once

#include "../core/robot-core.h"
#include "encoder.h"
#include "imu.h"
#include "motor_control.h"

#define DRIVE_CORRECTION_DEG 2.0f
#define DRIVE_LOOP_US 2000

typedef struct {
  motor_t m1;
  motor_t m2;
  imu_t imu;
  encoder_t enc1;
  encoder_t enc2;
  pos2_t pos;
  int gpio_handle;
} robot_t;

status_t robot_init(robot_t *bot);
status_t robot_drive(robot_t *bot, float inches);
status_t robot_turn(robot_t *bot, float degrees);
status_t robot_swing(robot_t *bot, float degrees, int pivot_left);
void robot_cleanup(robot_t *bot);

static inline status_t forward(robot_t *bot, float inches) {
  return robot_drive(bot, inches);
}
static inline status_t backward(robot_t *bot, float inches) {
  return robot_drive(bot, -inches);
}
static inline status_t right(robot_t *bot) { return robot_turn(bot, 90.0f); }
static inline status_t left(robot_t *bot) { return robot_turn(bot, -90.0f); }
static inline status_t u_turn(robot_t *bot) { return robot_turn(bot, 180.0f); }
static inline status_t swing_right(robot_t *bot, float degrees) {
  return robot_swing(bot, degrees, 0);
}
static inline status_t swing_left(robot_t *bot, float degrees) {
  return robot_swing(bot, -degrees, 1);
}
