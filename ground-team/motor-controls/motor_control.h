#pragma once

#include <lgpio.h>
#include <stdio.h>
#include <unistd.h>

#include "../core/include/robot-core.h"
#include "imu.h"

#define MOTOR1_IN1 12 // Forward
#define MOTOR1_IN2 16 // Backward
#define MOTOR2_IN1 21 // Forward
#define MOTOR2_IN2 20 // Backward

#define SECS_PER_FOOT 0.85f // Calibrate: seconds to travel 1 foot

typedef enum MotorDirection {
  MOTOR_FORWARD,
  MOTOR_BACKWARD,
  MOTOR_STOP
} motordir_t;

typedef struct Motor {
  int handle;
  int in1;
  int in2;
} motor_t;

status_t motors_init(motor_t *m1, motor_t *m2);
void motor_set(motor_t *m, motordir_t dir);
void motors_cleanup(motor_t *m1, motor_t *m2);
