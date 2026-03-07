#pragma once

#include <lgpio.h>
#include <stdio.h>
#include <unistd.h>

#include "../core/src/robot-core.c"

#define MOTOR1_IN1 12 // Forward
#define MOTOR1_IN2 16 // Backward
#define MOTOR2_IN1 20 // Backward
#define MOTOR2_IN2 21 // Forward

#define SECS_PER_FOOT 1.0f // Calibrate: seconds to travel 1 foot
#define SECS_PER_360 2.0f  // Calibrate: seconds to spin 360 degrees

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

int motors_init(motor_t *m1, motor_t *m2);
void motor_set(motor_t *m, motordir_t dir);
void motors_drive_distance(motor_t *m1, motor_t *m2, float feet);
void motors_spin(motor_t *m1, motor_t *m2, float degrees);
void motors_cleanup(motor_t *m1, motor_t *m2);
