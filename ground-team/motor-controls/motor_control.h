#pragma once

#include "../core/robot-core.h"
#include "imu.h"
#include <lgpio.h>
#include <stdio.h>
#include <unistd.h>

#define MOTOR1_IN1 12
#define MOTOR1_IN2 16
#define MOTOR2_IN1 21
#define MOTOR2_IN2 20

typedef enum { MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_STOP } motordir_t;

typedef struct {
  int handle;
  int in1;
  int in2;
} motor_t;

status_t motors_init(motor_t *m1, motor_t *m2);
void motor_set(motor_t *m, motordir_t dir);
void motors_cleanup(motor_t *m1, motor_t *m2);
