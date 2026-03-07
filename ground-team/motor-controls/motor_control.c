#include "motor_control.h"
#include <time.h>

status_t motors_init(motor_t *m1, motor_t *m2) {
  int handle = lgGpiochipOpen(0);
  if (handle < 0) {
    fprintf(stderr, "Failed to open GPIO chip: %s\n", lguErrorText(handle));
    return ERR_BUS_FAIL;
  }

  int pins[] = {MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2};
  for (int i = 0; i < 4; i++) {
    int rc = lgGpioClaimOutput(handle, 0, pins[i], 0);
    if (rc < 0) {
      fprintf(stderr, "Failed to claim GPIO %d: %s", pins[i], lguErrorText(rc));
      lgGpiochipClose(handle);
      return ERR_WRITE_FAIL;
    }
  }

  m1->handle = handle;
  m1->in1 = MOTOR1_IN1;
  m1->in2 = MOTOR1_IN2;

  m2->handle = handle;
  m2->in1 = MOTOR2_IN1;
  m2->in2 = MOTOR2_IN2;

  return OK;
}

void motor_set(motor_t *m, motordir_t dir) {
  int a, b;
  switch (dir) {
  case MOTOR_FORWARD:
    a = 1;
    b = 0;
    break;
  case MOTOR_BACKWARD:
    a = 0;
    b = 1;
    break;
  default:
    a = 0;
    b = 0;
    break;
  }
  lgGpioWrite(m->handle, m->in1, a);
  lgGpioWrite(m->handle, m->in2, b);
}

void motors_drive_distance(motor_t *m1, motor_t *m2, float feet) {
  motordir_t dir = feet >= 0 ? MOTOR_FORWARD : MOTOR_BACKWARD;
  if (feet < 0)
    feet = -feet;

  unsigned int us = (unsigned int)(feet * SECS_PER_FOOT * 1e6f);

  motor_set(m1, dir);
  motor_set(m2, dir);
  usleep(us);
  motor_set(m1, MOTOR_STOP);
  motor_set(m2, MOTOR_STOP);
}

void motors_spin(motor_t *m1, motor_t *m2, imu_t *imu, float degrees) {
  int clockwise = degrees >= 0;
  if (degrees < 0)
    degrees = -degrees;

  if (clockwise) {
    motor_set(m1, MOTOR_FORWARD);
    motor_set(m2, MOTOR_BACKWARD);
  } else {
    motor_set(m1, MOTOR_BACKWARD);
    motor_set(m2, MOTOR_FORWARD);
  }

  float turned = 0.0f;
  struct timespec prev, now;
  clock_gettime(CLOCK_MONOTONIC, &prev);

  while (turned < degrees) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    float dt =
        (now.tv_sec - prev.tv_sec) + (now.tv_nsec - prev.tv_nsec) * 1e-9f;
    prev = now;

    float gz;
    if (imu_read_gyro_z(imu, &gz) != OK)
      break;
    turned += (gz < 0 ? -gz : gz) * dt;

    usleep(2000);
  }

  motor_set(m1, MOTOR_STOP);
  motor_set(m2, MOTOR_STOP);
}

void motors_turn_right(motor_t *m1, motor_t *m2, imu_t *imu) {
  motors_spin(m1, m2, imu, 90.0f);
}

void motors_turn_left(motor_t *m1, motor_t *m2, imu_t *imu) {
  motors_spin(m1, m2, imu, -90.0f);
}

void motors_cleanup(motor_t *m1, motor_t *m2) {
  motor_set(m1, MOTOR_STOP);
  motor_set(m2, MOTOR_STOP);
  lgGpiochipClose(m1->handle);
}
