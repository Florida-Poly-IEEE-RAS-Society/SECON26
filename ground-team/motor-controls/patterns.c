#include "patterns.h"
#include <unistd.h>

#define PAUSE_US 500000

void pattern_square(motor_t *m1, motor_t *m2, imu_t *imu, float side_feet) {
  for (int i = 0; i < 4; i++) {
    motors_drive_distance(m1, m2, imu, side_feet);
    usleep(PAUSE_US);
    motors_turn_right(m1, m2, imu);
    usleep(PAUSE_US);
  }
}

void pattern_circle(motor_t *m1, motor_t *m2) {
  motor_set(m1, MOTOR_FORWARD);
  motor_set(m2, MOTOR_STOP);
  usleep(2000000);
  motor_set(m1, MOTOR_STOP);
}

void pattern_figure8(motor_t *m1, motor_t *m2) {
  motor_set(m1, MOTOR_FORWARD);
  motor_set(m2, MOTOR_STOP);
  usleep(2000000);
  motor_set(m1, MOTOR_STOP);
  motor_set(m2, MOTOR_FORWARD);
  usleep(2000000);
  motor_set(m1, MOTOR_STOP);
  motor_set(m2, MOTOR_STOP);
}
