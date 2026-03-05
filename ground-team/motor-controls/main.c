#include "motor_control.h"
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

#define PAUSE_US 500000

static motor_t m1, m2;

static void cleanup(int sig) {
  (void)sig;
  motors_cleanup(&m1, &m2);
  _exit(0);
}

static void forward(float feet) {
  printf("Forward %.1f ft\n", feet);
  motors_drive_distance(&m1, &m2, feet);
  usleep(PAUSE_US);
}

static void backward(float feet) {
  printf("Backward %.1f ft\n", feet);
  motors_drive_distance(&m1, &m2, -feet);
  usleep(PAUSE_US);
}

static void spin_360(void) {
  printf("360\n");
  motors_spin(&m1, &m2, 360.0f);
  usleep(PAUSE_US);
}

static void spin_180(void) {
  printf("180\n");
  motors_spin(&m1, &m2, 180.0f);
  usleep(PAUSE_US);
}

static void pattern_square(float side_feet) {
  printf("--- Square (%.1f ft sides) ---\n", side_feet);
  for (int i = 0; i < 4; i++) {
    forward(side_feet);
    motors_spin(&m1, &m2, 90.0f);
    usleep(PAUSE_US);
  }
}

static void pattern_circle(void) {
  printf("--- Circle ---\n");
  motor_set(&m1, MOTOR_FORWARD);
  motor_set(&m2, MOTOR_STOP);
  usleep((unsigned int)(SECS_PER_360 * 1e6f));
  motor_set(&m1, MOTOR_STOP);
  motor_set(&m2, MOTOR_STOP);
  usleep(PAUSE_US);
}

static void pattern_figure8(void) {
  printf("--- Figure-8 ---\n");
  motor_set(&m1, MOTOR_FORWARD);
  motor_set(&m2, MOTOR_STOP);
  usleep((unsigned int)(SECS_PER_360 * 1e6f));
  motor_set(&m1, MOTOR_STOP);
  motor_set(&m2, MOTOR_FORWARD);
  usleep((unsigned int)(SECS_PER_360 * 1e6f));
  motor_set(&m1, MOTOR_STOP);
  motor_set(&m2, MOTOR_STOP);
  usleep(PAUSE_US);
}

int main(void) {
  signal(SIGINT, cleanup);

  if (motors_init(&m1, &m2) != OK)
    return 1;

  forward(1.0f);
  backward(1.0f);
  spin_180();
  spin_360();
  pattern_square(1.0f);
  pattern_circle();
  pattern_figure8();

  motors_cleanup(&m1, &m2);
  return 0;
}
