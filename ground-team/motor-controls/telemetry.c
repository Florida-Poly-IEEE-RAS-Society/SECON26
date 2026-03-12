#include "telemetry.h"
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#define STEP_FEET 0.25f
#define TURN_DEG 90.0f

static struct termios orig_termios;

static void term_raw(void) {
  struct termios raw;
  tcgetattr(STDIN_FILENO, &orig_termios);
  raw = orig_termios;
  raw.c_lflag &= ~(ECHO | ICANON);
  raw.c_cc[VMIN] = 1;
  raw.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
}

static void term_restore(void) {
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios);
}

status_t telemetry_init(telemetry_t *telem, motor_t *m1, motor_t *m2,
                        imu_t *imu) {
  if (!telem || !m1 || !m2 || !imu)
    return ERR_INVALID_ARG;
  telem->m1 = m1;
  telem->m2 = m2;
  telem->imu = imu;
  return OK;
}

status_t telemetry_run(telemetry_t *telem) {
  if (!telem)
    return ERR_INVALID_ARG;

  term_raw();

  printf("Telemetry active\n");

  char c;
  while (read(STDIN_FILENO, &c, 1) == 1) {
    float gz;
    switch (c) {
    case 'w':
    case 'W':
      printf("Forward %.2f ft\n", STEP_FEET);
      motors_drive_distance(telem->m1, telem->m2, STEP_FEET);
      break;
    case 's':
    case 'S':
      printf("Backward %.2f ft\n", STEP_FEET);
      motors_drive_distance(telem->m1, telem->m2, -STEP_FEET);
      break;
    case 'a':
    case 'A':
      printf("Turn left\n");
      motors_turn_left(telem->m1, telem->m2, telem->imu);
      break;
    case 'd':
    case 'D':
      printf("Turn right\n");
      motors_turn_right(telem->m1, telem->m2, telem->imu);
      break;
    case 'i':
    case 'I':
      if (imu_read_gyro_z(telem->imu, &gz) == OK)
        printf("Gyro Z: %.3f deg/s\n", gz);
      else
        printf("IMU read failed\n");
      break;
    case 'q':
    case 'Q':
      printf("Exiting telemetry\n");
      term_restore();
      return OK;
    default:
      break;
    }
  }

  term_restore();
  return OK;
}

void telemetry_cleanup(telemetry_t *telem) {
  if (!telem)
    return;
  term_restore();
}
