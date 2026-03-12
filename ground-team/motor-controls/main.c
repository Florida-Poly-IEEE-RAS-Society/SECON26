#include "imu.h"
#include "motor_control.h"
#include "patterns.h"
#include <signal.h>
#include <unistd.h>

#define PAUSE_US 500000

static motor_t m1, m2;
static imu_t imu;

static void cleanup(int sig) {
  (void)sig;
  motors_cleanup(&m1, &m2);
  imu_cleanup(&imu);
  _exit(0);
}

int main(void) {
  signal(SIGINT, cleanup);

  if (imu_init(&imu) != OK)
    return 1;

  if (imu_load_cal(&imu) != OK)
    if (imu_calibrate(&imu) != OK)
      return 1;

  if (motors_init(&m1, &m2) != OK)
    return 1;

  motors_drive_distance(&m1, &m2, &imu, 1.0f);

  motors_cleanup(&m1, &m2);
  imu_cleanup(&imu);
  return 0;
}
