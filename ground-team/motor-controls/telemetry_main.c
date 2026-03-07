#include "imu.h"
#include "motor_control.h"
#include "telemetry.h"
#include <signal.h>

static motor_t m1, m2;
static imu_t imu;
static telemetry_t telem;

static void cleanup(int sig) {
  (void)sig;
  telemetry_cleanup(&telem);
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

  if (telemetry_init(&telem, &m1, &m2, &imu) != OK)
    return 1;

  telemetry_run(&telem);

  motors_cleanup(&m1, &m2);
  imu_cleanup(&imu);
  return 0;
}
