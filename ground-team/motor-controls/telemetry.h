#pragma once

#include "../core/include/robot-core.h"
#include "imu.h"
#include "motor_control.h"

typedef struct {
  motor_t *m1;
  motor_t *m2;
  imu_t *imu;
} telemetry_t;

status_t telemetry_init(telemetry_t *telem, motor_t *m1, motor_t *m2,
                        imu_t *imu);
status_t telemetry_run(telemetry_t *telem);
void telemetry_cleanup(telemetry_t *telem);
