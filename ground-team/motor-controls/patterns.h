#pragma once

#include "imu.h"
#include "motor_control.h"

void pattern_square(motor_t *m1, motor_t *m2, imu_t *imu, float side_feet);
void pattern_circle(motor_t *m1, motor_t *m2);
void pattern_figure8(motor_t *m1, motor_t *m2);
