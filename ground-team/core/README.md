# RAS Core Library (SECON2026)

The RAS Core Library is a single C header file (`robot-core.h`) that provides essential utilities, mathematical functions, and standardized status codes for SECON2026 robot projects. It is designed to be portable and is suitable for both embedded and Linux-based systems. This document outlines the library's features, functions, enums, and structs.

-----

## Overview

The library simplifies robotics development by providing a set of key tools, including status codes for error handling, mathematical utilities for common calculations, and a PID controller for stable system control. Its lightweight design ensures broad compatibility, giving developers consistent, reusable code for efficient project integration.

### Usage Example

```c
#include "robot-core.h"
#include <stdio.h>

int main(void) {
    // Convert degrees to radians
    float angle_deg = 90.0f;
    float angle_rad = DEG_TO_RAD * angle_deg;
    printf("Angle in radians: %.3f\n", angle_rad);

    // Initialize and use a PID controller
    PID_t motor_pid = { .kp = 1.0f, .ki = 0.1f, .kd = 0.05f };
    float setpoint = 100.0f;
    float measured = 80.0f;
    float dt = 0.01f;

    float output = pid_compute(&motor_pid, setpoint, measured, dt);
    printf("PID output: %.2f\n", output);

    return 0;
}
```

This example demonstrates how to use the `DEG_TO_RAD` constant and the `PID_t` struct with the `pid_compute` function.

-----

## Status Codes

The library uses a set of standardized **status codes** for consistent error handling across all modules. The `status_t` enum ensures clear diagnostics and simplifies debugging.

| Code | Value | Description |
| :--- | :--- | :--- |
| `OK` | `0` | Operation successful |
| `ERR_BUS_FAIL` | `-1` | I2C/SPI bus opening failure or peripheral not found |
| `ERR_CONFIG_FAIL` | `-2` | Failed to set slave address or configure a device |
| `ERR_WRITE_FAIL` | `-3` | Failed to write data |
| `ERR_READ_FAIL` | `-4` | Failed to read data |
| `ERR_INVALID_ARG` | `-5` | Invalid argument (out of range or bad ID) |
| `ERR_NOT_INIT` | `-6` | Function called before initialization |

Example usage for robust error checking:

```c
status_t imu_init() {
    if (!bus_found()) return ERR_BUS_FAIL;
    return OK;
}
```

-----

## Mathematical Utilities

The library provides a set of mathematical functions and constants for common robotics tasks, such as limiting values, filtering noise, and normalizing angles.

### Functions

#### clamp

`float clamp(float x, float min, float max)`

This function restricts the value `x` to a specified range `[min, max]`. It is useful for limiting motor speeds or sensor outputs. It returns `min` if `x` is less than `min`, `max` if `x` is greater than `max`, and `x` otherwise.

#### deadband

`float deadband(float x, float threshold)`

This function zeros the value `x` if it is within `[-threshold, threshold]`, effectively reducing noise in sensor readings. It returns `0` if the absolute value of `x` is less than or equal to `threshold`, and `x` otherwise.

(I dont know how useful this will be for SECON2026 unless more sensors get added to the ground robot)

#### wrap_angle

`float wrap_angle(float angle)`

This function normalizes an angle to the range `[-180°, 180°]` to ensure consistent representations in motion planning. For example, `wrap_angle(190.0f)` returns `-170.0f`.

### Constants

  * `DEG_TO_RAD`: A `float` constant used to convert an angle from degrees to radians.
  * `RAD_TO_DEG`: A `float` constant used to convert an angle from radians to degrees.

-----

## PID Controller

The PID controller manages dynamic systems like motors and servos to ensure stable convergence to a target value. It is defined by a struct that holds its state and a function that performs the control calculation.

### PID Structure

The `PID_t` struct holds the controller's parameters and state.

| Member | Type | Description |
| :--- | :--- | :--- |
| `kp` | `float` | **Proportional gain**: Scales the current error to adjust responsiveness. |
| `ki` | `float` | **Integral gain**: Scales accumulated error to eliminate steady-state error. |
| `kd` | `float` | **Derivative gain**: Scales the error rate to dampen overshoot. |
| `prev_error` | `float` | The previous error, used for the derivative calculation. |
| `integral` | `float` | The accumulated sum of errors, used for integral control. |

### PID Function

The `pid_compute` function calculates the control output based on the controller's state and current measurements.

`float pid_compute(PID_t *pid, float setpoint, float measured, float dt)`

  * `pid`: A pointer to the `PID_t` struct containing the controller's state.
  * `setpoint`: The desired target value, such as a motor's speed.
  * `measured`: The current measured value from the system.
  * `dt`: The time step in seconds since the last computation.

The function returns the control output to drive the system toward the `setpoint`. Tuning the `kp`, `ki`, and `kd` values is crucial for optimizing system behavior.
