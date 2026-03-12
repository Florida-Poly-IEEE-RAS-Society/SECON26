// robot-core.h
//
// RAS Core Robotics Library
// SECON 2026 Tank Chassis Ground Robot
//
// Motor Model
// JGB3865 520R45 12 DC Gearmotor
//
// Motor specifications
// Rated voltage:   12 V
// Operating range: 7 to 13 V
// Gear ratio:      45 to 1
// Motor speed:     about 6500 RPM before gearbox
// Output speed:    about 150 RPM after gearbox
//
// Encoder
// Hall effect quadrature encoder mounted on the motor shaft.
// Pulses occur before the gearbox so the gear ratio multiplies
// the effective counts per output revolution.
//
// Counts per output revolution
// raw pulses * gear ratio * decoding factor
// 11 * 45 * 2 = 990 using x2 decoding
//
// Robot navigation
// Encoders measure distance travelled.
// Turning accuracy comes from the MPU6050 gyro and not the encoders.
// Tank tracks slip during turns which makes encoder heading unreliable.

#pragma once

#include <math.h>
#include <stdint.h>

// Status codes

typedef enum {
    OK                =  0,
    ERR_BUS_FAIL      = -1,
    ERR_ADDRESS_FAIL  = -2,
    ERR_WRITE_FAIL    = -3,
    ERR_READ_FAIL     = -4,
    ERR_INVALID_ARG   = -5,
    ERR_INIT_REQUIRED = -6
} status_t;

// Mathematical constants

#define BOT_PI   3.14159265358979323846f
#define EPSILON  1e-6f

#define to_rad(x) ((x) * 0.017453292519943295f)
#define to_deg(x) ((x) * 57.29577951308232f)

// Hardware configuration

// Encoder

#define GEAR_RATIO   45.0f
#define ENC_PPR_RAW  11.0f  // Hall pulses per motor shaft revolution

// Decoding factor
// 1 = rising edge only
// 2 = both channels rising edges
// 4 = full quadrature

#define ENC_DECODE  2.0f

// Total counts per output shaft revolution
// counts = raw pulses * gear ratio * decoding factor

#define ENC_PPR (ENC_PPR_RAW * GEAR_RATIO * ENC_DECODE)

// Drive sprocket
// Measure the toothed gear the motor shaft drives, not the road wheels.

#define SPROCKET_DIA_MM  65.0f                          // CALIBRATE PLEASEEEE
#define SPROCKET_RAD_M   (SPROCKET_DIA_MM / 2000.0f)
#define SPROCKET_CIRC_M  (BOT_PI * (SPROCKET_DIA_MM / 1000.0f))

// Track spacing
// Center to center distance between left and right tracks.
// Measured from chassis diagram.

#define TRACK_M  0.10158f

// Track slip compensation
// Tracks slip against the ground during straight driving.
// To calibrate, command exactly one meter and measure actual distance.
//
// SLIP_FACTOR = actual distance / encoder reported distance
//
// Example: encoder reports 1.00 m, robot actually moved 0.92 m, set 0.92

#define SLIP_FACTOR  1.0f                               // CALIBRATE

// Maximum speed

#define MAX_RPM  150.0f
#define MAX_MPS  (MAX_RPM / 60.0f * SPROCKET_CIRC_M)

// Data types

typedef struct { float x; float y;               } vec2_t;
typedef struct { float x; float y; float heading; } pos2_t;
typedef struct { float linear;     float angular;  } vel_t;

// Encoder conversions

static inline float counts_to_revs(int32_t counts) {
    return (float)counts / ENC_PPR;
}

static inline float counts_to_m(int32_t counts) {
    return counts_to_revs(counts) * SPROCKET_CIRC_M * SLIP_FACTOR;
}

static inline float cps_to_rpm(float cps) {
    return (cps / ENC_PPR) * 60.0f;
}

static inline float rpm_to_mps(float rpm) {
    return (rpm / 60.0f) * SPROCKET_CIRC_M;
}

static inline float mps_to_rpm(float mps) {
    return (mps / SPROCKET_CIRC_M) * 60.0f;
}

// Angle normalization
// Keeps angles within negative pi to positive pi.

static inline float wrap_angle(float rad) {
    while (rad >  BOT_PI) rad -= 2.0f * BOT_PI;
    while (rad < -BOT_PI) rad += 2.0f * BOT_PI;
    return rad;
}

// Odometry
//
// dl = distance travelled by left track
// dr = distance travelled by right track
//
// Center distance  = (dl + dr) / 2
// Heading change   = (dr - dl) / track width
//
// Heading should be corrected from the gyro after every turn.

static inline void odom_update(pos2_t *pos, float dl, float dr) {
    float dist    = (dl + dr) * 0.5f;
    float dtheta  = (dr - dl) / TRACK_M;
    pos->heading += dtheta;
    pos->x       += dist * cosf(pos->heading);
    pos->y       += dist * sinf(pos->heading);
}

static inline void odom_set_heading(pos2_t *pos, float heading_rad) {
    pos->heading = wrap_angle(heading_rad);
}

static inline float dist2d(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1, dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

// Pivot turn geometry
//
// Both tracks travel equal arcs in opposite directions.
// arc length = pi * track width * angle / 360
//
// Use for reference only. Gyro controls actual turn behavior.

static inline float turn_arc_m(float deg) {
    return BOT_PI * TRACK_M * (deg < 0 ? -deg : deg) / 360.0f;
}

static inline int32_t turn_counts(float deg) {
    return (int32_t)((turn_arc_m(deg) / SPROCKET_CIRC_M) * ENC_PPR);
}

// Utility math

static inline float clamp(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static inline float remap(float x, float in_lo, float in_hi,
                          float out_lo, float out_hi) {
    return (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo) + out_lo;
}

static inline float lowpass(float prev, float input, float alpha) {
    return alpha * input + (1.0f - alpha) * prev;
}

static inline float deadband(float x, float threshold) {
    return fabsf(x) <= threshold ? 0.0f : x;
}

// Vector math

static inline float vec2_dot(vec2_t a, vec2_t b) {
    return a.x * b.x + a.y * b.y;
}

static inline float vec2_len(vec2_t v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

static inline vec2_t vec2_norm(vec2_t v) {
    float len = vec2_len(v);
    if (len < EPSILON) return (vec2_t){0.0f, 0.0f};
    return (vec2_t){v.x / len, v.y / len};
}

static inline void rotate_pt(float *x, float *y, float rad) {
    float nx = *x * cosf(rad) - *y * sinf(rad);
    float ny = *x * sinf(rad) + *y * cosf(rad);
    *x = nx; *y = ny;
}

// PID controller

typedef struct {
    float kp, ki, kd;
    float prev_err;
    float integral;
} bot_pid_t;

static inline void pid_init(bot_pid_t *pid, float kp, float ki, float kd) {
    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->prev_err = 0.0f;
    pid->integral = 0.0f;
}

static inline float pid_update(bot_pid_t *pid, float setpoint, float measured, float dt) {
    float err      = setpoint - measured;
    pid->integral += err * dt;
    float deriv    = (err - pid->prev_err) / dt;
    pid->prev_err  = err;
    return pid->kp * err + pid->ki * pid->integral + pid->kd * deriv;
}
