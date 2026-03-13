#pragma once

#include <math.h>
#include <stdint.h>

typedef enum {
  OK = 0,
  ERR_BUS_FAIL = -1,
  ERR_ADDRESS_FAIL = -2,
  ERR_WRITE_FAIL = -3,
  ERR_READ_FAIL = -4,
  ERR_INVALID_ARG = -5,
  ERR_INIT_REQUIRED = -6
} status_t;

#define BOT_PI 3.14159265358979323846f
#define EPSILON 1e-6f

#define to_rad(x) ((x) * 0.017453292519943295f)
#define to_deg(x) ((x) * 57.29577951308232f)

#define GEAR_RATIO 45.0f
#define ENC_PPR_RAW 11.0f
#define ENC_DECODE 2.0f
#define ENC_PPR (ENC_PPR_RAW * GEAR_RATIO * ENC_DECODE)

#define SPROCKET_DIA_MM 55.5f
#define SPROCKET_CIRC_IN (BOT_PI * (SPROCKET_DIA_MM / 25.4f))

#define TRACK_IN 3.999f
#define SLIP_FACTOR 1.0f

#define MAX_RPM 150.0f
#define MAX_IPS (MAX_RPM / 60.0f * SPROCKET_CIRC_IN)

typedef struct {
  float x;
  float y;
} vec2_t;
typedef struct {
  float x;
  float y;
  float heading;
} pos2_t;
typedef struct {
  float linear;
  float angular;
} vel_t;

static inline float counts_to_revs(int32_t counts) {
  return (float)counts / ENC_PPR;
}

static inline float counts_to_in(int32_t counts) {
  return counts_to_revs(counts) * SPROCKET_CIRC_IN * SLIP_FACTOR;
}

static inline float cps_to_rpm(float cps) { return (cps / ENC_PPR) * 60.0f; }

static inline float rpm_to_ips(float rpm) {
  return (rpm / 60.0f) * SPROCKET_CIRC_IN;
}

static inline float ips_to_rpm(float ips) {
  return (ips / SPROCKET_CIRC_IN) * 60.0f;
}

static inline float wrap_angle(float rad) {
  while (rad > BOT_PI)
    rad -= 2.0f * BOT_PI;
  while (rad < -BOT_PI)
    rad += 2.0f * BOT_PI;
  return rad;
}

static inline void odom_update(pos2_t *pos, float dl, float dr) {
  float dist = (dl + dr) * 0.5f;
  float dtheta = (dr - dl) / TRACK_IN;
  pos->heading += dtheta;
  pos->x += dist * cosf(pos->heading);
  pos->y += dist * sinf(pos->heading);
}

static inline void odom_set_heading(pos2_t *pos, float heading_rad) {
  pos->heading = wrap_angle(heading_rad);
}

static inline float dist2d(float x1, float y1, float x2, float y2) {
  float dx = x2 - x1, dy = y2 - y1;
  return sqrtf(dx * dx + dy * dy);
}

static inline float turn_arc_in(float deg) {
  return BOT_PI * TRACK_IN * (deg < 0 ? -deg : deg) / 360.0f;
}

static inline int32_t turn_counts(float deg) {
  return (int32_t)((turn_arc_in(deg) / SPROCKET_CIRC_IN) * ENC_PPR);
}

static inline float clamp(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

static inline float remap(float x, float in_lo, float in_hi, float out_lo,
                          float out_hi) {
  return (x - in_lo) * (out_hi - out_lo) / (in_hi - in_lo) + out_lo;
}

static inline float lowpass(float prev, float input, float alpha) {
  return alpha * input + (1.0f - alpha) * prev;
}

static inline float deadband(float x, float threshold) {
  return fabsf(x) <= threshold ? 0.0f : x;
}

static inline float vec2_dot(vec2_t a, vec2_t b) {
  return a.x * b.x + a.y * b.y;
}

static inline float vec2_len(vec2_t v) { return sqrtf(v.x * v.x + v.y * v.y); }

static inline vec2_t vec2_norm(vec2_t v) {
  float len = vec2_len(v);
  if (len < EPSILON)
    return (vec2_t){0.0f, 0.0f};
  return (vec2_t){v.x / len, v.y / len};
}

static inline void rotate_pt(float *x, float *y, float rad) {
  float nx = *x * cosf(rad) - *y * sinf(rad);
  float ny = *x * sinf(rad) + *y * cosf(rad);
  *x = nx;
  *y = ny;
}

typedef struct {
  float kp, ki, kd;
  float prev_err;
  float integral;
} bot_pid_t;

static inline void pid_init(bot_pid_t *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->prev_err = 0.0f;
  pid->integral = 0.0f;
}

static inline float pid_update(bot_pid_t *pid, float setpoint, float measured,
                               float dt) {
  float err = setpoint - measured;
  pid->integral += err * dt;
  float deriv = (err - pid->prev_err) / dt;
  pid->prev_err = err;
  return pid->kp * err + pid->ki * pid->integral + pid->kd * deriv;
}
