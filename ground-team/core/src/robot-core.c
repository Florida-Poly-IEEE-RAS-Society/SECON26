#include "../include/robot-core.h"

// ========================== MATH =============================
static inline float ras_distance_2d(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

static inline float ras_vec2_dot(ras_vec2_t a, ras_vec2_t b) {
    return a.x * b.x + a.y * b.y;
}

static inline float ras_vec2_length(ras_vec2_t v) {
    return sqrtf(v.x * v.x + v.y * v.y);
}

static inline ras_vec2_t ras_vec2_normalize(ras_vec2_t v) {
    float len = ras_vec2_length(v);
    if (len < RAS_EPSILON) return (ras_vec2_t){0.0f, 0.0f};
    return (ras_vec2_t){v.x / len, v.y / len};
}

static inline void ras_rotate_point(float *x, float *y, float angle_rad) {
    float x_new = *x * cosf(angle_rad) - *y * sinf(angle_rad);
    float y_new = *x * sinf(angle_rad) + *y * cosf(angle_rad);
    *x = x_new;
    *y = y_new;
}

static inline float ras_angle_wrap(float angle_rad) {
    while (angle_rad > RAS_PI) angle_rad -= 2.0f * RAS_PI;
    while (angle_rad < -RAS_PI) angle_rad += 2.0f * RAS_PI;
    return angle_rad;
}

// ========================== UTIL FUNCS =========================

static inline float ras_clamp(float x, float min_val, float max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static inline float ras_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float ras_lowpass(float prev_output, float input, float alpha) {
    return alpha * input + (1.0f - alpha) * prev_output;
}

// ========================== PID CONTROLLER =========================

typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integral;
} ras_pid_t;

static inline void ras_pid_init(ras_pid_t *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}

static inline float ras_pid_compute(ras_pid_t *pid, float setpoint, float measured, float dt) {
    float error = setpoint - measured;
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
