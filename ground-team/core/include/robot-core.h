// JOSHUA GOTTUS      @jr-cho
//
// @file robot-core.h
// @brief RAS Core Library - Central Header for SECON2026 Hardware Competition
//
// RAS Core Library for South East Con 2026. I will refer to South East Con 2026 as SECON2026 from here on out!
// This Library consist of common functions(etc. common robotics math func and utils func) and status codes.
//
// Include this header in any robot module to access the core features.

#pragma once

// ========================== STATUS =============================
// STATUS  0: OPERATION SUCCESSFUL
// STATUS -1: I2C/SPI BUS OPENING FAILURE || PERIPHERAL NOT FOUND
// STATUS -2: FAILED TO SET SLAVE ADDRESS OR CONFIGURE A DEVICE
// STATUS -3: FAILED TO WRITE DATA
// STATUS -4: FAILED TO READ DATA
// STATUS -5: AN ARGUMENT (like a value or id) IS OUT OF RANAGE
// STATUS -6: FUNCTION WAS CALLED BEFORE INITIALIZATION
// ===============================================================

typedef enum {
    OK = 0,
    ERR_BUS_FAIL = -1,  
    ERR_ADDRESS_FAIL = -2,
    ERR_WRITE_FAIL = -3,
    ERR_READ_FAIL = -4,
    ERR_INVALID_ARG = -5,
    ERR_INIT_REQUIRED = -6
} status_t;

static inline const char* status_to_str(status_t status);

// ========================== CONST =============================

#define RAS_PI 3.14159265358979323846f
#define DEG_TO_RAD(x) ((x) * 0.017453292519943295f)
#define RAD_TO_DEG(x) ((x) * 57.29577951308232f)
#define RAS_EPSILON 1e-6f

// ========================== DATA TYPES =========================


typedef struct {
    float x;
    float y;
} 2d_vec_t;

typedef struct {
    float x;
    float y;
    float theta; // THIS IS YOUR RADIANS
} 2d_pos_t;

typedef struct {
    float linear; // M/S
    float angular // RAD/S
} velocity_t;


// ========================== MATH =============================

static inline float ras_distance_2d(float x1, float y1, float x2, float y2);
static inline float ras_vec2_dot(ras_vec2_t a, ras_vec2_t b);
static inline float ras_vec2_length(ras_vec2_t v);
static inline ras_vec2_t ras_vec2_normalize(ras_vec2_t v);
static inline void ras_rotate_point(float *x, float *y, float angle_rad);
static inline float ras_angle_wrap(float angle_rad);

// ========================== UTIL FUNCS =========================

static inline float ras_clamp(float x, float min_val, float max_val);
static inline float ras_map(float x, float in_min, float in_max, float out_min, float out_max);
static inline float ras_lowpass(float prev_output, float input, float alpha);

// ========================== PID CONTROLLER =========================

typedef struct {
    float kp, ki, kd;
    float prev_error;
    float integral;
} ras_pid_t;

static inline void ras_pid_init(ras_pid_t *pid, float kp, float ki, float kd);
static inline float ras_pid_compute(ras_pid_t *pid, float setpoint, float measured, float dt);
