#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;

    // autotone
    float ti;
    float td;
    float ku;
    float tu;
    
    float setpoint;
    float A[3];
    float error[3];
    float output;
} PID;

void pid_init(PID *pid);
void pid_update(PID *pid, float input);
void pid_set_coefficients(PID* pid);
void pid_log_terms(PID *pid);
void pid_set_auto_tune(bool use_autotune);
void pid_set_dt(float dt_ms);
void pid_set_realtime_coefficients(bool use_realtime);
