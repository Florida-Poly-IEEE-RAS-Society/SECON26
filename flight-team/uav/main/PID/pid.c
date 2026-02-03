#include "pid.h"

// Private
#define DEFAULT_KP 0.0f
#define DEFAULT_KI 0.0f
#define DEFAULT_KD 0.0f
#define DEFAULT_DT 0.01f

static float _dt;
static bool _realtime_coefficients;
static bool _autotune;

// https://en.wikipedia.org/wiki/Ziegler%E2%80%93Nichols_method
// https://www.mstarlabs.com/control/znrule.html
// https://www.mstarlabs.com/control/selftest.html
void pid_autotune_step(PID *p, float input) {
    /* 1. The ratio of output level to input level at low frequencies determines the gain parameter K of the model.
       2. Observe the frequency Fu at which the phase passes through -pi radians (-180 degrees). The inverse of this frequency is the period of the oscillation, Tu.
       3. Observe the plant gain Kc that occurs at the critical oscillation frequency Fu. The inverse of this is the gain margin Ku.
     */
    p->ku = 0.0f; 
    p->tu = 0.0f;
    
    p->kp = (1.0f/3.0f) * p->ku;
    p->ti = 0.5f * p->tu;
    p->td = (1.0f/3.0f) * p->tu;
    
    p->ki = p->kp / p->ti;
    p->kd = p->kp * p->td;
}
// End Private

// Public
void pid_init(PID *pid) {
    *pid = (PID){
        .kp = DEFAULT_KP,
        .ki = DEFAULT_KI,
        .kd = DEFAULT_KD,
    };
    _dt = DEFAULT_DT;
    _realtime_coefficients = false;
}

// https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller#Pseudocode
void pid_update(PID *pid, float input) {
    if (_autotune) pid_autotune_step(pid, input);
    else if (_realtime_coefficients) pid_set_coefficients(pid);
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->error[0] = pid->setpoint - input;
    pid->output += pid->A[0] * pid->error[0] + pid->A[1] * pid->error[1] + pid->A[2] * pid->error[2];
}

bool pid_set_coefficients(PID* pid) {
    pid->A[0] = pid->kp + pid->ki * _dt + pid->kd / _dt;
    pid->A[1] = -pid->kp - 2 * pid.kd / _dt;
    pid->A[2] = pid.kd / _dt;
}


void pid_log_terms(PID *pid) {
    // todo
}

void pid_auto_tune(bool use_autotune) {
    _autotune = use_autotune;
}

void pid_set_dt(float dt_ms) {
    _dt = dt_ms;
}

void pid_set_realtime_coefficients(bool use_realtime) {
    _realtime_coefficients = use_realtime;
}
// End Public
