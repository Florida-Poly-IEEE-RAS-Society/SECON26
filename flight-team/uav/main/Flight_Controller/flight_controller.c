#include "flight_controller.h"

#include <freertos/FreeRTOS.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include <math.h>

#include <Motor/motor.h>
#include <driver/gpio.h>
#include <Gyro/bno055.h>
#include <stdint.h>


/* Nominal period for first iteration after arm and for vTaskDelay target (~100 Hz) */
#define NOMINAL_DT     (0.01f)
#define SLEEP_TIME_MS  (NOMINAL_DT * 1000.0f)
/* Clamp measured dt to avoid integral/derivative blowups on long stalls */
#define DT_MIN (0.001f)
#define DT_MAX (0.05f)

#define MIN_THROTTLE 0.06f
#define ACCEPTABLE_ERROR 0.1f

// arbitrary
#define GYRO_ID 55

#define METER_PER_INCH (0.0254f)
#define RAD_PER_DEG (3.141592654f / 180.0f)

#define NVS_BNO055_NAMESPACE "bno055"
#define NVS_BNO055_CAL_KEY   "cal_offsets"

#define NVS_PID_NAMESPACE "pid"
#define NVS_PID_PARAMS_KEY "pid_kp_ki_kd"
#define PID_STORAGE_SIZE (Num_Pid_Types * 3) /* kp, ki, kd per PID */

/* One-pole low-pass: filtered = GYRO_*_ALPHA * filtered_prev + (1 - GYRO_*_ALPHA) * raw */
#define GYRO_RATE_ALPHA   (0.9f)   /* angular rates: stronger smoothing, ~95 ms time constant at 100 Hz */
#define GYRO_ANGLE_ALPHA (0.8f)   /* roll/pitch angles: lighter smoothing */

static const char* TAG = "FC";

static inline float lowpass(float alpha, float prev, float raw) {
    return alpha * prev + (1.0f - alpha) * raw;
}

struct Pid {
    union {
        float params[4];
        struct {
            float kp;
            float ki;
            float kd;
            float setpoint;
        };
    };
    
    float integral;
    float prev_err;
    float integral_limit;
};

static bool _direct_throttle = false;

// meter/second
static float _x_vel = 0.0f;
static float _y_vel = 0.0f;
static float _z_vel = 0.0f;

// meter
static float _x_pos = 0.0f;
static float _y_pos = 0.0f;
static float _z_pos = 0.0f;

static float _throttle = 0.0f;
static bool _should_run = false;
static bool _should_really_not_run = false;

/* Low-pass filter state for gyro/orientation; re-inited when arming */
static float _filt_roll = 0.0f;
static float _filt_pitch = 0.0f;
static float _filt_roll_rate = 0.0f;
static float _filt_pitch_rate = 0.0f;
static float _filt_yaw_rate = 0.0f;
static bool _gyro_filter_initialized = false;

static motor_config _cfg[4] = {
    [0] = {.pin = GPIO_NUM_9,  .task_name="MFrontRightCCW"}, // front is side with usb port
    [1] = {.pin = GPIO_NUM_1,  .task_name="MFrontLeftCW"},
    [2] = {.pin = GPIO_NUM_7,  .task_name="MBackRightCW"},
    [3] = {.pin = GPIO_NUM_43, .task_name="MBackLeftCCW"},
};

static motor_handler *_mh[4] = {0};

static struct Pid _pid[Num_Pid_Types] = {
    [Roll] = (struct Pid){
        .kp = 0.2f,
        .ki = 0.08f,
        .kd = 0.04f,
        .integral_limit = 10,
    },
    [Pitch] = (struct Pid){
        .kp = 0.2f/*0.4*/,
        .ki = 0.08f/*0.3*/,
        .kd = 0.04f/*0.1*/,
        .integral_limit = 10,
    },
    [Yaw] = (struct Pid){
        .kp = 0.5,
        .ki = 0.09,
        .kd = 0.0003,
        .integral_limit = 10,
    },
    [Z_Vel] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.377,
        .kd = 0.377,
        .integral_limit = 10,
    },
    [X_Pos] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.0,
        .kd = -1.0,
        .integral_limit = 10,
    },
    [Y_Pos] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.0,
        .kd = -1.0,
        .integral_limit = 10,
    },
    [Z_Pos] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.0,
        .kd = -1.0,
        .integral_limit = 10,
    },
};

static inline void clamp(float *x, float y) {
    if (*x < -y) *x = -y;
    if (*x > y) *x = y;
}

static inline void clamp0(float *x, float y) {
    if (*x < 0.0f) *x = 0.0f;
    if (*x > y) *x = y;
}

static void pid_reset(struct Pid *p) {
    p->prev_err = 0.0f;
    p->setpoint = 0.0f;
    p->integral = 0.0f;
}

static float update_pid_angle(struct Pid *p, float angle, float rate, float dt) {
    float err = p->setpoint - angle;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += err * dt;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = rate;

    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static float update_pid_rate(struct Pid *p, float rate, float dt) {
    float err = p->setpoint - rate;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += err * dt;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = (err - p->prev_err) / dt;
    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static float update_pid_throttle(struct Pid *p, float rate, float dt) {
    float err = p->setpoint - rate;

    p->integral += err * dt;
    clamp(&p->integral, p->integral_limit);

    float derivative = (err - p->prev_err) / dt;
    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp0(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static void flight_task(void *data) {
    float roll_bias = 0.0f;
    float pitch_bias = 0.0f;
    float yaw_rate_bias = 0.0f;
    /* 0 = not yet armed this session; first armed iteration uses NOMINAL_DT */
    static int64_t prev_loop_us = 0;

    while (true) {
        if (!_should_run) {
            prev_loop_us = 0;
            _gyro_filter_initialized = false;
            set_motor_speed_pcnt(_mh[0], 0); // front right
            set_motor_speed_pcnt(_mh[1], 0); // front left
            set_motor_speed_pcnt(_mh[2], 0); // back right
            set_motor_speed_pcnt(_mh[3], 0); // back left

            for (int i = 0; i < Num_Pid_Types; ++i) pid_reset(&_pid[i]);

            _x_pos = 0.0f;
            _y_pos = 0.0f;
            _z_pos = 0.0f;

            _x_vel = 0.0f;
            _y_vel = 0.0f;
            _z_vel = 0.0f;

            _throttle = 0.0f;

            sensors_event_t orient_ev;
            sensors_event_t gyro_ev;
            bno055_getEvent2(&orient_ev, VECTOR_EULER);
            bno055_getEvent2(&gyro_ev, VECTOR_GYROSCOPE);

            float roll = orient_ev.orientation.z;
            float pitch = orient_ev.orientation.y;
            float yaw_rate = gyro_ev.gyro.heading;

            float prev_roll;
            float prev_pitch;
            float prev_yaw_rate;
            int i = 0;

            do {
                vTaskDelay(250 / portTICK_PERIOD_MS);

                bno055_getEvent2(&orient_ev, VECTOR_EULER);
                bno055_getEvent2(&gyro_ev, VECTOR_GYROSCOPE);
                prev_roll = roll;
                prev_pitch = pitch;
                prev_yaw_rate = yaw_rate;
                roll = orient_ev.orientation.z;
                pitch = orient_ev.orientation.y;
                yaw_rate = gyro_ev.gyro.heading;
                if (fabsf(prev_roll - roll) < 0.01f &&
                    fabsf(prev_pitch - pitch) < 0.01f && fabsf(prev_yaw_rate - yaw_rate) < 0.01f) {
                    if (i == 4) {
                        ESP_LOGI(TAG, "Set Roll, Pitch, and Yaw Rate Bias to: (%f, %f, %f)", roll, pitch, yaw_rate);
                        roll_bias = roll;
                        pitch_bias = pitch;
                        yaw_rate_bias = yaw_rate;
                        _pid[Yaw].integral = 0;
                        i = 0;
                    }
                    i += 1;
                }
            } while (!_should_run || _should_really_not_run);
        }

        /* True delta since last loop start (includes vTaskDelay jitter) */
        int64_t now_us = esp_timer_get_time();
        float dt;
        if (prev_loop_us == 0) {
            dt = NOMINAL_DT;
        } else {
            dt = (float)(now_us - prev_loop_us) / 1000000.0f;
            if (dt < DT_MIN) {
                dt = DT_MIN;
            }
            if (dt > DT_MAX) {
                dt = DT_MAX;
            }
        }
        prev_loop_us = now_us;

        /*
          On the Chip: "BNO055" is the front
          x: forward-backward; roll (around this axis)
          y: left-right; pitch (around this axis)
          z: up-down; yaw (around this axis)
         */
        sensors_event_t orient_ev;
        sensors_event_t gyro_ev;
        sensors_event_t accel_ev;
        bno055_getEvent2(&orient_ev, VECTOR_EULER); // degree
        bno055_getEvent2(&gyro_ev, VECTOR_GYROSCOPE); // radians/second
        bno055_getEvent2(&accel_ev, VECTOR_LINEARACCEL); // m/s^2 (acceleration - gravity)

        float roll = (orient_ev.orientation.z - roll_bias) * RAD_PER_DEG;
        float pitch = (orient_ev.orientation.y - pitch_bias) * RAD_PER_DEG;
        float yaw = orient_ev.orientation.x * RAD_PER_DEG;

        float roll_rate_raw = gyro_ev.gyro.roll;
        float pitch_rate_raw = gyro_ev.gyro.pitch;
        float yaw_rate_raw = gyro_ev.gyro.heading - yaw_rate_bias;

        if (!_gyro_filter_initialized) {
            _filt_roll = roll;
            _filt_pitch = pitch;
            _filt_roll_rate = roll_rate_raw;
            _filt_pitch_rate = pitch_rate_raw;
            _filt_yaw_rate = yaw_rate_raw;
            _gyro_filter_initialized = true;
        } else {
            _filt_roll = lowpass(GYRO_ANGLE_ALPHA, _filt_roll, roll);
            _filt_pitch = lowpass(GYRO_ANGLE_ALPHA, _filt_pitch, pitch);
            _filt_roll_rate = lowpass(GYRO_RATE_ALPHA, _filt_roll_rate, roll_rate_raw);
            _filt_pitch_rate = lowpass(GYRO_RATE_ALPHA, _filt_pitch_rate, pitch_rate_raw);
            _filt_yaw_rate = lowpass(GYRO_RATE_ALPHA, _filt_yaw_rate, yaw_rate_raw);
        }
        float roll_rate = _filt_roll_rate;
        float pitch_rate = _filt_pitch_rate;
        float yaw_rate = _filt_yaw_rate;
        roll = _filt_roll;
        pitch = _filt_pitch;

        float x_acc = accel_ev.acceleration.x;
        float y_acc = accel_ev.acceleration.y;
        float z_acc = accel_ev.acceleration.z;

        _x_pos += _x_vel * dt + x_acc * dt * dt / 2.0f;
        _y_pos += _y_vel * dt + y_acc * dt * dt / 2.0f;
        _z_pos += _z_vel * dt + z_acc * dt * dt / 2.0f;

        _x_vel += x_acc * dt;
        _y_vel += y_acc * dt;
        _z_vel += z_acc * dt;

        float t = _throttle;
        if (!_direct_throttle) {
            _pid[Pitch].setpoint = update_pid_rate(&_pid[X_Pos], _x_pos, dt);
            _pid[Roll].setpoint = update_pid_rate(&_pid[Y_Pos], _y_pos, dt);
            _pid[Z_Vel].setpoint = update_pid_rate(&_pid[Z_Pos], _z_pos, dt); // maybe just have this control the throttle directly
            /* find some good values */
            clamp(&_pid[Pitch].setpoint, 0.3);
            clamp(&_pid[Roll].setpoint, 0.3);
            clamp(&_pid[Z_Vel].setpoint, 0.03);

            t = update_pid_throttle(&_pid[Z_Vel], _z_vel, dt);
            _throttle = t;
        }

        float r = update_pid_angle(&_pid[Roll], roll, roll_rate, dt);
        float p = update_pid_angle(&_pid[Pitch], pitch, pitch_rate, dt);
        float y = update_pid_rate(&_pid[Yaw], yaw_rate, dt);

        float m1 = t - r + p - y;
        float m2 = t - r - p + y;
        float m3 = t + r + p + y;
        float m4 = t + r - p - y;
        /* ESP_LOGI(TAG, "T: %f, r: %f, p: %f, y: %f, M1: %f, M2: %f, M3: %f, M4: %f, roll: %f, pitch: %f, yaw: %f, roll_rate: %f, pitch_rate: %f, yaw_rate %f, acc_x: %f, acc_y: %f, acc_z: %f\n", t, r, p, y, m1, m2, m3, m4, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, x_acc, y_acc, z_acc); */
        set_motor_speed_pcnt(_mh[0], m1);
        set_motor_speed_pcnt(_mh[1], m2);
        set_motor_speed_pcnt(_mh[2], m3);
        set_motor_speed_pcnt(_mh[3], m4);

        vTaskDelay((TickType_t)(SLEEP_TIME_MS / portTICK_PERIOD_MS));    
    }
}

void reset_height(float offset_inches_z) {
    _z_pos = offset_inches_z * METER_PER_INCH;
    _pid[Z_Pos].setpoint = _z_pos;
}

void reset_pos(float offset_inches_x, float offset_inches_y) {
    _x_pos = offset_inches_x * METER_PER_INCH;
    _y_pos = offset_inches_y * METER_PER_INCH;
    _pid[X_Pos].setpoint = _x_pos;
    _pid[Y_Pos].setpoint = _y_pos;
}

void change_height_by(float inches_z) {
    _pid[Z_Pos].setpoint += inches_z * METER_PER_INCH;
}

void return_to_last_height(void) {
    _pid[Z_Pos].setpoint = 0.0f;
}

void change_pos_by(float inches_x, float inches_y) {
    _pid[X_Pos].setpoint += inches_x * METER_PER_INCH;
    _pid[Y_Pos].setpoint += inches_y * METER_PER_INCH;
}

void rotate_by(float degrees) {
    _pid[Yaw].setpoint += degrees * RAD_PER_DEG;
}

bool at_desired_position(void) {
    float e1 = fabsf(_pid[X_Pos].prev_err) < ACCEPTABLE_ERROR;
    float e2 = fabsf(_pid[Y_Pos].prev_err) < ACCEPTABLE_ERROR;
    float e3 = fabsf(_pid[Z_Pos].prev_err) < ACCEPTABLE_ERROR;
    float e4 = fabsf(_pid[Yaw].prev_err) < ACCEPTABLE_ERROR;

    return e1 && e2 && e3 && e4;
}

void flight_controller_set_run(bool should_run) {
    _should_run = should_run;
}

void emergency_stop(void) {
    _should_really_not_run = true;
}

bool is_flight_controller_calibrated(void) {
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    bno055_getCalibration(&system, &gyro, &accel, &mag);
    return (gyro == 3) && (accel == 3) && (mag == 3);
}

void set_throttle(float power) {
    if (power < 0.0f) {
        flight_controller_set_run(false);
        _direct_throttle = false;
    } else {
        clamp0(&power, 1);
        _throttle = power;
        _direct_throttle = true;
        flight_controller_set_run(true);
    }
}

void set_thrust_control_mode(bool direct_throttle) {
    _direct_throttle = direct_throttle;
}

bool set_pid(enum Pid_Type pid_idx, enum Pid_Param_Type param_idx, float value) {
  if (pid_idx < 0 || pid_idx >= Num_Pid_Types) return false;
  if (param_idx < 0 || param_idx >= 4) return false;
  _pid[pid_idx].params[param_idx] = value;
  return true;
}

float get_pid(enum Pid_Type pid_idx, enum Pid_Param_Type param_idx) {
    if (pid_idx < 0 || pid_idx >= Num_Pid_Types) return NAN;
    if (param_idx < 0 || param_idx >= 4) return NAN;
    return _pid[pid_idx].params[param_idx];
}

float get_x_pos(void) { return _x_pos; }
float get_y_pos(void) { return _y_pos; }
float get_z_pos(void) { return _z_pos; }
float get_x_vel(void) { return _x_vel; }
float get_y_vel(void) { return _y_vel; }
float get_z_vel(void) { return _z_vel; }

bool save_gyro_calibration_data(void) {
    bno055_offsets_t offsets;
    if (!bno055_getSensorOffsets2(&offsets)) {
        ESP_LOGE(TAG, "Gyro not calibrated; cannot save data");
        return false; /* sensor not fully calibrated, nothing to save */
    }
    nvs_handle_t handle;
    if (nvs_open(NVS_BNO055_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Could not open Gyro calibration namespace");
        return false;
    }
    esp_err_t err = nvs_set_blob(handle, NVS_BNO055_CAL_KEY, &offsets, sizeof(offsets));
    if (err == ESP_OK) {
        nvs_commit(handle);
        ESP_LOGI(TAG, "Gyro calibration data saved sucessfully");
    } else {
        ESP_LOGE(TAG, "Could not save Gyro calibration data");
    }
    nvs_close(handle);

    return true; // yeah yeah it returns true regardless
}

bool set_gyro_calibration_data(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_BNO055_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "No Gyro calibration data available");
        return false; /* no saved calibration or NVS not available */
    }
    bno055_offsets_t offsets;
    size_t len = sizeof(offsets);
    if (nvs_get_blob(handle, NVS_BNO055_CAL_KEY, &offsets, &len) == ESP_OK && len == sizeof(offsets)) {
        bno055_setSensorOffsets4(&offsets);
        ESP_LOGI(TAG, "Gyro calibration loaded sucessfully");
    } else {
        ESP_LOGE(TAG, "Could load gyro calibration");
    }
    nvs_close(handle);

    return true; // yeah yeah it returns true regardless
}

bool save_pid_parameters(void) {
    float buf[PID_STORAGE_SIZE];
    for (int i = 0; i < Num_Pid_Types; i++) {
        buf[i * 3 + 0] = _pid[i].kp;
        buf[i * 3 + 1] = _pid[i].ki;
        buf[i * 3 + 2] = _pid[i].kd;
    }
    nvs_handle_t handle;
    if (nvs_open(NVS_PID_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGI(TAG, "Could not open PID namespace");
        return false;
    }
    esp_err_t err = nvs_set_blob(handle, NVS_PID_PARAMS_KEY, buf, sizeof(buf));
    if (err == ESP_OK) {
        nvs_commit(handle);
        ESP_LOGI(TAG, "PID values saved successfully");
    } else {
        ESP_LOGI(TAG, "Could not save PID values");
    }
    nvs_close(handle);

    return true; // yeah yeah it returns true regardless of 'err'
}

bool set_pid_parameters(void) {
    nvs_handle_t handle;
    if (nvs_open(NVS_PID_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGE(TAG, "Could not load PID values");
        return false;
    }
    float buf[PID_STORAGE_SIZE];
    size_t len = sizeof(buf);
    if (nvs_get_blob(handle, NVS_PID_PARAMS_KEY, buf, &len) != ESP_OK || len != sizeof(buf)) {
        nvs_close(handle);
        return false;
    }
    for (int i = 0; i < Num_Pid_Types; i++) {
        _pid[i].kp = buf[i * 3 + 0];
        _pid[i].ki = buf[i * 3 + 1];
        _pid[i].kd = buf[i * 3 + 2];
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "PID values loaded successfully");
    return true;
}

bool flight_controller_init(void) {
    for (int i = 0; i < 4; ++i) {
        gpio_reset_pin(_cfg[i].pin);
        gpio_set_direction(_cfg[i].pin, GPIO_MODE_OUTPUT);
    }

    for (int i = 0; i < 4; ++i) {
        _mh[i] = init_motor(&_cfg[i]);
        set_motor_speed(_mh[i], 0);
    }

    bool ok = bno055_begin(GYRO_ID, OPERATION_MODE_NDOF, BNO055_ADDRESS_A);
    if (!ok) {
        printf("Gyro not found");
        return false;
    }
    
    bno055_setExtCrystalUse(true);

    set_gyro_calibration_data();
    
    /* while (true) { */
    /*     sensors_event_t orient_ev; */
    /*     sensors_event_t gyro_ev; */
    /*     sensors_event_t accel_ev; */
    /*     bno055_getEvent2(&orient_ev, VECTOR_EULER); // degree */
    /*     bno055_getEvent2(&gyro_ev, VECTOR_GYROSCOPE); // radians/second */
    /*     bno055_getEvent2(&accel_ev, VECTOR_LINEARACCEL); // m/s^2 (acceleration - gravity) */
        
    /*     printf("Gyro: roll: %f, pitch: %f, yaw: %f\n", orient_ev.orientation.z, orient_ev.orientation.y, orient_ev.orientation.x); */
    /*     printf("Gyro: roll_rate: %f, pitch_rate: %f, yaw_rate: %f\n", gyro_ev.gyro.roll, gyro_ev.gyro.pitch, gyro_ev.gyro.heading); */
    /*     printf("Gyro: acc_x: %f, acc_y: %f, acc_z: %f\n", accel_ev.acceleration.x, accel_ev.acceleration.y, accel_ev.acceleration.z); */

    /*     printf("\n"); */
    /*     vTaskDelay(125 / portTICK_PERIOD_MS); */
    /* } */


    set_pid_parameters();

    xTaskCreate(flight_task, "flight_controller", 4096, NULL, configMAX_PRIORITIES-1, NULL);

    return true;
}
