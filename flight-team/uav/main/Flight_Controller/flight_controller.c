#include "flight_controller.h"

#include <freertos/FreeRTOS.h>
#include <math.h>

#include <Motor/motor.h>
#include <Gyro/bno055.h>

#define DT (0.01f)
#define SLEEP_TIME (DT*1000.0f)

#define MIN_THROTTLE 0.06f
#define ACCEPTABLE_ERROR 0.1f

// arbitrary
#define GYRO_ID 55

#define METER_PER_INCH (0.0254f)
#define RAD_PER_DEG (3.141592654f / 180.0f)

struct Pid {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_err;

    float setpoint;
    float integral_limit;
};

enum Pid_Type {
    Roll, Pitch, Yaw, Z_Vel, X_Pos, Y_Pos, Z_Pos, Num_Pid_Types
};

// meter/second
static float _x_vel = 0.0f;
static float _y_vel = 0.0f;
static float _z_vel = 0.0f;

// meter
static float _x_pos = 0.0f;
static float _y_pos = 0.0f;
static float _z_pos = 0.0f;

static float _throttle = 0.0f;
static bool _should_run = true;

static motor_config _cfg[4] = {
    [0] = {.pin = 43, .task_name="M1"},
    [1] = {.pin =  8, .task_name="M2"},
    [2] = {.pin =  1, .task_name="M3"},
    [3] = {.pin =  9, .task_name="M4"},
};

static motor_handler *_mh[4] = {0};

static struct Pid _pid[Num_Pid_Types] = {
    [Roll] = (struct Pid){
        .kp = 0.4,
        .ki = 0.3,
        .kd = 0.1,
        .integral_limit = 25,
    },
    [Pitch] = (struct Pid){
        .kp = 0.4,
        .ki = 0.3,
        .kd = 0.1,
        .integral_limit = 25,
    },
    [Yaw] = (struct Pid){
        .kp = 0.7,
        .ki = 0.09,
        .kd = 0.0003,
        .integral_limit = 25,
    },
    [Z_Vel] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.377,
        .kd = 0.377,
        .integral_limit = 25,
    },
    [X_Pos] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.0,
        .kd = -1.0,
        .integral_limit = 25,
    },
    [Y_Pos] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.0,
        .kd = -1.0,
        .integral_limit = 25,
    },
    [Z_Pos] = (struct Pid) {
        .kp = 1.0,
        .ki = 0.0,
        .kd = -1.0,
        .integral_limit = 25,
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
}

static float update_pid_angle(struct Pid *p, float angle, float rate) {
    float err = p->setpoint - angle;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += err * DT;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = rate;

    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static float update_pid_rate(struct Pid *p, float rate) {
    float err = p->setpoint - rate;

    if (_throttle <= MIN_THROTTLE) {
        p->integral = 0.0f;
    } else {
        p->integral += err * DT;
        clamp(&p->integral, p->integral_limit);
    }

    float derivative = (err - p->prev_err) / DT;
    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static float update_pid_throttle(struct Pid *p, float rate) {
    float err = p->setpoint - rate;

    p->integral += err * DT;
    clamp(&p->integral, p->integral_limit);

    float derivative = (err - p->prev_err) / DT;
    float output = p->kp*err + p->ki*p->integral - p->kd*derivative;
    clamp0(&output, 1.0f);

    p->prev_err = err;

    return output;
}

static void flight_task(void* data) {
    while (true) {
        if (!_should_run) {
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

            do {
                vTaskDelay(500 / portTICK_PERIOD_MS);
          } while (!_should_run);
        }

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
        
        float roll = orient_ev.orientation.roll * RAD_PER_DEG;
        float pitch = orient_ev.orientation.pitch * RAD_PER_DEG;

        float roll_rate = gyro_ev.gyro.roll;
        float pitch_rate = gyro_ev.gyro.pitch;
        float yaw_rate = gyro_ev.gyro.heading;

        float x_acc = accel_ev.acceleration.x;
        float y_acc = accel_ev.acceleration.y;
        float z_acc = accel_ev.acceleration.z;

        _x_pos += _x_vel*DT + x_acc*DT*DT/2.0f;
        _y_pos += _y_vel*DT + y_acc*DT*DT/2.0f;
        _z_pos += _z_vel*DT + z_acc*DT*DT/2.0f;

        _x_vel += x_acc*DT;
        _y_vel += y_acc*DT;
        _z_vel += z_acc*DT;

        _pid[Pitch].setpoint = update_pid_rate(&_pid[X_Pos], _x_pos);
        _pid[Roll].setpoint = update_pid_rate(&_pid[Y_Pos], _y_pos);
        _pid[Z_Vel].setpoint = update_pid_rate(&_pid[Z_Pos], _z_pos); // maybe just have this control the throttle directly
        // find some good values
        clamp(&_pid[Pitch].setpoint, 0.3);
        clamp(&_pid[Roll].setpoint, 0.3);
        clamp(&_pid[Z_Vel].setpoint, 0.03);

        float t = update_pid_throttle(&_pid[Z_Vel], _z_vel); _throttle = t;
        float r = update_pid_angle(&_pid[Roll], roll, roll_rate);
        float p = update_pid_angle(&_pid[Pitch], pitch, pitch_rate);
        float y = update_pid_rate(&_pid[Yaw], yaw_rate);

        // ensure y signs are correct
        set_motor_speed_pcnt(_mh[0], t + p + r + y); // front right
        set_motor_speed_pcnt(_mh[1], t + p - r - y); // front left
        set_motor_speed_pcnt(_mh[2], t - p + r + y); // back right
        set_motor_speed_pcnt(_mh[3], t - p - r - y); // back left

        vTaskDelay(SLEEP_TIME / portTICK_PERIOD_MS);    
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

void emergency_stop(void) {
    _should_run = false;
}

bool flight_controller_init(void) {
    bool ok = bno055_begin(GYRO_ID, OPERATION_MODE_NDOF, BNO055_ADDRESS_A);
    if (!ok) {
        printf("Gyro not found");
        return false;
    }

    bno055_setExtCrystalUse(true);

    for (int i = 0; i < 4; ++i) {
        gpio_reset_pin(_cfg[i].pin);
        gpio_set_direction(_cfg[i].pin, GPIO_MODE_OUTPUT);
        _mh[i] = init_motor(&_cfg[i]);
    }

    xTaskCreate(flight_task, "flight_controller", 4096, NULL, configMAX_PRIORITIES-1, NULL);

    return true;
}
