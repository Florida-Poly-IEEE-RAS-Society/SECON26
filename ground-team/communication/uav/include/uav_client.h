#include <stdint.h>

enum Command {
    IMAGE = 1,
    LAUNCH,
    RETRIEVE,
    TRANSMISSION_CODES,
    POS,
    STOP,
    THRUST,
    THRUST_CTRL_MODE,
    PITCH,
    ROLL,
    YAW,
    SET_HEIGHT,
    SET_X,
    SET_Y,
    SET_PID,
    GET_PID,
    SAVE_PID,
    GYRO_CALIBRATION_STATUS,
    GET_GAME_STATE,
    POS_VEL,
};

void* send_command_to_uav(enum Command cmd, void* args);

#define NUM_TRANSMISSION_CODES 4

// copied from uav IR code
typedef struct {
    uint16_t address;
    uint16_t command;
} ir_nec_scan_code_t;

typedef struct {
    ir_nec_scan_code_t codes[NUM_TRANSMISSION_CODES];
} transmission_codes_args;

// --- POS ---
// header: 4 floats (position data)
typedef struct {
    float a;
    float b;
    float c;
    float d;
} pos_args; /* semantic names: e.g. uav_x, uav_y, bot_x, bot_y in higher-level code */

// --- STOP ---
// header: none
// Response: none

// --- THRUST ---
// header: 1 float (throttle power)
typedef struct {
    float thrust;
} thrust_args;

// --- THRUST_CTRL_MODE ---
// header: 1 uint8_t (thrust control mode)
typedef struct {
    uint8_t mode;
} thrust_ctrl_mode_args;

// --- PITCH ---
// header: 1 float (pitch)
typedef struct {
    float pitch;
} pitch_args;

// --- ROLL ---
typedef struct {
    float roll;
} roll_args;

// --- YAW ---
typedef struct {
    float yaw;
} yaw_args;

// --- SET_HEIGHT ---
typedef struct {
    float delta_height;
} set_height_args;

// --- SET_X ---
typedef struct {
    float delta_x;
} set_x_args;

// --- SET_Y ---
typedef struct {
    float delta_y;
} set_y_args;

// --- SET_PID ---
// header: struct { uint8_t pid_idx; uint8_t param_idx; float value; }
typedef struct {
    uint8_t pid_idx;
    uint8_t param_idx;
    float value;
} set_pid_args;

// --- GET_PID ---
// header: struct { uint8_t pid_idx; uint8_t param_idx; }
// response: 1 float (current PID parameter value)
typedef struct {
    uint8_t pid_idx;
    uint8_t param_idx;
} get_pid_args;


// gonna see how temporary stack allocation goes, if it creates problems then
// go back 2 heap
typedef struct {
    uint32_t size;
    uint8_t data[];
} image_response;

typedef struct {
    float value;
} get_pid_response;

typedef struct {
    uint8_t success;
} save_pid_response;

typedef struct {
    uint8_t system;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} gyro_calibration_status_response;

typedef struct {
    uint8_t state;
} get_game_state_response;

typedef struct {
    float x_pos;
    float y_pos;
    float z_pos;
    float x_vel;
    float y_vel;
    float z_vel;
} pos_vel_response;
