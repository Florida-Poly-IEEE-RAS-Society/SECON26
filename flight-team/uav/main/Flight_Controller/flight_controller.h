#pragma once

#include <stdbool.h>

enum Pid_Type {
    Roll, Pitch, Yaw, Z_Vel, X_Pos, Y_Pos, Z_Pos, Num_Pid_Types
};

enum Pid_Param_Type {
    PPT_P, PPT_I, PPT_D, PPT_Setpoint,  
};

bool flight_controller_init(void);
void flight_controller_set_run(bool should_run);
void emergency_stop(void);
void change_pos_by(float inches_x, float inches_y);
void reset_pos(float offset_inches_x, float offset_inches_y);
void reset_height(float offset_inches_z);
void change_height_by(float inches_z);
void return_to_last_height(void);
void rotate_by(float degrees);
void set_throttle(float power);
void set_thrust_control_mode(bool direct_throttle);
bool at_desired_position(void);
void set_pid(enum Pid_Type pid_idx, enum Pid_Param_Type param_idx, float value);
float get_pid(enum Pid_Type pid_idx, enum Pid_Param_Type param_idx);
bool save_gyro_calibration_data(void);
bool set_gyro_calibration_data(void);
bool save_pid_parameters(void);
bool set_pid_parameters(void);

float get_x_pos(void);
float get_y_pos(void);
float get_z_pos(void);
float get_x_vel(void);
float get_y_vel(void);
float get_z_vel(void);
