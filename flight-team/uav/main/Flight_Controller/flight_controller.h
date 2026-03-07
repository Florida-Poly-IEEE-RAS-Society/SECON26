#pragma once

#include <stdbool.h>

bool flight_controller_init(void);
void change_pos_by(float inches_x, float inches_y);
void reset_pos(float offset_inches_x, float offset_inches_y);
void reset_height(float offset_inches_z);
void change_height_by(float inches_z);
void return_to_last_height(void);
void rotate_by(float degrees);
bool at_desired_position(void);
void emergency_stop(void);
