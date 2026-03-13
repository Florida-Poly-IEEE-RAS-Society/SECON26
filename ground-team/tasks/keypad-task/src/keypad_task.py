#!/usr/bin/env python3
"""
Keypad servo controller using pigpio on Raspberry Pi.
Controls three servos, each assigned to a single keypad button.
Usage: python main.py [sequence]
Example: python main.py 123
"""


################## SERVO TABLE ##################
# SERVO 1 - LINEAR SERVO
# SERVO 2 - LINEAR SERVO
# SERVO 3 - LINEAR SERVO
# SERVO 4 - ROTATIONAL SERVO PIVOTING SERVO 3
#################################################

################## COMMAND SEQUENCE ##################
# SERVO 4 -> ROTATE ~6 DEGREES TO HIT KEY 3
# SERVO 1 -> FORWARD PRESS INTO KEY 7
# SERVO 3 -> FORWARD PRESS INTO KEY 3
# SERVO 1 -> FORWARD PRESS INTO KEY 7
# SERVO 3 -> FORWARD PRESS INTO KEY 3
# SERVO 2 -> FORWARD PRESS INTO KEY 8
# SERVO 4 -> ROTATE TO REST PULSE TO AIM FOR KEY '#'
# SERVO 3 -> FORWARD PRESS INTO KEY '#'
######################################################


import sys
import time
import pigpio

# Class set to mimic status_t under robot-core.h
class status_t:
    OK                =  0
    ERR_BUS_FAIL      = -1
    ERR_ADDRESS_FAIL  = -2
    ERR_WRITE_FAIL    = -3
    ERR_READ_FAIL     = -4
    ERR_INVALID_ARG   = -5
    ERR_INIT_REQUIRED = -6

SERVO1_PIN = 27
SERVO2_PIN = 17
SERVO3_PIN = 4
SERVO4_PIN = 19

# Adjust these pulse widths (in microseconds) after calibration
# Typical range is ~500–2500 µs; 1500 µs is usually "center".

# Servo 1 -> Pressing 7
SERVO1_REST_PULSE = 1500
SERVO1_PRESS_PULSE = 1000

# Servo 2 -> Pressing 8
SERVO2_REST_PULSE = 1500
SERVO2_PRESS_PULSE = 1000

# Servo 3 -> Pressing 3 and #
SERVO3_REST_PULSE = 1500
SERVO3_PRESS_PULSE = 1000

# Rotation Servo for Servo 3 -> Rotating 6 degrees to hit
SERVO4_ROTATE_PULSE = 1540
SERVO4_REST_PULSE = 1500

# How long to hold the button down and pause between presses (seconds)
PRESS_HOLD_TIME = 0.3
BETWEEN_KEYS_TIME = 0.2

RAW_TASK_SEQUENCE = "73738#"
FULL_TASK_SEQUENCE = "^73738v#"

LINEAR_SERVOS = [
    {"key": "7", "pin": SERVO1_PIN, "rest": SERVO1_REST_PULSE, "press": SERVO1_PRESS_PULSE},
    {"key": "8", "pin": SERVO2_PIN, "rest": SERVO2_REST_PULSE, "press": SERVO2_PRESS_PULSE},
    {"key": "3", "pin": SERVO3_PIN, "rest": SERVO3_REST_PULSE, "press": SERVO3_PRESS_PULSE},
    {"key": "#", "pin": SERVO3_PIN, "rest": SERVO3_REST_PULSE, "press": SERVO3_PRESS_PULSE},
]


def find_key_servo(key: str):
    for key_servo in LINEAR_SERVOS:
        if key_servo["key"] == key:
            return key_servo
    return None

def rotate_servo(pi: pigpio.pi, degree: float, pin: int):
    # Calculate rotation degree into pulse width
    pulse_width = calc_us(degree)
    rc = pi.set_servo_pulsewidth(pin, pulse_width)
    if rc != status_t.OK:
        print(f"set_servo_pulsewidth failed on pin {pin} with code {rc}", file=sys.stderr)


def move_servo(pi: pigpio.pi, pin: int, pulse_width: int) -> None:
    rc = pi.set_servo_pulsewidth(pin, pulse_width)
    if rc != status_t.OK:
        print(f"set_servo_pulsewidth failed on pin {pin} with code {rc}", file=sys.stderr)

def press_key(pi: pigpio.pi, key: str) -> None:
    key_servo = find_key_servo(key)
    if key_servo is None:
        print(f"Ignoring unsupported key '{key}'", file=sys.stderr)
        return

    move_servo(pi, key_servo["pin"], key_servo["press"])
    time.sleep(PRESS_HOLD_TIME)

    move_servo(pi, key_servo["pin"], key_servo["rest"])
    time.sleep(BETWEEN_KEYS_TIME)


def all_servos_to_rest(pi: pigpio.pi) -> None:
    for key_servo in LINEAR_SERVOS:
        move_servo(pi, key_servo["pin"], key_servo["rest"])
    time.sleep(0.2)


def main() -> int:

    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio failed to initialize.", file=sys.stderr)
        return 1

    try:
        all_servos_to_rest(pi)

        print(f"Pressing keypad sequence: {RAW_TASK_SEQUENCE}")

        # S
        for key in FULL_TASK_SEQUENCE:
            if key in " \t\n":
                continue
            if key == '^':
                rotate_servo(pi, 156, SERVO4_PIN)
            elif key == 'v':
                rotate_servo(pi, 150, SERVO4_PIN)
            else:
                press_key(pi, key)

        all_servos_to_rest(pi)
    finally:
        for key_servo in LINEAR_SERVOS:
            # Disable Linear Servos
            pi.set_servo_pulsewidth(key_servo["pin"], 0) 
            
        # Disable rotational servo
        pi.set_servo_pulsewidth(SERVO4_PIN, 0)    
        pi.stop()

    return status_t.OK


# Calculate pulse width for GoBilda servo
def calc_us(degree: float):
    degree = clamp(degree, 0, 300)
    return 500 + ((degree/300) * 2000)

def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))

if __name__ == "__main__":
    sys.exit(main())
