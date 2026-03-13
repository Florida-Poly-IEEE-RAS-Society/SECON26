#!/usr/bin/env python3
"""
Keypad servo controller using pigpio on Raspberry Pi.
Controls three servos, each assigned to a single keypad button.
Usage: python main.py [sequence]
Example: python main.py 123
"""

import sys
import time
import pigpio

# Adjust these to match your wiring (BCM pin numbers)
SERVO1_PIN = 18
SERVO2_PIN = 19
SERVO3_PIN = 20

# Adjust these pulse widths (in microseconds) after calibration
# Typical range is ~500–2500 µs; 1500 µs is usually "center".

# Servo 1 -> Pressing 7
SERVO1_REST_PULSE = 1500
SERVO1_PRESS_PULSE = 1000

# Servo 2 -> Pressing
SERVO2_REST_PULSE = 1500
SERVO2_PRESS_PULSE = 1000

# Servo 3 -> Pressing 
SERVO3_REST_PULSE = 1500
SERVO3_PRESS_PULSE = 1000

# Rotation Servo for Servo 3 -> Rotating 6 degrees to hit
SERVO4_ROTATE_PULSE = 1540
SERVO4_REST_PULSE = 1500

# How long to hold the button down and pause between presses (seconds)
PRESS_HOLD_TIME = 0.3
BETWEEN_KEYS_TIME = 0.2

KEY_SERVOS = [
    {"key": "7", "pin": SERVO1_PIN, "rest": SERVO1_REST_PULSE, "press": SERVO1_PRESS_PULSE},
    {"key": "8", "pin": SERVO2_PIN, "rest": SERVO2_REST_PULSE, "press": SERVO2_PRESS_PULSE},
    {"key": "3", "pin": SERVO3_PIN, "rest": SERVO3_REST_PULSE, "press": SERVO3_PRESS_PULSE},
    {"key": "#", "pin": SERVO3_PIN, "rest": SERVO3_REST_PULSE, "press": SERVO3_PRESS_PULSE},
]


def find_key_servo(key: str):
    for ks in KEY_SERVOS:
        if ks["key"] == key:
            return ks
    return None


def move_servo(pi: pigpio.pi, pin: int, pulse_width: int) -> None:
    rc = pi.set_servo_pulsewidth(pin, pulse_width)
    if rc != 0:
        print(f"set_servo_pulsewidth failed on pin {pin} with code {rc}", file=sys.stderr)

def press_key(pi: pigpio.pi, key: str) -> None:
    ks = find_key_servo(key)
    if ks is None:
        print(f"Ignoring unsupported key '{key}'", file=sys.stderr)
        return

    move_servo(pi, ks["pin"], ks["press"])
    time.sleep(PRESS_HOLD_TIME)

    move_servo(pi, ks["pin"], ks["rest"])
    time.sleep(BETWEEN_KEYS_TIME)


def all_servos_to_rest(pi: pigpio.pi) -> None:
    for ks in KEY_SERVOS:
        move_servo(pi, ks["pin"], ks["rest"])
    time.sleep(0.2)


def main() -> int:

    print(f'Pulse width for 6 degree rotation = {calc_us(156.0)}')
    return
    # Remove above
    pi = pigpio.pi()
    if not pi.connected:
        print("pigpio failed to initialize.", file=sys.stderr)
        return 1

    try:
        all_servos_to_rest(pi)

        if len(sys.argv) >= 2:
            sequence = sys.argv[1]
        else:
            sequence = "756"
            print(f"No sequence provided, using default '{sequence}'")

        print(f"Pressing keypad sequence: {sequence}")

        for key in sequence:
            if key in " \t\n":
                continue
            press_key(pi, key)

        all_servos_to_rest(pi)
    finally:
        for ks in KEY_SERVOS:
            pi.set_servo_pulsewidth(ks["pin"], 0)  # disable servo
        pi.stop()

    return 0


# Calculate pulse width for GoBilda servo
def calc_us(degree: float):
    return 500 + ((degree/300) * 2000)

if __name__ == "__main__":
    sys.exit(main())
