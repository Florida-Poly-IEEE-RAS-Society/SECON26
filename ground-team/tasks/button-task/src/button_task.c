#include "../include/button_task.h"
#include <lgpio.h>
#include <stdio.h>
#include <unistd.h>

#define SERVO_PIN 27
#define SERVO_EXTENDED 2500
#define SERVO_RETRACTED 1000
#define PWM_PERIOD_US 20000
#define PRESS_HOLD_MS 500
#define PRESS_DELAY_S 3
#define PRESS_COUNT 2

static void set_servo_pulse(int handle, int pulse_us) {
  lgGpioWrite(handle, SERVO_PIN, 1);
  lguSleep((double)pulse_us / 1e6);
  lgGpioWrite(handle, SERVO_PIN, 0);
  lguSleep((double)(PWM_PERIOD_US - pulse_us) / 1e6);
}

static void hold_position(int handle, int pulse_us, int hold_ms) {
  int cycles = (hold_ms * 1000) / PWM_PERIOD_US;
  for (int i = 0; i < cycles; i++)
    set_servo_pulse(handle, pulse_us);
}

static void press_button(int handle) {
  printf("Pressing button...\n");
  hold_position(handle, SERVO_EXTENDED, PRESS_HOLD_MS);
  printf("Retracting...\n");
  hold_position(handle, SERVO_RETRACTED, PRESS_HOLD_MS);
}

status_t button_task(void) {
  int handle = lgGpiochipOpen(0);
  if (handle < 0)
    return ERR_BUS_FAIL;

  if (lgGpioClaimOutput(handle, 0, SERVO_PIN, 0) < 0) {
    lgGpiochipClose(handle);
    return ERR_BUS_FAIL;
  }

  hold_position(handle, SERVO_RETRACTED, PRESS_HOLD_MS);

  for (int i = 0; i < PRESS_COUNT; i++) {
    press_button(handle);
    if (i < PRESS_COUNT - 1) {
      printf("Waiting %d seconds...\n", PRESS_DELAY_S);
      sleep(PRESS_DELAY_S);
    }
  }

  lgGpiochipClose(handle);
  return OK;
}

int main(void) {
  status_t result = button_task();
  if (result != OK) {
    fprintf(stderr, "Button task failed: %d\n", result);
    return result;
  }
  printf("Button task complete\n");
  return OK;
}
