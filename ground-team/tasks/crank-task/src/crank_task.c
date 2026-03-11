#include "../include/crank_task.h"
#include <lgpio.h>
#include <stdio.h>
#include <unistd.h>

#define SERVO_PIN 5
#define PWM_PERIOD_US 20000

#define SERVO_STOP 1500
#define SERVO_CW 1300
#define SERVO_CCW 1700

#define MS_PER_360 1200
#define MS_FOR_540 ((MS_PER_360 * 3) / 2)

#define DIRECTION SERVO_CCW

static void set_servo_pulse(int handle, int pulse_us) {
  lgGpioWrite(handle, SERVO_PIN, 1);
  lguSleep((double)pulse_us / 1e6);
  lgGpioWrite(handle, SERVO_PIN, 0);
  lguSleep((double)(PWM_PERIOD_US - pulse_us) / 1e6);
}

static void run_servo_for_ms(int handle, int pulse_us, int duration_ms) {
  int cycles = (duration_ms * 1000) / PWM_PERIOD_US;
  for (int i = 0; i < cycles; i++)
    set_servo_pulse(handle, pulse_us);
}

status_t crank_task(void) {
  int handle = lgGpiochipOpen(0);
  if (handle < 0)
    return ERR_BUS_FAIL;

  if (lgGpioClaimOutput(handle, 0, SERVO_PIN, 0) < 0) {
    lgGpiochipClose(handle);
    return ERR_BUS_FAIL;
  }

  printf("Rotating 540 degrees...\n");
  run_servo_for_ms(handle, DIRECTION, MS_FOR_540);

  printf("Stopping servo\n");
  run_servo_for_ms(handle, SERVO_STOP, 200);

  lgGpiochipClose(handle);
  return OK;
}

int main(void) {
  status_t result = crank_task();
  if (result != OK) {
    fprintf(stderr, "Crank task failed: %d\n", result);
    return result;
  }
  printf("Crank task complete\n");
  return OK;
}
