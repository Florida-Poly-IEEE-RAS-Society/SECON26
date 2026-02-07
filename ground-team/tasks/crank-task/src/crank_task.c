#include "../../../core/include/robot-core.h"
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

#define SERVO_PIN 18
#define SERVO_MIN 500
#define SERVO_MAX 2500
#define PWM_PERIOD_US 20000
#define STEP_DELAY_US 10000

static void set_servo_pulse(struct gpiod_line *line, int pulse_us) {
  struct timespec high = {0, pulse_us * 1000};
  struct timespec low = {0, (PWM_PERIOD_US - pulse_us) * 1000};

  gpiod_line_set_value(line, 1);
  nanosleep(&high, NULL);
  gpiod_line_set_value(line, 0);
  nanosleep(&low, NULL);
}

status_t crank_task(void) {
  struct gpiod_chip *chip;
  struct gpiod_line *servo_line;

  chip = gpiod_chip_open_by_name("gpiochip0");
  if (!chip)
    return ERR_BUS_FAIL;

  servo_line = gpiod_chip_get_line(chip, SERVO_PIN);
  if (!servo_line) {
    gpiod_chip_close(chip);
    return ERR_BUS_FAIL;
  }

  if (gpiod_line_request_output(servo_line, "crank", 0) < 0) {
    gpiod_chip_close(chip);
    return ERR_BUS_FAIL;
  }

  printf("Rotating 360 degrees\n");

  for (int pulse = SERVO_MIN; pulse <= SERVO_MAX; pulse += 100) {
    set_servo_pulse(servo_line, pulse);
    usleep(STEP_DELAY_US);
  }

  gpiod_line_release(servo_line);
  gpiod_chip_close(chip);

  return OK;
}

int main(void) {
  status_t result = crank_task();

  if (result != OK) {
    fprintf(stderr, "Crank task failed: %d\n", result);
    return result;
  }

  printf("Crank task completed\n");
  return OK;
}
