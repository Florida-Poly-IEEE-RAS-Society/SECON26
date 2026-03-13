#include "../../../core/robot-core.h"
#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>

#define SERVO_PIN_V 24
#define SERVO_PIN_H 26
#define SERVO_MIN 500
#define SERVO_MID 1500
#define SERVO_MAX 2500
#define PWM_PERIOD_US 20000
#define STEP_DELAY_US 10000
#define MOVE_TIME_MS 800

static void set_servo_pulse(
		struct gpiod_line *s1,
     		int pulse_us)
{
    struct timespec high = {0, pulse_us * 1000};
    struct timespec low = {0, (PWM_PERIOD_US - pulse_us) * 1000};

    gpiod_line_set_value(s1, 1);
    nanosleep(&high, NULL);
    gpiod_line_set_value(s1, 0);
    nanosleep(&low, NULL);
}

void move_servo(struct gpiod_line *line, int pulse_us) {
	int cycles = (MOVE_TIME_MS * 1000) / PWM_PERIOD_US;

	for (int i = 0; i < cycles; i++){
		send_pulse(line, pulse_us);
	}
}

status_t pressure_plate{
      struct gpiod_chip *chip;
      struct gpiod_line *servo_line;

  chip = gpiod_chip_open_by_name("gpiochip0");
  if (!chip)
    return ERR_BUS_FAIL;

  servo1_line = gpiod_chip_get_line(chip, SERVO_PIN_V);
  if (!servo1_line) {
    gpiod_chip_close(chip);
    return ERR_BUS_FAIL;
  }
  servo2_line = gpiod_chip_get_line(chip, SERVO_PIN_H);
  if (!servo2_line) {
	  gpiod_chip_close(chip);
	  return ERR_BUS_FAIL;
  }

  if (gpiod_line_request_output(servo1_line, "Vertical", 0) < 0) {
    gpiod_chip_close(chip);
    return ERR_BUS_FAIL;
  }
  if (gpiod_line_request_output(servo2_line, "Horizontal", 0) < 0) {
    gpiod_chip_close(chip);
    return ERR_BUS_FAIL;
  }

   printf("Ensure vertical servo to be perpendicular");
   move_servo(s1, 1500);
   printf("Moving vertical servo to the front to x degrees");
   move_servo(s1, 500);
   printf("Moving horizontal servo to the right");
   move_servo(s2, 500);
   printf("Moving horizontal servo to the left");
   move_servo(s2, 2500);
   printf("Moving horizontal servo to the middle");
   move_servo(s2, 1500);
   printf("Move vertical servo back to perpendicular");


  gpiod_line_release(servo1_line);
  gpiod_line_release(servo2_line);
  gpiod_chip_close(chip);
  return OK;
}


void main(void) {
    status_t result = pressure_plate();

    if(result != OK)
    {
	fprintf(stderr, "Pressure plate task failed: %d\n", result);
        return result;
    }

    printf("Pressure plate task complete\n");
    return OK;
}
