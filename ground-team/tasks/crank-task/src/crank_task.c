#include "../include/crank_task.h"

void crank_task() {
    printf("Rotating to 360");
    for (int pulse = SERVO_MIN; pulse <= SERVO_MAX; pulse += 100) {
        gpioServo(SERVO_PIN, pulse);
        usleep(10000);
    }
}