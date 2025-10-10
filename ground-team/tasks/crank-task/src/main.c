#include <stdio.h>
#include <pigpio.h>
#include <unistd.h>

#define SERVO_MAX 5000
#define SERVO_MIN 500
#define SERVO_PIN 18

void crank_task() {
    printf("Rotating to 360");
    for (int pulse = SERVO_MIN; pulse <= SERVO_MAX; pulse += 100) {
        gpioServo(SERVO_PIN, pulse);
        usleep(10000);
    }
}

int main(void) {
    printf("Rotating Cranking");

    if(gpioInitalise() < 0) {
        fprintf(stderr, "pigpio failed to initialize.\n");
        return 1;
    }
    
    crank_task();
}
