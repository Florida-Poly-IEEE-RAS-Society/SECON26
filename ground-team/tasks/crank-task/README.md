# Crank Task

***Problem***:
The ground robot is tasked with performing one full rotation (360 degrees) of a crank to earn points.
This can be done by controlling a servo motor using PWM signals.

What we have:
Following the servo motor's control signal (PWM pulses), we know that:
- 0 degree = 500 µs (microseconds) => Smallest value of servo.
- 360 degrees = 5000 µs (microseconds) => Largest value of servo.

***Solution***:
Use a for loop to increment the servo signal from the smallest value to the largest value to roll the crank a full 360 degree.
Set the delay to 10000 µs between steps to allow time for movement and ensure a smooth, continous motion.

##Note:
- We need to check whether if the pigpio header file is properly initialized:
  By checking if the function gpioInitalise() < 0, we can stop the program and exit.

## CODE:

`
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

    if(gpioInitialise() < 0) {
        fprintf(stderr, "pigpio failed to initialize.\n");
        return 1;
    }
    
    crank_task();
}
`
