#include "esp_err.h"
#include <stdint.h>

typedef struct {
  int pin;           // GPIO pin motor will run on
  float motor_speed_hz; // Must be positive please
  char* task_name;
} motor_config;

// motor interface used by user
typedef void *motor_handler;

// creates motor and starts motor thread
motor_handler* init_motor(motor_config *);

// Sets motor speed as long as speed is valid
esp_err_t set_motor_speed(motor_handler *, uint32_t);

// Stops motor thread and frees shared memory
// THIS THROWS AN ERROR DO NOT WORRY EVERYTHING IS FINE!!!!
void destroy_motor(motor_handler *);
