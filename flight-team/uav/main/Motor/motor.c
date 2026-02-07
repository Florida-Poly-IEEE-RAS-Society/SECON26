#include "motor.h"
#include "bdc_motor.h"
#include "esp_err.h"
#include <esp_log.h>
#include <esp_timer.h>
#include "sdkconfig.h"
#include <freertos/FreeRTOS.h>
#include "freertos/timers.h"
#include "soc/gpio_num.h"
#include <assert.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

typedef struct {
	gpio_num_t pin;
	bdc_motor_handle_t motor;
	uint32_t duty_cycle_ticks; // duty cycle of pwm in ticks
	char* task_name;
} motor_dev;

motor_handler* init_motor(motor_config* config) {
	ESP_LOGI(config->task_name, "Init Motor: %s", config->task_name);
	motor_dev* motor = malloc(sizeof(motor_dev)); 
	if (!motor) return NULL;

	*motor = (motor_dev){
		.pin = config->pin, // add validation for this later?
		.task_name = config->task_name,
	};

	bdc_motor_config_t motor_config = {
		.pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
		// bdc_pwm requires we define both of these but we'll only use a since we only need to turn one direction
		.pwma_gpio_num = 5, // random gpio number
		.pwmb_gpio_num = motor->pin,
	};
	bdc_motor_mcpwm_config_t mcpwm_config = {
		.group_id = 0,
		.resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
	};
	ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor->motor));

	ESP_LOGI(config->task_name, "Enable motor");
	ESP_ERROR_CHECK(bdc_motor_enable(motor->motor));
	ESP_LOGI(config->task_name, "Forward motor");
	// reverse because library is fucked!!!!!!!!
	ESP_ERROR_CHECK(bdc_motor_reverse(motor->motor));

	return (motor_handler*) motor;
}

esp_err_t set_motor_speed(motor_handler* motor_arg, uint32_t new_speed_ticks) {
	motor_dev* motor = (motor_dev*)motor_arg;

	if (new_speed_ticks > BDC_MCPWM_DUTY_TICK_MAX) {
		return ESP_ERR_INVALID_ARG;
	}

	esp_err_t error = bdc_motor_set_speed(motor->motor, new_speed_ticks);

	if (error == ESP_OK) {
		motor->duty_cycle_ticks = new_speed_ticks;
	}

	return error;
}

// THIS THROWS AN ERROR DO NOT WORRY EVERYTHING IS FINE!!!!
void destroy_motor(motor_handler* motor) {
	motor_dev* m = (motor_dev*) motor;
	ESP_LOGI(m->task_name, "Forward motor");
	bdc_motor_del(m->motor);
	free(motor);
}
