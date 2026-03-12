// encoder.h
//
// Quadrature encoder driver using lgpio alerts.
// Each encoder uses two Hall effect channels (A and B).
//
// Wiring
// M1 encoder A = GPIO 25
// M1 encoder B = GPIO 24
// M2 encoder A = GPIO 23
// M2 encoder B = GPIO 18

#pragma once

#include "../core/include/robot-core.h"
#include <lgpio.h>
#include <stdint.h>

#define ENC1_PIN_A  25
#define ENC1_PIN_B  24
#define ENC2_PIN_A  23
#define ENC2_PIN_B  18

typedef struct {
    volatile int32_t count;
    int pin_a;
    int pin_b;
    int last_a;
    int last_b;
} encoder_t;

status_t enc_init(int gpio_handle, encoder_t *enc, int pin_a, int pin_b);
void     enc_reset(encoder_t *enc);
int32_t  enc_get_count(const encoder_t *enc);
float    enc_get_dist_m(const encoder_t *enc);
