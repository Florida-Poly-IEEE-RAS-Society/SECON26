#pragma once

#include "../core/robot-core.h"
#include <lgpio.h>
#include <stdint.h>

// M1: forward=21, back=20
#define ENC1_PIN_A  25
#define ENC1_PIN_B  24

// M2: forward=12, back=16
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
float    enc_get_dist_in(const encoder_t *enc);
