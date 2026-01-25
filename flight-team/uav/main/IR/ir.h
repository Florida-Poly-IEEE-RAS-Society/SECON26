#pragma once

#include <stdint.h>
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"

typedef struct {
    uint16_t address;
    uint16_t command;
} ir_nec_scan_code_t;

typedef struct {
    rmt_channel_handle_t channel;
    rmt_encoder_handle_t encoder;
    rmt_transmit_config_t transmit_cfg;
} IRtx_t;


esp_err_t IRtx_init(IRtx_t *ir_handle, int gpio);
esp_err_t IRtx_transmit(IRtx_t *ir, ir_nec_scan_code_t scan_code);
