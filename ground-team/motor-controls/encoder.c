#include "encoder.h"

static void _encoder_callback(int handle, unsigned int gpio,
                               unsigned int level, uint32_t tick,
                               void *userdata) {
    (void)handle; (void)tick;
    encoder_t *enc = (encoder_t *)userdata;

    int b = (gpio == (unsigned int)enc->pin_b) ? (int)level : enc->last_b;

    if ((int)gpio == enc->pin_a) enc->last_a = (int)level;
    if ((int)gpio == enc->pin_b) enc->last_b = (int)level;

    if ((int)gpio == enc->pin_a && level == 1) {
        if (b == 0) enc->count++;
        else        enc->count--;
    } else if ((int)gpio == enc->pin_b && level == 1) {
        if (enc->last_a == 1) enc->count++;
        else                  enc->count--;
    }
}

status_t enc_init(int gpio_handle, encoder_t *enc, int pin_a, int pin_b) {
    enc->pin_a  = pin_a;
    enc->pin_b  = pin_b;
    enc->count  = 0;
    enc->last_a = lgGpioRead(gpio_handle, pin_a);
    enc->last_b = lgGpioRead(gpio_handle, pin_b);

    if (lgGpioClaimInput(gpio_handle, 0, pin_a) < 0) return ERR_BUS_FAIL;
    if (lgGpioClaimInput(gpio_handle, 0, pin_b) < 0) return ERR_BUS_FAIL;

    if (lgGpioSetAlertsFunc(gpio_handle, pin_a, _encoder_callback, enc) < 0)
        return ERR_ADDRESS_FAIL;
    if (lgGpioSetAlertsFunc(gpio_handle, pin_b, _encoder_callback, enc) < 0)
        return ERR_ADDRESS_FAIL;

    return OK;
}

void enc_reset(encoder_t *enc) {
    enc->count = 0;
}

int32_t enc_get_count(const encoder_t *enc) {
    return enc->count;
}

float enc_get_dist_in(const encoder_t *enc) {
    return counts_to_in(enc->count);
}
