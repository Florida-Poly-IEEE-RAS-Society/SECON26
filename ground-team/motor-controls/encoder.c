#include "encoder.h"

static void _encoder_callback(int num_alerts, lgGpioAlert_p alerts,
                              void *userdata) {
  encoder_t *enc = (encoder_t *)userdata;

  for (int i = 0; i < num_alerts; i++) {
    int gpio = alerts[i].report.gpio;
    int level = alerts[i].report.level;

    int b = (gpio == enc->pin_b) ? level : enc->last_b;

    if (gpio == enc->pin_a)
      enc->last_a = level;
    if (gpio == enc->pin_b)
      enc->last_b = level;

    if (gpio == enc->pin_a && level == 1) {
      if (b == 0)
        atomic_fetch_add(&enc->count, 1);
      else
        atomic_fetch_add(&enc->count, -1);
    } else if (gpio == enc->pin_b && level == 1) {
      if (enc->last_a == 1)
        atomic_fetch_add(&enc->count, 1);
      else
        atomic_fetch_add(&enc->count, -1);
    }
  }
}

status_t enc_init(int gpio_handle, encoder_t *enc, int pin_a, int pin_b) {
  enc->pin_a = pin_a;
  enc->pin_b = pin_b;
  atomic_store(&enc->count, 0);
  enc->last_a = lgGpioRead(gpio_handle, pin_a);
  enc->last_b = lgGpioRead(gpio_handle, pin_b);

  if (lgGpioClaimInput(gpio_handle, 0, pin_a) < 0)
    return ERR_BUS_FAIL;
  if (lgGpioClaimInput(gpio_handle, 0, pin_b) < 0)
    return ERR_BUS_FAIL;

  if (lgGpioSetAlertsFunc(gpio_handle, pin_a, _encoder_callback, enc) < 0)
    return ERR_ADDRESS_FAIL;
  if (lgGpioSetAlertsFunc(gpio_handle, pin_b, _encoder_callback, enc) < 0)
    return ERR_ADDRESS_FAIL;

  return OK;
}

void enc_reset(encoder_t *enc) { atomic_store(&enc->count, 0); }

int32_t enc_get_count(const encoder_t *enc) { return atomic_load(&enc->count); }

float enc_get_dist_in(const encoder_t *enc) {
  return counts_to_in(atomic_load(&enc->count));
}
