#include <stdio.h>

#include "IR/ir.h"
#include "Camera/camera.h"
#include "Gyro/adafruit_bno055.h"

#define IR_GPIO 8

void app_main(void) {
    bno055_begin(OPERATION_MODE_NDOF);
    Camera_init();

    IRtx_t ir;
    IRtx_init(&ir, IR_GPIO);
}
