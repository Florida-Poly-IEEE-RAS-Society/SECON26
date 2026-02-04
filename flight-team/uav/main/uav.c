#include "IR/ir.h"
#include "Camera/camera.h"
#include "Gyro/bno055.h"
#include "Wifi/wifi.h"

#include <stdio.h>

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"

#define IR_GPIO 8

// arbitrary
#define GYRO_ID 55

void app_main(void) {
    /* bool ok = bno055_begin(GYRO_ID, OPERATION_MODE_NDOF, BNO055_ADDRESS_A); */
    /* if (!ok) { */
    /*     printf("Gyro not found"); */
    /* } else { */
    /*     bno055_setExtCrystalUse(true); */

    /*     uint8_t system_status, self_test_result, system_error; */
    /*     bno055_getSystemStatus(&system_status, &self_test_result, &system_error); */
    /*     printf("Status: %d, Self Test: %d, System Error: %d\n", system_status, self_test_result, system_error); */
    /* } */

    /* while (1) { */
    /*     sensors_event_t event; */
    /*     bno055_getEvent1(&event); */
    /*     printf("%ld: (%f, %f, %f)\n", event.sensor_id, event.orientation.x, event.orientation.y, event.orientation.z); */
    /*     vTaskDelay(100 / portTICK_PERIOD_MS); */
    /* } */
    

    Camera_init();

    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        printf("UH OH!!!!\n");
        return;
    }

    printf("THE PICTURE IS THIS BIG!!!!: %zu\n", fb->len);

    /* IRtx_t ir; */
    /* IRtx_init(&ir, IR_GPIO); */

    wifi_init();
}
