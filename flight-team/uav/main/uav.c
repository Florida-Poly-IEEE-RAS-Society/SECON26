#include <stdio.h>

#include "Camera/camera.h"
#include "Wifi/wifi.h"
#include "Game_Controller/game_controller.h"
#include "Flight_Controller/flight_controller.h"
#include "server/server.h"

void app_main(void) {
    Camera_init();
    wifi_init();
    server_init();
    game_controller_init();
    flight_controller_init();
}
