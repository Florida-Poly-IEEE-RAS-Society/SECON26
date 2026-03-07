#include "../Camera/camera.h"
#include "esp_log.h"

#include <Game_Controller/game_controller.h>
#include <Flight_Controller/flight_controller.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

static const char *TAG = "TCP_SERVER";

void command_image(int sock) {
    ESP_LOGI(TAG, "Serving IMAGE command...");
    char size_buffer[4];
    memset(size_buffer, 0, 4);
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "Error while taking picture");
        int written = send(sock, size_buffer, 4, 0);
        if (written != 4) {
            ESP_LOGE(TAG, "I heard you like errors, so I put a socket error inside of your camera error so you can error while you error");
            return;
        }
    }

    //fill size_buffer with a big-endian encoded version of len
    for (uint32_t i = 0; i < 4; i++) {
        size_buffer[4-i-1] = ((uint32_t)(fb->len)) >> (i*8) & (0xFF);
    }

    ESP_LOGI(TAG, "Image length: %d", fb->len);

    int written = send(sock, size_buffer, 4, 0);
    if (written != 4) {
        ESP_LOGE(TAG, "Error occurred while sending image size: errno %d", errno);
        esp_camera_fb_return(fb);
        return;
    }

    written = send(sock, fb->buf, fb->len, 0);
    if (written != fb->len) {
        ESP_LOGE(TAG, "Error occurred while sending image data: errno %d", errno);
        esp_camera_fb_return(fb);
        return;
    }
    esp_camera_fb_return(fb);
};

void command_launch(int sock) {
    game_state_change_maybe(Game_Launch);
}

void command_retrieve(int sock) {
    game_state_change_maybe(Game_Retrieve);
}

void command_transmission_codes(int sock) {
    ir_nec_scan_code_t codes[4];
    int length = recv(sock, codes, sizeof(codes), MSG_WAITALL);
    if (length == sizeof(codes)) {
        game_set_ir_codes(codes);
        game_state_change_maybe(Game_Send_Codes);
    } else {
        ESP_LOGE(TAG, "Error occurred while getting transmission codes: errno %d", errno);
    }
}

void command_pos(int sock) {
    float pos[4];
    int length = recv(sock, pos, sizeof(pos), MSG_WAITALL);
    if (length == sizeof(pos)) game_set_pos_data(pos[0], pos[1], pos[2], pos[3]);
    else ESP_LOGE(TAG, "Error occurred while getting position: errno %d", errno);
}

void command_stop(int sock) {
    emergency_stop();
}
