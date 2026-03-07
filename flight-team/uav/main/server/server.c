#include <string.h>
#include "esp_log.h"

#include <Camera/camera.h>
#include <Gyro/bno055.h>
#include <Flight_Controller/flight_controller.h>
#include <Game_Controller/game_controller.h>

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#define PORT 3333

enum Command {
  /*
  IMAGE command
  header: no header
  response: a 4 byte size, then the image data
  On error taking image: a size of 0 followed by no data
  */
  IMAGE = 1,
  /*
  LAUNCH command
  header: no header
  response: none
  */
  LAUNCH,
  /*
  RETRIEVE command
  header: no header
  response: none
  */
  RETRIEVE,
  /*
  TRANSMISSION_CODES command
  header: 4 x ir_nec_scan_code_t (IR transmission codes)
  response: none
  */
  TRANSMISSION_CODES,
  /*
  POS command
  header: 4 floats (position data: x, y, z, and one more)
  response: none
  */
  POS,
  /*
  STOP command
  header: no header
  response: none
  */
  STOP,
  /*
  THRUST command
  header: 1 float (throttle power)
  response: none
  */
  THRUST,
  /*
  THRUST_CTRL_MODE command
  header: 1 uint8_t (thrust control mode)
  response: none
  */
  THRUST_CTRL_MODE,
  PITCH,
  ROLL,
  YAW,
  /*
  SET_HEIGHT command
  header: 1 float (height delta)
  response: none
  */
  SET_HEIGHT,
  /*
  SET_X command
  header: 1 float (x position delta)
  response: none
  */
  SET_X,
  /*
  SET_Y command
  header: 1 float (y position delta)
  response: none
  */
  SET_Y,
  /*
  SET_PID command
  header: struct { uint8_t pid_idx; uint8_t param_idx; float value; }
  response: none
  */
  SET_PID,
  /*
  GET_PID command
  header: struct { uint8_t pid_idx; uint8_t param_idx; } (same as SET_PID without value)
  response: 1 float (current PID parameter value)
  */
  GET_PID,
  /*
  SAVE_PID command
  header: none 
  response: 1 byte (1 if saved pid parameters, 0 if it failed)
  */
  SAVE_PID,
  /*
  GYRO_CALIBRATION_STATUS command
  header: no header
  response: 4 bytes (system, gyro, accel, mag calibration status)
  */
  GYRO_CALIBRATION_STATUS,
  /*
  GET_GAME_STATE command
  header: no header
  response: 1 byte (current game state)
  */
  GET_GAME_STATE,
  /*
  POS_VEL command
  header: no header
  response: 6 floats (x_pos, y_pos, z_pos, x_vel, y_vel, z_vel), in meters and m/s
  */
  POS_VEL,
};

static const char *TAG = "TCP_SERVER";

static void command_image(int sock) {
    ESP_LOGI(TAG, "Serving IMAGE command...");
    char size_buffer[4];
    memset(size_buffer, 0, 4);
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "Error while taking picture");
        int written = send(sock, size_buffer, 4, 0);
        if (written != 4) {
            ESP_LOGE(TAG, "I heard you like errors, so I put a socket error inside of your camera error so you can error while you error");
        }
        return;
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
}

static void serve_command(int sock) {
    char command;
    int length = recv(sock, &command, 1, MSG_WAITALL);
    if (length != 1) {
        ESP_LOGE(TAG, "Error occurred while reading command: errno %d", errno);
        return;
    }

    switch (command) {
    case IMAGE:
        command_image(sock);
        break;
    case LAUNCH:
        game_state_change_maybe(Game_Launch);
        break;
    case RETRIEVE:
        game_state_change_maybe(Game_Retrieve);
        break;
    case TRANSMISSION_CODES: {
        ir_nec_scan_code_t codes[4];
        int length = recv(sock, codes, sizeof(codes), MSG_WAITALL);
        if (length == sizeof(codes)) {
            game_set_ir_codes(codes);
            game_state_change_maybe(Game_Send_Codes);
        } else {
            ESP_LOGE(TAG, "Error occurred while getting transmission codes: errno %d", errno);
        }
    } break;
    case POS: {
        float pos[4];
        int length = recv(sock, pos, sizeof(pos), MSG_WAITALL);
        if (length == sizeof(pos)) game_set_pos_data(pos[0], pos[1], pos[2], pos[3]);
        else ESP_LOGE(TAG, "Error occurred while getting position: errno %d", errno);
    } break;
    case STOP:
        emergency_stop();
        break;
    case THRUST: {
        float power;
        int l = recv(sock, &power, sizeof(power), MSG_WAITALL);
        if (l == sizeof(power)) set_throttle(power);
    } break;
    case THRUST_CTRL_MODE: {
        uint8_t x;
        int l = recv(sock, &x, sizeof(x), MSG_WAITALL);
        if (l == sizeof(x)) set_thrust_control_mode(x);
    } break;
    case PITCH:
        break;
    case ROLL:
        break;
    case YAW:
        break;
    case SET_HEIGHT: {
        float h;
        int l = recv(sock, &h, sizeof(h), MSG_WAITALL);
        if (l == sizeof(h)) change_height_by(h);
    } break;
    case SET_X: {
        float x_pos;
        int l = recv(sock, &x_pos, sizeof(x_pos), MSG_WAITALL);
        if (l == sizeof(x_pos)) change_pos_by(x_pos, 0);
    } break;
    case SET_Y: {
        float y_pos;
        int l = recv(sock, &y_pos, sizeof(y_pos), MSG_WAITALL);
        if (l == sizeof(y_pos)) change_pos_by(0, y_pos);
    } break;
    case SET_PID: {
        struct Set_Pid_Type { uint8_t pid_idx; uint8_t param_idx; float value; } params;
        int l = recv(sock, &params, sizeof(params), MSG_WAITALL);
        if (l == sizeof(params)) {
            set_pid(params.pid_idx, params.param_idx, params.value);
        }
    } break;
    case GET_PID: {
        struct Get_Pid_Type { uint8_t pid_idx; uint8_t param_idx; } params;
        int l = recv(sock, &params, sizeof(params), MSG_WAITALL);
        if (l == sizeof(params)) {
            float value = get_pid(params.pid_idx, params.param_idx);
            send(sock, &value, sizeof(value), 0);
        } else {
            ESP_LOGE(TAG, "Error occurred while getting PID params: errno %d", errno);
        }
    } break;
    case GYRO_CALIBRATION_STATUS: {
        uint8_t data[4]; //system, gyro, accel, mag;
        bno055_getCalibration(&data[0], &data[1], &data[2], &data[3]);
        send(sock, data, sizeof(data), 0);
    } break;
    case GET_GAME_STATE: {
        enum Game_State gs = game_get_state();
        uint8_t data = (uint8_t)gs;
        send(sock, &data, sizeof(data), 0);
    } break;
    case POS_VEL: {
        float data[6] = {
            get_x_pos(), get_y_pos(), get_z_pos(),
            get_x_vel(), get_y_vel(), get_z_vel(),
        };
        send(sock, data, sizeof(data), 0);
    } break;
    }
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = 5;
    int keepInterval = 5;
    int keepCount = 3;
    struct sockaddr_storage dest_addr;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string

        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }

        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);

        serve_command(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

void server_init() {
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
}
