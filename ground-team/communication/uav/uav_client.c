#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "include/uav_client.h"

#define SERVER_IP "127.0.0.1" // Change to "192.168.4.1" for the actual drone
#define SERVER_PORT 3333      // Matched with PORT 3333 from server.c

// Function to receive exactly 'amount' bytes
int recv_exact(int sock, uint8_t *buffer, size_t amount) {
    size_t total_received = 0;
    while (total_received < amount) {
        ssize_t received = recv(sock, buffer + total_received, amount - total_received, 0);
        if (received <= 0) {
            return -1; // Error or connection closed
        }
        total_received += received;
    }
    return 0;
}

// Helper function to connect to the UAV server
int connect_to_server() {
    int sock = 0;
    struct sockaddr_in serv_addr;

    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        perror("Socket creation error");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(SERVER_PORT);

    if (inet_pton(AF_INET, SERVER_IP, &serv_addr.sin_addr) <= 0) {
        perror("Invalid address/ Address not supported");
        close(sock);
        return -1;
    }

    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("Connection Failed");
        close(sock);
        return -1;
    }

    return sock;
}

// Requests an image from the UAV and saves it to a file
image_response* image_handler(int sock) {
    const char* filename = "./output.jpg"; // for debug only
    uint8_t cmd = IMAGE;
    image_response* response = malloc(sizeof(image_response));
    if (send(sock, &cmd, 1, 0) != 1) {
        perror("Failed to send IMAGE command");
        return NULL;
    }

    uint8_t size_buffer[4];
    if (recv_exact(sock, size_buffer, 4) < 0) {
        perror("Failed to receive image size");
        return NULL;
    }

    uint32_t size = 0;
    for (int i = 0; i < 4; i++) {
        size |= ((uint32_t)size_buffer[4 - i - 1]) << (i * 8);
    }

    response->size = size;

    printf("-> Image size reported by UAV: %u bytes\n", size);

    if (size == 0) {
        printf("-> No image data received.\n");
        return NULL;
    }

    if (recv_exact(sock, response->data, size) < 0) {
        perror("Failed to receive image data");
        return NULL;
    }

    // debug
    // FILE *fp = fopen(filename, "wb");
    // if (fp != NULL) {
    //     fwrite(image_data, 1, size, fp);
    //     fclose(fp);
    //     printf("-> Image saved to %s\n", filename);
    // } else {
    //     perror("Failed to open file for writing");
    // }


    return response;
}

// LAUNCH no handler
// RETREIVE no handler

void transmission_codes_handler(int sock, transmission_codes_args* args) {
    for (int i = 0; i < NUM_TRANSMISSION_CODES; i++)
        send(sock, &args->codes[i], sizeof(args->codes[i]), 0);
}

void pos_handler(int sock, pos_args* args) {
    float uav_x = args->a, uav_y = args->b, bot_x = args->c, bot_y = args->d;
    send(sock, &uav_x, sizeof(uav_x), 0);
    send(sock, &uav_y, sizeof(uav_y), 0);
    send(sock, &bot_x, sizeof(bot_x), 0);
    send(sock, &bot_y, sizeof(bot_y), 0);
}

// STOP no handler

void thrust_handler(int sock, thrust_args* args) {
    send(sock, &args->thrust, sizeof(&args->thrust), 0);
}

void thrust_control_mode_handler(int sock, thrust_ctrl_mode_args* args) {
    send(sock, &args->mode, sizeof(&args->mode), 0);
}

// ROLL PITCH and YAW not implemented

void set_height_handler(int sock, set_height_args* args) {
    send(sock, &args->delta_height, sizeof(args->delta_height), 0);
}

void set_x_handler(int sock, set_x_args* args) {
    send(sock, &args->delta_x, sizeof(args->delta_x), 0);
}

void set_y_handler(int sock, set_y_args* args) {
    send(sock, &args->delta_y, sizeof(args->delta_y), 0);
}

void set_pid_handler(int sock, set_pid_args* args) {
    send(sock, &args->pid_idx, sizeof(args->pid_idx), 0);
    send(sock, &args->param_idx, sizeof(args->param_idx), 0);
    send(sock, &args->value, sizeof(args->value), 0);
}

get_pid_response* get_pid_handler(int sock, get_pid_args* args) {
    send(sock, &args->pid_idx, sizeof(args->pid_idx), 0);
    send(sock, &args->param_idx, sizeof(args->param_idx), 0);
    
    get_pid_response* response = malloc(sizeof(get_pid_response));
    if (recv_exact(sock, (u_int8_t*)response, sizeof(*response)) < 0) {
        perror("Failed to receive PID response");
        response = NULL;
    }
    return response;
}

save_pid_response* save_pid_handler(int sock) {
    // Allocate memory for the response (uint8_t success)
    save_pid_response* response = malloc(sizeof(save_pid_response));
    if (response == NULL) return NULL;

    // Wait for the UAV to confirm if the flash/EEPROM save was successful
    if (recv_exact(sock, (uint8_t*)response, sizeof(save_pid_response)) < 0) {
        perror("Failed to receive SAVE_PID response");
        free(response);
        return NULL;
    }
    return response;
}

gyro_calibration_status_response* gyro_calibration_status_handler(int sock) {
    // Allocate memory for the 4-byte status struct
    gyro_calibration_status_response* response = malloc(sizeof(gyro_calibration_status_response));
    if (response == NULL) return NULL;

    // Receive the status bytes (system, gyro, accel, mag)
    if (recv_exact(sock, (uint8_t*)response, sizeof(gyro_calibration_status_response)) < 0) {
        perror("Failed to receive gyro status");
        free(response);
        return NULL;
    }
    return response;
}

get_game_state_response* get_game_state_handler(int sock) {
    get_game_state_response* response = malloc(sizeof(get_game_state_response));
    if (response == NULL) return NULL;

    // Receive the single byte representing the current state
    if (recv_exact(sock, (uint8_t*)response, sizeof(get_game_state_response)) < 0) {
        perror("Failed to receive game state");
        free(response);
        return NULL;
    }
    return response;
}

pos_vel_response* pos_vel_handler(int sock) {
    pos_vel_response* response = malloc(sizeof(pos_vel_response));
    if (recv_exact(sock, (u_int8_t*)response, sizeof(*response)) < 0) {
        perror("Failed to receive position and velocity response");
        free(response);
        return NULL;
    }
    return response;
}

// Delegate command to its respective handler
void* send_command_to_uav(enum Command cmd, void* args) {
    int sock = connect_to_server();
    if (sock < 0) return NULL;

    uint8_t c = cmd;
    if (send(sock, &c, 1, 0) != 1) {
        perror("Failed to send command");
    } else {
        printf("-> Sent command %d to UAV.\n", cmd);
    }

    void* response = NULL;

    switch (cmd) {
    case IMAGE:
        response = (void*)image_handler(sock);
        break;
    case LAUNCH:
    case RETRIEVE:
        break; // no header + args
    case TRANSMISSION_CODES: {
        transmission_codes_args* tc_args = (transmission_codes_args*) args;
        transmission_codes_handler(sock, tc_args);
        free(tc_args);
    } break;
    case POS: {
        pos_args* p_args = (pos_args*) args;
        pos_handler(sock, p_args);
        free(p_args);
    } break;
    case STOP:
        break; // no header + args
    case THRUST: {
        thrust_args* th_args = (thrust_args*) args;
        thrust_handler(sock, th_args);
        free(th_args);
    } break;
    case THRUST_CTRL_MODE: {
        thrust_ctrl_mode_args* th_ctrl_args = (thrust_ctrl_mode_args*) args;
        thrust_handler(sock, args);
        free(th_ctrl_args);
    } break;
    case PITCH:
    case ROLL:
    case YAW:
        break; // not implemented yet
    case SET_HEIGHT: {
        set_height_args* sh_args = (set_height_args*) args;
        set_height_handler(sock, args);
        free(sh_args);
    } break;
    case SET_X: {
        set_x_args* x_args = (set_x_args*) args;
        set_x_handler(sock, args);
        free(x_args);
    } break;
    case SET_Y: {
        set_y_args* y_args = (set_y_args*) args;
        set_y_handler(sock, args);
        free(y_args);
    } break;
    case SET_PID: {
        set_pid_args* spid_args = (set_pid_args*) args;
        set_pid_handler(sock, spid_args);
        free(spid_args);
    } break;
    case GET_PID: {
        get_pid_args* gpid_args = (get_pid_args*) args;
        response = (void*) get_pid_handler(sock, args);
        free(gpid_args);
    } break;
    case SAVE_PID:
        response = (void*)save_pid_handler(sock);
        break;
    case GYRO_CALIBRATION_STATUS:
        response = (void*)gyro_calibration_status_handler(sock);
        break;
    case GET_GAME_STATE:
        response = (void*)get_game_state_handler(sock);
        break;
    case POS_VEL:
        response = (void*)pos_vel_handler(sock);
        break;
    default:
        perror("Invalid command");
        response = NULL;
    }

    close(sock);

    return response;
}

