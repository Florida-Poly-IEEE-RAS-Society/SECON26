#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include "include/uav_client.h"

#define SERVER_IP "127.0.0.1" // Change to "192.168.4.1" for the actual drone
#define SERVER_PORT 3333      // Matched with PORT 3333 from server.c


// Game states based on the provided game plan
// enum GameState {
//     STATE_PRE_START,
//     STATE_TASK_0_START,
//     STATE_TASK_1_LEAVE_START,
//     STATE_TASK_2_PLANT_FLAG,
//     STATE_TASK_3_UAV_LAUNCH,
//     STATE_TASK_4_ANTENNA_1,
//     STATE_TASK_5_ANTENNA_2,
//     STATE_TASK_6_ANTENNA_3,
//     STATE_TASK_7_CRATER,
//     STATE_TASK_8_ANTENNA_4,
//     STATE_TASK_9_SATELLITE_CMDS,
//     STATE_TASK_10_COLLECT_DUCKS,
//     STATE_TASK_11_RETRIEVE_UAV,
//     STATE_TASK_12_RETREAT,
//     STATE_DONE
// };

// IR NEC Scan Code struct matching ESP-IDF's definition
typedef struct {
    uint16_t address;
    uint16_t command;
} ir_nec_scan_code_t;

// Function to receive exactly 'amount' bytes
int recv_exact(int sock, unsigned char *buffer, size_t amount) {
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

// Sends a simple 1-byte command to the UAV
void send_simple_command(enum Command cmd) {
    int sock = connect_to_server();
    if (sock < 0) return;

    unsigned char c = cmd;
    if (send(sock, &c, 1, 0) != 1) {
        perror("Failed to send command");
    } else {
        printf("-> Sent command %d to UAV.\n", cmd);
    }
    close(sock);
}

// Requests an image from the UAV and saves it to a file
void request_image(const char* filename) {
    int sock = connect_to_server();
    if (sock < 0) return;

    unsigned char cmd = IMAGE;
    if (send(sock, &cmd, 1, 0) != 1) {
        perror("Failed to send IMAGE command");
        close(sock);
        return;
    }

    unsigned char size_buffer[4];
    if (recv_exact(sock, size_buffer, 4) < 0) {
        perror("Failed to receive image size");
        close(sock);
        return;
    }

    uint32_t size = 0;
    for (int i = 0; i < 4; i++) {
        size |= ((uint32_t)size_buffer[4 - i - 1]) << (i * 8);
    }

    printf("-> Image size reported by UAV: %u bytes\n", size);

    if (size == 0) {
        printf("-> No image data received.\n");
        close(sock);
        return;
    }

    unsigned char *image_data = (unsigned char *)malloc(size);
    if (image_data == NULL) {
        perror("Memory allocation failed");
        close(sock);
        return;
    }

    if (recv_exact(sock, image_data, size) < 0) {
        perror("Failed to receive image data");
        free(image_data);
        close(sock);
        return;
    }

    FILE *fp = fopen(filename, "wb");
    if (fp != NULL) {
        fwrite(image_data, 1, size, fp);
        fclose(fp);
        printf("-> Image saved to %s\n", filename);
    } else {
        perror("Failed to open file for writing");
    }

    free(image_data);
    close(sock);
}

get_pid_response get_pid_handler(int sock, get_pid_args* args) {
    send(sock, &args->arg1, sizeof(args->arg1), 0)
    get_pid_response response;
    if (recv_exact(sock, (unsigned char*)&response, sizeof(response)) < 0) {
        perror("Failed to receive PID response");
        response.value = -1; // Indicate error
    }
    return response;
}

// Sends 4 IR transmission codes to the UAV
void send_transmission_codes(ir_nec_scan_code_t codes[4]) {
    int sock = connect_to_server();
    if (sock < 0) return;

    unsigned char cmd = TRANSMISSION_CODES;
    if (send(sock, &cmd, 1, 0) != 1) {
        perror("Failed to send TRANSMISSION_CODES command");
        close(sock);
        return;
    }

    if (send(sock, codes, sizeof(ir_nec_scan_code_t) * 4, 0) != sizeof(ir_nec_scan_code_t) * 4) {
        perror("Failed to send codes data");
    } else {
        printf("-> Sent transmission codes to UAV.\n");
    }

    close(sock);
}

int main() {
    // enum GameState current_state = STATE_PRE_START;

    // printf("=== Starting Ground Bot Game Loop ===\n\n");

    // while (current_state != STATE_DONE) {
    //     switch (current_state) {
    //         case STATE_PRE_START:
    //             printf("[Pre-Start] Bots are placed. Ground bot and UAV turned on.\n");
    //             sleep(2);
    //             current_state = STATE_TASK_0_START;
    //             break;

    //         case STATE_TASK_0_START:
    //             printf("[Task 0] Waiting for LED Start Bar...\n");
    //             sleep(1); // Simulate waiting for LED
    //             printf("[Task 0] LED Start Bar detected. Moving to next state.\n");
    //             current_state = STATE_TASK_1_LEAVE_START;
    //             break;

    //         case STATE_TASK_1_LEAVE_START:
    //             printf("[Task 1] Ground bot moving forward to leave starting area.\n");
    //             sleep(2); // Simulate movement
    //             current_state = STATE_TASK_2_PLANT_FLAG;
    //             break;

    //         case STATE_TASK_2_PLANT_FLAG:
    //             printf("[Task 2] Ground bot rotating and planting the flag.\n");
    //             sleep(2); // Simulate planting
    //             current_state = STATE_TASK_3_UAV_LAUNCH;
    //             break;

    //         case STATE_TASK_3_UAV_LAUNCH:
    //             printf("[Task 3] Sending LAUNCH command to UAV.\n");
    //             send_simple_command(LAUNCH);
    //             sleep(2); // Wait for UAV to launch and hover
    //             current_state = STATE_TASK_4_ANTENNA_1;
    //             break;

    //         case STATE_TASK_4_ANTENNA_1:
    //             printf("[Task 4] Moving to antenna #1 and pressing button 3 times.\n");
    //             sleep(2);
    //             current_state = STATE_TASK_5_ANTENNA_2;
    //             break;

    //         case STATE_TASK_5_ANTENNA_2:
    //             printf("[Task 5] Moving to antenna #2 and turning crank 540 degrees.\n");
    //             sleep(2);
    //             current_state = STATE_TASK_6_ANTENNA_3;
    //             break;

    //         case STATE_TASK_6_ANTENNA_3:
    //             printf("[Task 6] Moving to antenna #3 and pushing astro-duck off pressure plate.\n");
    //             sleep(2);
    //             current_state = STATE_TASK_7_CRATER;
    //             break;

    //         case STATE_TASK_7_CRATER:
    //             printf("[Task 7] Entering crater and completing one full lap.\n");
    //             sleep(3);
    //             current_state = STATE_TASK_8_ANTENNA_4;
    //             break;

    //         case STATE_TASK_8_ANTENNA_4:
    //             printf("[Task 8] Moving to antenna #4 and entering code '73738#'.\n");
    //             sleep(2);
    //             current_state = STATE_TASK_9_SATELLITE_CMDS;
    //             break;

    //         case STATE_TASK_9_SATELLITE_CMDS:
    //             printf("[Task 9] Requesting image from UAV to extract antenna colors.\n");
    //             request_image("antenna_colors.jpg");
                
    //             // Simulate extracting colors and generating IR codes
    //             printf("[Task 9] Extracting colors and sending satellite commands to UAV.\n");
    //             ir_nec_scan_code_t codes[4] = {
    //                 {0x00FF, 0x00FF}, {0x00FF, 0x00FF}, 
    //                 {0x00FF, 0x00FF}, {0x00FF, 0x00FF}
    //             };
    //             send_transmission_codes(codes);
    //             sleep(2);
    //             current_state = STATE_TASK_10_COLLECT_DUCKS;
    //             break;

    //         case STATE_TASK_10_COLLECT_DUCKS:
    //             printf("[Task 10] Requesting image from UAV for astro-duck locations.\n");
    //             request_image("astro_ducks.jpg");
    //             printf("[Task 10] Moving astro-ducks to lunar landing area.\n");
    //             sleep(3);
    //             current_state = STATE_TASK_11_RETRIEVE_UAV;
    //             break;

    //         case STATE_TASK_11_RETRIEVE_UAV:
    //             printf("[Task 11] Sending RETRIEVE command to UAV.\n");
    //             send_simple_command(RETRIEVE);
    //             sleep(3); // Wait for UAV to land
    //             current_state = STATE_TASK_12_RETREAT;
    //             break;

    //         case STATE_TASK_12_RETREAT:
    //             printf("[Task 12] Retreating back to starting area.\n");
    //             sleep(2);
    //             current_state = STATE_DONE;
    //             break;

    //         default:
    //             break;
    //     }
    //     printf("\n");
    // }

    // printf("=== Game plan completed successfully! ===\n");
    return 0;
}