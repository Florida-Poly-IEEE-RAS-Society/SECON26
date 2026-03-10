#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>

const char* test_file = "./UAV_Computer_Vision/cool_duck_images/IMG_20251101_181840.jpg";

typedef int pipe_t[2];

// for debug
uint8_t* read_file(const char* filename, uint32_t* out_size) {
    FILE* f = fopen(filename, "rb");
    if (!f) {
        perror("fopen");
        return NULL;
    }

    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        fclose(f);
        return NULL;
    }

    uint8_t* buffer = malloc(size);
    if (!buffer) {
        fclose(f);
        return NULL;
    }

    if (fread(buffer, 1, size, f) != (uint32_t)size) {
        perror("fread");
        free(buffer);
        fclose(f);
        return NULL;
    }

    fclose(f);
    *out_size = size;
    return buffer;
}

void send_image(pipe_t write_pipe, uint8_t* image, uint32_t image_size) {
    write(write_pipe[1], &image_size, sizeof(image_size));
    write(write_pipe[1], image, image_size);

    perror("write");
}

float* receive_coords(pipe_t read_pipe, uint32_t* out_size) {
    read(read_pipe[0], out_size, sizeof(*out_size));

    unsigned long buf_size = sizeof(float)*(unsigned long)*out_size;
    printf("%u\n", *out_size);
    float* buf = malloc(buf_size);
    read(read_pipe[0], buf, buf_size);

    return buf;
}

float* get_computer_vision_result(pipe_t read_pipe, pipe_t write_pipe, uint8_t* image, uint32_t image_size, uint32_t* out_size) {
    if (!read_pipe || !write_pipe) {
        perror("pipe failed");
        return NULL;
    }

    send_image(write_pipe, image, image_size);
    float* coords = receive_coords(read_pipe, out_size);

    return coords;
}

void create_pipes(const char* command, pipe_t read_pipe, pipe_t write_pipe){
    perror("pipe creation");
    pid_t pid = fork();
    if (pid == 0) {
        dup2(write_pipe[0], STDIN_FILENO);
        dup2(read_pipe[1], STDOUT_FILENO);

        close(write_pipe[1]);
        close(read_pipe[0]);

        execlp("source", "source", "./venv/bin/activate", NULL);
        execlp("python3", "python3", "computer_vision.py", command, NULL);
        perror("exec failed");
        exit(1);
    }

    close(write_pipe[0]);
    close(read_pipe[1]);
}

float* get_duck_points(uint8_t* image, uint32_t image_size, uint32_t* out_size) {
    pipe_t read_pipe;
    pipe_t write_pipe;
    pipe(read_pipe);
    pipe(write_pipe);
    create_pipes("duck_points", read_pipe, write_pipe);

    float* coords = get_computer_vision_result(read_pipe, write_pipe, image, image_size, out_size);
    close(write_pipe[1]);
    close(read_pipe[0]);

    return coords;
}

float* get_field_corners(uint8_t* image, uint32_t image_size, uint32_t* out_size) {
    pipe_t read_pipe;
    pipe_t write_pipe;
    pipe(read_pipe);
    pipe(write_pipe);
    create_pipes("field_corners", read_pipe, write_pipe);

    float* coords = get_computer_vision_result(read_pipe, write_pipe, image, image_size, out_size);
    close(write_pipe[1]);
    close(read_pipe[0]);

    return coords;
}

int test() {
    uint32_t* image_size = malloc(sizeof(uint32_t));
    printf("Reading test file\n");
    uint8_t* image = read_file(test_file, image_size);

    printf("Getting duck points\n");
    uint32_t* output_size = malloc(sizeof(uint32_t));
    float* output = get_field_corners(image, *image_size, output_size);
    if (output == NULL) {
        perror("lmao skill issue");
    }

    printf("Output:");
    for (uint32_t i = 0; i < *output_size; i++) {
        printf(" %f", output[i]);
    }
    printf("\n");

    free(image_size);
    free(image);
    free(output_size);
    free(output);

    return 0;

}

//
// int main() {
//     return test();
// }
