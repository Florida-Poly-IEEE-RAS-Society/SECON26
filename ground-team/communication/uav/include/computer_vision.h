#include <cstdint>
#include <stdint.h>

typedef struct {
    float x;
    float y;
} duck_point;

typedef struct {
    float x;
    float y;
} field_corner_point;

// in order of { top left, bottom right, bottom left, top right }
typedef field_corner_point field_corners[4];

// use these for now, may be annoying
float* get_duck_points(uint8_t* image, uint32_t image_size, uint32_t* out_size);
float* get_field_corners(uint8_t* image, uint32_t image_size, uint32_t* out_size);

// not implemented yet
field_corners* tcp_get_field_corners(int sock);
duck_point* tcp_get_duck_points(int sock, uint32_t* out_size);

