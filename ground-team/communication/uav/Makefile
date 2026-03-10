CC      = gcc
CFLAGS  = -Wall -Wextra -O2

COMMON_SRCS = computer_vision.c uav_client.c
TARGET      = build/

computer-vision: $(COMMON_SRCS) computer_vision.c
	$(CC) $(CFLAGS) -o $@ $^ $(shell python3-config --cflags --ldflags)

all: computer-vision

clean:
	rm -f computer-vision
