CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -llgpio

COMMON_SRCS = motor_control.c imu.c patterns.c telemetry.c

autonomous: $(COMMON_SRCS) main.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

telemetry: $(COMMON_SRCS) telemetry_main.c
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

all: autonomous telemetry

clean:
	rm -f autonomous telemetry
