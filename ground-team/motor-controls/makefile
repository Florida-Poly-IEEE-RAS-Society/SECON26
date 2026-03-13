CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -llgpio -lm

SRCS = main.c      \
       robot.c     \
       encoder.c   \
       motor_control.c \
       imu.c

robot: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f robot
