CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -llgpio

TARGET = motors
SRCS   = main.c motor_control.c imu.c patterns.c

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f $(TARGET)
