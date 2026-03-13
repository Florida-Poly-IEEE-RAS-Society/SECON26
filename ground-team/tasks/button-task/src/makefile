CC      = gcc
CFLAGS  = -Wall -Wextra -O2
LDFLAGS = -llgpio -lm

SRCS = button_task.c
button-task-start: $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	rm -f button-task-start
