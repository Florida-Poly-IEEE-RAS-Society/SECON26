#include "robot.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

static robot_t bot;

static void cleanup(int sig) {
  (void)sig;
  robot_cleanup(&bot);
  _exit(0);
}

#define CHECK(fn, label)                                                       \
  do {                                                                         \
    status_t _rc = (fn);                                                       \
    if (_rc != OK) {                                                           \
      fprintf(stderr, label " failed: %d\n", _rc);                             \
      robot_cleanup(&bot);                                                     \
      return 1;                                                                \
    }                                                                          \
  } while (0)

int main(void) {
  signal(SIGINT, cleanup);

  CHECK(robot_init(&bot), "Robot init");
  CHECK(forward(&bot, 10.0f), "Drive to crank");
  CHECK(right(&bot), "Turn to crank");
  CHECK(left(&bot), "Turn after crank");
  CHECK(forward(&bot, 5.0f), "Drive to keypad");

  robot_cleanup(&bot);
  return 0;
}
