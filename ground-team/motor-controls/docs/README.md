# Motor Controller

Import the header robot.h

I made it very easy to control the bot with basic functions. You can use the following functions to control the bot:

```c
static inline status_t forward(robot_t *bot, float meters)       { return robot_drive(bot,  meters); }
static inline status_t backward(robot_t *bot, float meters)      { return robot_drive(bot, -meters); }
static inline status_t right(robot_t *bot)                       { return robot_turn(bot,   90.0f);  }
static inline status_t left(robot_t *bot)                        { return robot_turn(bot,  -90.0f);  }
static inline status_t u_turn(robot_t *bot)                      { return robot_turn(bot,  180.0f);  }
static inline status_t swing_right(robot_t *bot, float degrees)  { return robot_swing(bot,  degrees, 0); }
static inline status_t swing_left(robot_t *bot, float degrees)   { return robot_swing(bot, -degrees, 1); }
```

if you want to turn a specific angle, you can use the `robot_turn` function directly. For example, to turn 45 degrees to the right, you can call:

```c
robot_turn(bot, 45.0f);
```

if you just want more dynamic control, use:

```c
robot_drive(bot, angle);
robot_turn(bot, angle);
robot_swing(bot, angle, int32_t: left ? 1 : 0); // if you want to swing left put in 1 if you want to swing right put in 0
```
