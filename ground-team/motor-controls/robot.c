#include "robot.h"
#include <time.h>

static float _now_s(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (float)t.tv_sec + (float)t.tv_nsec * 1e-9f;
}

status_t robot_init(robot_t *bot) {
    int handle = lgGpiochipOpen(0);
    if (handle < 0) return ERR_BUS_FAIL;

    bot->gpio_handle = handle;
    bot->pos         = (pos2_t){0.0f, 0.0f, 0.0f};

    bot->m1.handle = handle; bot->m1.in1 = MOTOR1_IN1; bot->m1.in2 = MOTOR1_IN2;
    bot->m2.handle = handle; bot->m2.in1 = MOTOR2_IN1; bot->m2.in2 = MOTOR2_IN2;

    int motor_pins[] = { MOTOR1_IN1, MOTOR1_IN2, MOTOR2_IN1, MOTOR2_IN2 };
    for (int i = 0; i < 4; i++) {
        if (lgGpioClaimOutput(handle, 0, motor_pins[i], 0) < 0) {
            lgGpiochipClose(handle);
            return ERR_WRITE_FAIL;
        }
    }

    status_t rc;
    rc = enc_init(handle, &bot->enc1, ENC1_PIN_A, ENC1_PIN_B);
    if (rc != OK) { lgGpiochipClose(handle); return rc; }

    rc = enc_init(handle, &bot->enc2, ENC2_PIN_A, ENC2_PIN_B);
    if (rc != OK) { lgGpiochipClose(handle); return rc; }

    rc = imu_init(&bot->imu);
    if (rc != OK) { lgGpiochipClose(handle); return rc; }

    if (imu_load_cal(&bot->imu) != OK) {
        rc = imu_calibrate(&bot->imu);
        if (rc != OK) { lgGpiochipClose(handle); return rc; }
    }

    return OK;
}

status_t robot_drive(robot_t *bot, float meters) {
    if (meters == 0.0f) return OK;

    motordir_t dir = (meters > 0.0f) ? MOTOR_FORWARD : MOTOR_BACKWARD;
    float target   = (meters < 0.0f) ? -meters : meters;

    enc_reset(&bot->enc1);
    enc_reset(&bot->enc2);

    float heading = 0.0f;
    float prev_t  = _now_s();

    motor_set(&bot->m1, dir);
    motor_set(&bot->m2, dir);

    while (1) {
        float dl = enc_get_dist_m(&bot->enc1);
        float dr = enc_get_dist_m(&bot->enc2);

        if ((dl + dr) * 0.5f >= target) break;

        float now = _now_s();
        float dt  = now - prev_t;
        prev_t    = now;

        float gz;
        if (imu_read_gyro_z(&bot->imu, &gz) == OK)
            heading = lowpass(heading, heading + gz * dt, 0.8f);

        if (heading > DRIVE_CORRECTION_DEG) {
            motor_set(&bot->m1, MOTOR_STOP);
            motor_set(&bot->m2, dir);
        } else if (heading < -DRIVE_CORRECTION_DEG) {
            motor_set(&bot->m1, dir);
            motor_set(&bot->m2, MOTOR_STOP);
        } else {
            motor_set(&bot->m1, dir);
            motor_set(&bot->m2, dir);
        }

        usleep(DRIVE_LOOP_US);
    }

    motor_set(&bot->m1, MOTOR_STOP);
    motor_set(&bot->m2, MOTOR_STOP);

    float dl = enc_get_dist_m(&bot->enc1);
    float dr = enc_get_dist_m(&bot->enc2);
    if (dir == MOTOR_BACKWARD) { dl = -dl; dr = -dr; }
    odom_update(&bot->pos, dl, dr);

    return OK;
}

status_t robot_turn(robot_t *bot, float degrees) {
    if (degrees == 0.0f) return OK;

    int   clockwise = (degrees > 0.0f);
    float target    = (degrees < 0.0f) ? -degrees : degrees;

    if (clockwise) {
        motor_set(&bot->m1, MOTOR_FORWARD);
        motor_set(&bot->m2, MOTOR_BACKWARD);
    } else {
        motor_set(&bot->m1, MOTOR_BACKWARD);
        motor_set(&bot->m2, MOTOR_FORWARD);
    }

    float turned = 0.0f;
    float prev_t = _now_s();

    while (turned < target) {
        float now = _now_s();
        float dt  = now - prev_t;
        prev_t    = now;

        float gz;
        if (imu_read_gyro_z(&bot->imu, &gz) != OK) break;
        turned += (gz < 0.0f ? -gz : gz) * dt;

        usleep(DRIVE_LOOP_US);
    }

    motor_set(&bot->m1, MOTOR_STOP);
    motor_set(&bot->m2, MOTOR_STOP);

    float turned_rad = to_rad(clockwise ? turned : -turned);
    odom_set_heading(&bot->pos, bot->pos.heading + turned_rad);

    return OK;
}

status_t robot_swing(robot_t *bot, float degrees, int pivot_left) {
    if (degrees == 0.0f) return OK;

    int   clockwise = (degrees > 0.0f);
    float target    = (degrees < 0.0f) ? -degrees : degrees;

    if (pivot_left) {
        motor_set(&bot->m1, MOTOR_STOP);
        motor_set(&bot->m2, clockwise ? MOTOR_FORWARD : MOTOR_BACKWARD);
    } else {
        motor_set(&bot->m1, clockwise ? MOTOR_FORWARD : MOTOR_BACKWARD);
        motor_set(&bot->m2, MOTOR_STOP);
    }

    float turned = 0.0f;
    float prev_t = _now_s();

    while (turned < target) {
        float now = _now_s();
        float dt  = now - prev_t;
        prev_t    = now;

        float gz;
        if (imu_read_gyro_z(&bot->imu, &gz) != OK) break;
        turned += fabsf(gz) * dt;

        usleep(DRIVE_LOOP_US);
    }

    motors_stop(&bot->m1, &bot->m2);

    float turned_rad = to_rad(clockwise ? turned : -turned);
    odom_set_heading(&bot->pos, bot->pos.heading + turned_rad);

    return OK;
}

void robot_cleanup(robot_t *bot) {
    motors_cleanup(&bot->m1, &bot->m2);
    imu_cleanup(&bot->imu);
}
