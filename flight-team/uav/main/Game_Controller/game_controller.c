#include "game_controller.h"

#include <Flight_Controller/flight_controller.h>

#include "freertos/FreeRTOS.h"

#define IR_GPIO 8

static enum Game_State _current_state = Game_Waiting;
static IRtx_t _ir;
static ir_nec_scan_code_t _ir_codes[4] = {0};

static struct {
    float uav_x;
    float uav_y;
    float bot_x;
    float bot_y;
    bool prev;
    bool curr;
} _pos = {0};


bool game_state_change_maybe(enum Game_State new_state) {
    if (_current_state == Game_Waiting) {
        _current_state = new_state;
        return true;
    } else {
        // ignore request if busy
        return false;
    }
}

void game_set_ir_codes(ir_nec_scan_code_t codes[4]) {
    for (int i = 0; i < 4; ++i) {
        _ir_codes[i] = codes[i];
    }
}

void game_set_pos_data(float uav_x, float uav_y, float bot_x, float bot_y) {
    _pos.uav_x = uav_x;
    _pos.uav_y = uav_y;
    _pos.bot_x = bot_x;
    _pos.bot_y = bot_y;
    _pos.curr = true;
    _pos.prev = false;
}

static void launch(void) {
    reset_height(0.0f);
    change_height_by(15.0f);
    while (!at_desired_position()) {
        vTaskDelay(125 / portTICK_PERIOD_MS);
    }

    reset_pos(0.0f, 0.0f);
    change_pos_by(10.6f, 10.6f);
    while (!at_desired_position()) {
        vTaskDelay(125 / portTICK_PERIOD_MS);
    }
}

static void send_codes(void) {
    rotate_by(360.0f);
    while (!at_desired_position()) {
        for (int i = 0; i < 4; ++i) IRtx_transmit(&_ir, _ir_codes[i]);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void retrieve(void) {
    do {
        if (_pos.prev == _pos.curr) continue;
        _pos.prev = _pos.curr = false;

        float dx = _pos.bot_x - _pos.uav_x;
        float dy = _pos.bot_y - _pos.uav_y;

        reset_pos(_pos.uav_x, _pos.uav_y);
        change_pos_by(dx, dy);
        
        vTaskDelay(50 / portTICK_PERIOD_MS); // what number is good?
    } while (!at_desired_position());

    return_to_last_height();
    while (!at_desired_position()) {
        vTaskDelay(125 / portTICK_PERIOD_MS);
    }
}

static void game_task(void*) {
    while (true) {
        switch (_current_state) {
        case Game_Waiting:
            vTaskDelay(500 / portTICK_PERIOD_MS);
            break;
        case Game_Launch:
            launch();
            break;
        case Game_Send_Codes:
            send_codes();
            break;
        case Game_Retrieve:
            retrieve();
            break;
        }

        _current_state = Game_Waiting;
    }
}

void game_controller_init(void) {
    IRtx_init(&_ir, IR_GPIO);
    xTaskCreate(game_task, "game_thread", 4096, NULL, 5, NULL);
}
