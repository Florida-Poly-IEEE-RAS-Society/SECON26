#include <IR/ir.h>
#include <stdbool.h>

enum Game_State {
    Game_Waiting,
    Game_Launch,
    Game_Send_Codes,
    Game_Retrieve,
    Game_Calibrate,
};

bool game_state_change_maybe(enum Game_State new_state);
enum Game_State game_get_state(void);
void game_set_ir_codes(ir_nec_scan_code_t codes[4]);
void game_set_pos_data(float uav_x, float uav_y, float bot_x, float bot_y);
void game_controller_init(void);
