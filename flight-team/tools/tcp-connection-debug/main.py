import cli
from client import MessageType, DebugClient
import handlers

import logging


logging.basicConfig(
    # filename=Path("./log.log"),
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)


HOST = '192.168.4.1'
PORT = 3333

"""
format for registering command:
name: [
    handler,
    help message,
    arg list,
    message type (can be none)
]
found out about argparsing library that couldve made this a whole
lot cleaner and easier to program but we dont make good decisions here
"""
cli_commands = {
    "camera": [
        handlers.handle_camera,
        "Pulls camera data from server, applies colormask and outputs to output.jpg",
        [],
        MessageType.CAMERA_DATA
    ],
    "launch": [
        handlers.handle_launch,
        "Launches Drone",
        [],
        MessageType.LAUNCH
    ],
    "retreive": [
        handlers.handle_retreive,
        "Sends the retreive command",
        [],
        MessageType.RETREIVE
    ],
    "transmission_codes": [
        handlers.handle_transmission_codes,
        "Sends transmission codes to uav",
        ["address_1", "command_1", "address_2", "command_2",
            "address_3", "command_3", "address_4", "command_4"],
        MessageType.TRANSMISSION_CODES
    ],
    "pos": [
        handlers.handle_pos,
        "Sets the relative position of the uav to ground bot",
        ["uav_x", "uav_y", "bot_x", "bot_y"],
        MessageType.POS
    ],
    "stop": [
        handlers.handle_stop,
        "Emergency stops the drone",
        [],
        MessageType.STOP
    ],
    "t": [
        handlers.handle_thrust,
        "Sets the thrust",
        ["thrust"],
        MessageType.THRUST
    ],
    "thrust_control_mode": [
        handlers.handle_thrust_control_mode,
        "Sets the thrust control mode",
        ["thrust_control_mode"],
        MessageType.THRUST_CONTROL_MODE
    ],
    "pitch": [
        handlers.handle_pitch,
        "Sets the pitch",
        ["pitch"],
        MessageType.PITCH
    ],
    "roll": [
        handlers.handle_roll,
        "Sets the roll",
        ["roll"],
        MessageType.ROLL
    ],
    "yaw": [
        handlers.handle_yaw,
        "Sets the yaw",
        ["yaw"],
        MessageType.YAW
    ],
    "set_height": [
        handlers.handle_set_height,
        "Sets the height delta of the drone",
        ["height"],
        MessageType.SET_HEIGHT
    ],
    "set_x": [
        handlers.handle_set_x,
        "Sets the relative x coordinate of the drone",
        ["x"],
        MessageType.SET_X
    ],
    "set_y": [
        handlers.handle_set_y,
        "Sets the relative y coordinate of the drone",
        ["y"],
        MessageType.SET_Y
    ],
    "set_pid": [
        handlers.handle_set_pid,
        "Sets the pid values of the drone",
        ["pid_idx", "param_idx", "value"],
        MessageType.SET_PID
    ],
    "get_pid": [
        handlers.handle_get_pid,
        "Returns current PID value",
        ["pid_idx", "param_idx"],
        MessageType.GET_PID
    ],
    "save_pid": [
        handlers.handle_save_pid,
        "Saves PID on uav",
        [],
        MessageType.SAVE_PID
    ],
    "g": [
        handlers.handle_gyro_calibration_status,
        "Returns calibration status of the gyro",
        [],
        MessageType.GYRO_CALIBRATION_STATUS
    ],
    "get_game_state": [
        handlers.handle_get_game_state,
        "Returns the current game state",
        [],
        MessageType.GET_GAME_STATE
    ],
    "pos_vel": [
        handlers.handle_pos_vel,
        "Returns the position and velocity of uav",
        [],
        MessageType.POS_VEL
    ],
    "ga": [
        handlers.handle_gyro_angle,
        "Returns the absolute angular position of the gyroscope",
        [],
        MessageType.GYRO_ANGLE
    ],
}


def register_cli_commands(client: DebugClient):
    for k, v in cli_commands.items():
        # dont register client if no msg_type
        if v[3] is None:
            cli.regiser_command(k, v[0], v[1], v[2])
            continue

        cli.regiser_command(k, v[0], v[1], v[2], client, v[3])


if __name__ == '__main__':
    client = DebugClient(HOST, PORT)

    register_cli_commands(client)

    cli.start()
