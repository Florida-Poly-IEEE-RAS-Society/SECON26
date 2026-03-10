from client import DebugClient
import UAV_Computer_Vision.field_detection as fd

from PIL import Image
import logging
import numpy as np
import cv2 as cv
from pathlib import Path
import sys
import struct


cv_module_path = Path("./UAV_Computer_Vision/")
sys.path.insert(1, cv_module_path)
output_image_file = Path("./output.jpg")

VALUE_CHANGE_STEP = 0.06


"""
helper functions
"""


def downscale_image(image: cv.UMat) -> cv.UMat:
    img = Image.fromarray(image)
    new_width = img.size[0] // 3
    new_height = img.size[1] // 3
    img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
    return np.array(img)


def receive_image(client: DebugClient):
    logging.debug("getting image")
    size = int.from_bytes(client.receive_n_bytes(4))
    payload = client.receive_n_bytes(size)

    image = np.frombuffer(payload, dtype=np.uint8)
    image_decoded = cv.imdecode(image, cv.IMREAD_COLOR)

    return image_decoded


"""
handlers
"""


def handle_camera(client: DebugClient):
    opencv_image = receive_image(client)
    image_downscaled = downscale_image(opencv_image)

    computer_vision_result = fd.recompute_display(
        image_downscaled, opencv_image)
    cv.imwrite(output_image_file, computer_vision_result)

    logging.info(f"Image saved to {output_image_file}")

    duck_points = fd.get_duck_points(opencv_image)
    field_corners = fd.get_field_corner_points(opencv_image)

    logging.info(f"Duck points: {duck_points}\n Field corners: {field_corners}")


def handle_launch(client: DebugClient):
    # no data sent from server here to handle
    pass


def handle_retreive(client: DebugClient):
    # no data sent from server here to handle
    pass


def handle_transmission_codes(client: DebugClient, args):
    payload = b''
    for i in range(4):
        address_i = int(args.get(f"address_{i+1}"))
        command_i = int(args.get(f"command_{i+1}"))

        payload += struct.pack("<hh", address_i, command_i)

    client.send_payload(payload)


def handle_pos(client: DebugClient, args):
    uav_x = float(args.get("uav_x"))
    uav_y = float(args.get("uav_y"))
    bot_x = float(args.get("bot_x"))
    bot_y = float(args.get("bot_y"))

    payload = struct.pack("<ffff", uav_x, uav_y, bot_x, bot_y)

    client.send_payload(payload)


def handle_stop(client: DebugClient):
    # no data sent from server here to handle
    pass


def handle_thrust(client: DebugClient, args):
    thrust = float(args.get("thrust"))

    # apparently its all little endian so I gotta do this ugly
    # piece of shit function everytime I encode into bytes :(
    client.send_payload(struct.pack("<f", thrust))


def handle_thrust_control_mode(client: DebugClient, args):
    thrust_control_mode = int(
        args.get("thrust_control_mode")).to_bytes(1, 'little')

    client.send_payload(thrust_control_mode)


def handle_pitch(client: DebugClient, args):
    pitch = float(args.get("pitch"))

    client.send_payload(struct.pack("<f", pitch))


def handle_roll(client: DebugClient, args):
    roll = float(args.get("roll"))

    client.send_payload(struct.pack("<f", roll))


def handle_yaw(client: DebugClient, args):
    yaw = float(args.get("yaw"))

    client.send_payload(struct.pack("<f", yaw))


def handle_set_height(client: DebugClient, args):
    height = float(args.get("height"))

    client.send_payload(struct.pack("<f", height))


def handle_set_x(client: DebugClient, args):
    x = float(args.get("x"))

    client.send_payload(struct.pack("<f", x))


def handle_set_y(client: DebugClient, args):
    y = float(args.get("y"))

    client.send_payload(struct.pack("<f", y))


def handle_set_pid(client: DebugClient, args):
    pid_idx = int(args.get("pid_idx")).to_bytes(1, 'little')
    param_idx = int(args.get("param_idx")).to_bytes(1, 'little')
    value = float(args.get("value"))

    # chars because bytes have weird behavior in structs
    payload = struct.pack("<ccf", pid_idx, param_idx, value)

    client.send_payload(payload)


def handle_get_pid(client: DebugClient, args):
    pid_idx = int(args.get("pid_idx")).to_bytes(1, 'little')
    param_idx = int(args.get("param_idx")).to_bytes(1, 'little')

    payload = struct.pack("<cc", pid_idx, param_idx)

    client.send_payload(payload)

    pid = struct.unpack('<f', client.receive_n_bytes(4))

    logging.info(f"Current PID value : {pid}")


def handle_save_pid(client: DebugClient):
    result = int.from_bytes(client.receive_n_bytes(1), 'little')

    if result == 1:
        logging.info("PID saved successfully")
    elif result == 0:
        logging.info("PID not saved successfully... but why?")
    else:
        logging.error("SOMETHING IS WRONG")


def handle_gyro_calibration_status(client: DebugClient):
    system = int.from_bytes(client.receive_n_bytes(1), 'little')
    gyro = int.from_bytes(client.receive_n_bytes(1), 'little')
    accel = int.from_bytes(client.receive_n_bytes(1), 'little')
    mag_calibration_status = int.from_bytes(
        client.receive_n_bytes(1), 'little')

    logging.info(f"Gyro Calibration Status: System: {system}, Gyro: {
                 gyro}, Acceleration: {accel}, Magnet Calibration Status {mag_calibration_status}")


def handle_get_game_state(client: DebugClient):
    game_state = int.from_bytes(client.receive_n_bytes(1), 'little')

    logging.info(f"Current game state: {game_state}")


def handle_pos_vel(client: DebugClient):
    x_pos, y_pos, z_pos, x_vel, y_vel, z_vel = struct.unpack(
        "<ffffff", client.receive_n_bytes(24))

    logging.info(f"Pos Vel: pos: [{x_pos}, {y_pos}, {
                 z_pos}], vel: [{x_vel}, {y_vel}, {z_vel}]")


def handle_gyro_angle(client: DebugClient):
    roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate, acc_x, acc_y, acc_z = struct.unpack("<fffffffff", client.receive_n_bytes(36))

    logging.info(f"Pitch, Yaw, Roll: ({roll}, {pitch}, {yaw})\nRoll, Pitch, Yaw (Rate): ({roll_rate}, {pitch_rate}, {yaw_rate})\nAcc X, Acc Y, Acc Z: ({acc_x}, {acc_y}, {acc_z})")
