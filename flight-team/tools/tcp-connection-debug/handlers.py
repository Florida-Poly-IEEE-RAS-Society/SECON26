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


"""
handlers
"""


def handle_camera(client: DebugClient):
    logging.debug("getting image")
    size = int.from_bytes(client.receive_n_bytes(4))
    payload = client.receive_n_bytes(size)

    image = np.frombuffer(payload, dtype=np.uint8)
    image_decoded = cv.imdecode(image, cv.IMREAD_COLOR)
    image_downscaled = downscale_image(image_decoded)

    computer_vision_result = fd.recompute_display(
        image_downscaled, image_decoded)
    cv.imwrite(output_image_file, computer_vision_result)

    logging.info(f"Image saved to {output_image_file}")


def handle_stop(client: DebugClient):
    # no data sent from server here to handle
    pass


def handle_thrust(client: DebugClient, args):
    thrust = float(args.get("thrust"))

    # apparently its all big endian so I gotta do this ugly
    # piece of shit function everytime I encode into bytes :(
    client.send_payload(struct.pack("<f", thrust))


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
    pid_idx = int(args.get("pid_idx")).to_bytes(1, 'big')
    param_idx = int(args.get("param_idx")).to_bytes(1, 'big')
    value = float(args.get("value"))

    # chars because bytes have weird behavior in structs
    payload = struct.pack("<ccf", pid_idx, param_idx, value)

    client.send_payload(payload)


def handle_get_pid(client: DebugClient):
    pid = struct.unpack('<f', client.receive_n_bytes(4))

    logging.info(f"Current PID value : {pid}")


def handle_gyro_calibration_status(client: DebugClient):
    system = int.from_bytes(client.receive_n_bytes(1), 'big')
    gyro = int.from_bytes(client.receive_n_bytes(1), 'big')
    accel = int.from_bytes(client.receive_n_bytes(1), 'big')
    mag_calibration_status = int.from_bytes(client.receive_n_bytes(1), 'big')

    logging.info(f"Gyro Calibration Status: System: {system}, Gyro: {
                 gyro}, Acceleration: {accel}, Magnet Calibration Status {mag_calibration_status}")
