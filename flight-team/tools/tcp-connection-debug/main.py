import os
import time
import UAV_Computer_Vision.field_detection as fd
import cv2 as cv
import io
from PIL import Image
import numpy as np
from pathlib import Path
import socket
from enum import Enum
import logging
import sys

cv_module_path = Path("./UAV_Computer_Vision/")
sys.path.insert(1, cv_module_path)

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

output_image_file = Path("./output.jpg")

REQUEST_DELAY = 2000


class MessageType(Enum):
    CAMERA_DATA = 1
    LAUNCH = 2
    RETRIEVE = 3
    TRANSMISSION_CODES = 4
    POS = 5
    STOP = 6


class DebugClient:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = None
        self.handlers = {}

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port))
        logging.debug(f"Connected to {self.host}:{self.port}")

    def close(self):
        if not self.sock:
            return

        self.sock.close()
        logging.debug(f"Connection to {self.host}:{self.port} closed")

    def register_handler(self, msg_type: MessageType, handler: callable):
        self.handlers[msg_type] = handler

    def send_request(self, msg_type: MessageType):
        self.sock.sendall(bytes(msg_type.value))

    def receive_n_bytes(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            buf = self.sock.recv(n - len(data))
            if len(buf) == 0:
                logging.warning("Connection closed unexpectedly")

            data += buf

        return data

    # just decide in the handlers what to do when not ok
    def wait_for_ok(self) -> int:
        response = self.receive_n_bytes(1)
        response = MessageType(response)

        if response == MessageType.OK:
            return 0

        logging.error("Error response from server!!!!")
        return -1

    def handle(self, msg_type: MessageType):
        handler = self.handlers.get(msg_type)
        handler(self)  # man I love python


def downscale_image(image: cv.UMat) -> cv.UMat:
    img = Image.fromarray(image)
    new_width = img.size[0] // 3
    new_height = img.size[1] // 3
    img = img.resize((new_width, new_height), Image.Resampling.LANCZOS)
    return np.array(img)


def handle_camera(client: DebugClient):
    client.send_request(MessageType.CAMERA_DATA)
    logging.debug("getting image")
    size = int.from_bytes(client.receive_n_bytes(4))
    payload = client.receive_n_bytes(size)

    image = np.frombuffer(payload, dtype=np.uint8)
    image_decoded = cv.imdecode(image, cv.IMREAD_COLOR)
    image_downscaled = downscale_image(image_decoded)

    computer_vision_result = fd.recompute_display(image_downscaled, image_decoded)
    cv.imwrite(output_image_file, computer_vision_result)

    logging.info(f"Image saved to {output_image_file}")


def handle_launch(client: DebugClient):
    client.send_request(MessageType.LAUNCH)
    #something else 


def handle_retreive(client: DebugClient):
    client.send_request(MessageType.RETRIEVE)
    #something else 

def handle_transmission_codes(client: DebugClient):
    client.send_request(MessageType.TRANSMISSION_CODES)
    #something else 


def handle_pos(client: DebugClient):
    client.send_request(MessageType.POS)
    #something else 


def get_that_stuff(client: DebugClient):
    while True:
        client.connect()
        logging.debug("Starting Camera")
        client.handle(MessageType.CAMERA_DATA)
        client.close()

        client.connect()
        logging.debug("Starting Launch")
        client.handle(MessageType.LAUNCH)
        client.close()

        client.connect()
        logging.debug("Starting Retreive")
        client.handle(MessageType.RETRIEVE)
        client.close()

        client.connect()
        logging.debug("Starting Transmission Codes")
        client.handle(MessageType.TRANSMISSION_CODES)
        client.close()

        client.connect()
        logging.debug("Starting Pos")
        client.handle(MessageType.POS)
        client.close()

        time.sleep(REQUEST_DELAY)


if __name__ == '__main__':
    client = DebugClient('localhost', 5000)

    client.register_handler(MessageType.CAMERA_DATA, handle_camera)
    client.register_handler(MessageType.LAUNCH, handle_launch)
    client.register_handler(MessageType.RETRIEVE, handle_retreive)
    client.register_handler(MessageType.TRANSMISSION_CODES, handle_transmission_codes)
    client.register_handler(MessageType.POS, handle_pos)

    get_that_stuff(client)
