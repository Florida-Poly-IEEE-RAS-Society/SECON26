import logging
import socket
from enum import Enum


class MessageType(Enum):
    CAMERA_DATA = 1
    LAUNCH = 2
    RETREIVE = 3
    TRANSMISSION_CODES = 4
    POS = 5
    STOP = 6
    THRUST = 7
    THRUST_CONTROL_MODE = 8
    PITCH = 9
    ROLL = 10
    YAW = 11
    SET_HEIGHT = 12
    SET_X = 13
    SET_Y = 14
    SET_PID = 15
    GET_PID = 16
    SAVE_PID = 17
    GYRO_CALIBRATION_STATUS = 18
    GET_GAME_STATE = 19
    POS_VEL = 20
    GYRO_ANGLE = 21


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
        self.sock.sendall(msg_type.value.to_bytes(1, 'big'))

    def send_payload(self, payload: bytes):
        self.sock.sendall(payload)

    def receive_n_bytes(self, n: int) -> bytes:
        data = b""
        while len(data) < n:
            buf = self.sock.recv(n - len(data))
            if len(buf) == 0:
                logging.warning("Connection closed unexpectedly")

            data += buf

        return data

    def handle(self, msg_type: MessageType, args: dict | None):
        handler = self.handlers.get(msg_type)
        if not handler:
            logging.error(f"handler {msg_type} not registered")
            return

        self.connect()
        self.send_request(msg_type)

        if args is None:
            handler(self)
        else:
            handler(self, args)  # man I love python

        self.close()

