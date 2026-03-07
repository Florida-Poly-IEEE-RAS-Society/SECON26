import socket
import struct
import numpy as np
import cv2 as cv
import UAV_Computer_Vision.field_detection as fd
from time import sleep

# host = '192.168.4.1'
host = 'localhost'
port = 5000

IMAGE = 1

def recv_exact(client, amount):
    rtn = b''
    while (len(rtn) < amount):
        rtn += client.recv(1024)
    return rtn

frame_number = 0

def save_image():
    global frame_number
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_tcp:
        client_tcp.connect((host, port))
        client_tcp.send(chr(IMAGE).encode())
        data = client_tcp.recv(4)
        size = int.from_bytes(data)
        print(size)
        if size == 0:
            print("No image...")
            return
        #nprint(f'The message was received from the server: {data}')

        image = recv_exact(client_tcp, size)
        image_decode = np.frombuffer(image, dtype=np.uint8)
        image_decode = cv.imdecode(image_decode, cv.IMREAD_COLOR)
        duck_points = fd.get_duck_points(image_decode)
        field_points = fd.get_field_corner_points(image_decode)
        print(duck_points)
        print(field_points)
    
    return duck_points

if __name__ == '__main__':
    while True:
        save_image()
        sleep(5)
