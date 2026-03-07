import socket
import struct
from time import sleep

host = '192.168.4.1'
port = 3333

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
        data = client_tcp.recv(4, socket.MSG_WAITALL)
        size = struct.unpack('>I', data)[0]
        print(size)
        if size == 0:
            print("No image...")
            return
        print(f'The message was received from the server: {data}')

        image = recv_exact(client_tcp, size)
        frame_number += 1
        with open(f"images/frame{frame_number}.jpg", 'wb') as frame:
            frame.write(image)

if __name__ == '__main__':
    while True:
        save_image()
        sleep(5)
