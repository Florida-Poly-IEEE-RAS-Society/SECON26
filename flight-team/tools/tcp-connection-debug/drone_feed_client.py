from client import DebugClient, MessageType
from time import sleep
import handlers
import cv2 as cv

HOST = '192.168.4.1'
PORT = 3333

client = DebugClient(HOST, PORT)
count = 1
while True:
    client.connect()
    client.send_request(MessageType.CAMERA_DATA)
    img = handlers.receive_image(client)
    client.close()
    cv.imwrite(f'drone_feed/frame{count}.png', img)
    count += 1
    sleep(2)

