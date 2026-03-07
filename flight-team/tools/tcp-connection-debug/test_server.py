import socket

HOST = '0.0.0.0'
PORT = 5000
IMAGE_PATH = './UAV_Computer_Vision/cool_duck_images/IMG_20251101_181840.jpg'


def send_image(conn, image_path):
    with open(image_path, 'rb') as f:
        data = f.read()
        conn.send(len(data).to_bytes(4, byteorder='big'))
        conn.sendall(data)
    print("Image sent.")


def start_server():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen(1)
        print(f"Server listening on {HOST}:{PORT}...")
        while True:
            conn, addr = s.accept()
            with conn:
                print(f"Connected by {addr}")
                request = conn.recv(1024)
                if request.find(1):
                    send_image(conn, IMAGE_PATH)
                else:
                    conn.send(b"Unknown request")


if __name__ == "__main__":
    start_server()
