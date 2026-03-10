import UAV_Computer_Vision.field_detection as fd

import sys
import struct
import numpy as np
import cv2 as cv


def read_image_from_pipe() -> bytes:
    size = sys.stdin.buffer.read(4)
    if len(size) < 4:
        return None

    size = int.from_bytes(size, 'little')

    output = sys.stdin.buffer.read(size)

    return output


def write_coords_to_pipe(coords: list[float]):
    # very pythonic ver niceee
    data = b''.join(struct.pack('<ff', float(
        f[0]), float(f[1])) for f in coords)
    sys.stdout.buffer.write(struct.pack('<I', len(coords)*2))
    sys.stdout.buffer.write(data)
    sys.stdout.flush()


def write_field_corners_to_pipe(coords: dict[str, float]):
    data = b''
    for _, v in coords.items():
        data += struct.pack("<ff", float(v[0]), float(v[1]))

    sys.stdout.buffer.write(struct.pack('<I', 8))
    sys.stdout.buffer.write(data)
    sys.stdout.flush()


def bytes_to_opencv_image(image: bytes) -> cv.UMat:
    image_decode = np.frombuffer(image, dtype=np.uint8)
    image_decode = cv.imdecode(image_decode, cv.IMREAD_COLOR)

    return image_decode


def call_function(func: str) -> list[float]:
    image_bytes = read_image_from_pipe()
    image = bytes_to_opencv_image(image_bytes)

    output = []

    match func:
        case 'duck_points':
            output = fd.get_duck_points(image)
            write_coords_to_pipe(output)
        case 'field_corners':
            output = fd.get_field_corner_points(image)
            write_field_corners_to_pipe(output)

    return output


if __name__ == "__main__":
    command = sys.argv[1]
    result = call_function(command)
