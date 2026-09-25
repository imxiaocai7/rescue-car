"""OpenMV H7 Plus camera -> ESP32-S3 UART3, bench-test firmware.

Send one JPEG only after receiving byte 'R'. The ESP32 receiver must keep
reading until the entire 26-byte header and declared JPEG body have arrived,
verify CRC32, then send the next 'R'. No motion control is performed here.
"""

import sensor
import struct
import time
from pyb import UART


BAUD = 921600  # Change both ends together; begin with 115200 for a text test.
QUALITY = 40
MAX_JPEG_SIZE = 120000
HEADER_FORMAT = "<4sBBIIHHII"


def make_crc32_table():
    table = []
    for number in range(256):
        crc = number
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
        table.append(crc)
    return table


CRC32_TABLE = make_crc32_table()


def crc32(data):
    crc = 0xFFFFFFFF
    for value in data:
        crc = CRC32_TABLE[(crc ^ value) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def write_all(uart, data):
    view = memoryview(data)
    position = 0
    while position < len(view):
        written = uart.write(view[position : position + 1024])
        if written is None or written <= 0:
            raise OSError("UART write timed out")
        position += written


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
uart = UART(3, BAUD, timeout_char=1000)
sequence = 0

while True:
    if not uart.any():
        time.sleep_ms(2)
        continue
    if uart.readchar() != ord("R"):
        continue

    image = sensor.snapshot()
    jpeg = image.compress(quality=QUALITY).bytearray()
    size = len(jpeg)
    if size < 2 or size > MAX_JPEG_SIZE:
        continue
    header = struct.pack(
        HEADER_FORMAT,
        b"RSV1",
        1,
        1,
        sequence,
        time.ticks_ms() & 0xFFFFFFFF,
        image.width(),
        image.height(),
        size,
        crc32(jpeg),
    )
    write_all(uart, header)
    write_all(uart, jpeg)
    sequence = (sequence + 1) & 0xFFFFFFFF
