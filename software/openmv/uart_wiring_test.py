"""Temporary 115200-baud wiring test; no camera, filesystem or motion IO."""
from pyb import UART
import time

uart = UART(3, 115200, timeout_char=50)
buffer = bytearray()
print("RESCUE_UART_WIRING_READY")
while True:
    data = uart.read()
    if data:
        buffer.extend(data)
        if b"RESCUE_PING\n" in buffer:
            uart.write(b"RESCUE_PONG\n")
            buffer = bytearray()
        elif len(buffer) > 64:
            buffer = buffer[-32:]
    time.sleep_ms(2)
