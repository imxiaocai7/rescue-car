"""RAM-only USB preview: no UART, motors, filesystem writes, or overlays.

Executed by the PC USBDBG client; do not copy this over an existing main.py.
Keep RGB565/QVGA aligned with the initial ESP32 capture configuration.
"""
import sensor
import time
import os

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
print("RESCUE_PREVIEW", os.uname())
print("SENSOR_ID", hex(sensor.get_id()))
print("FRAME_SIZE", sensor.width(), sensor.height())
print("RESCUE_USB_READY")

while True:
    sensor.snapshot()
    time.sleep_ms(10)
