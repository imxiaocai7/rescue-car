"""Bench check: OpenMV UART -> ESP32 -> USB, without changing Windows Wi-Fi.

ESP32 must already run our gateway firmware. Temporarily runs camera-only scripts
on OpenMV, never writes its filesystem. This does not verify the Wi-Fi link.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time

import serial

from openmv_usb import OpenMVUSB
from rescue_transport import ImageFrameDecoder
from rescue_vision import decode_jpeg


SCRIPTS = Path(__file__).resolve().parents[1] / "openmv"


def read_until(port, marker: bytes, timeout: float = 5) -> bytes:
    deadline = time.monotonic() + timeout
    response = bytearray()
    while time.monotonic() < deadline:
        chunk = port.read(max(1, min(port.in_waiting, 4096)))
        response.extend(chunk)
        if marker in response:
            return bytes(response)
        if len(response) > 65536:
            raise ValueError("Diagnostic reply exceeds limit")
    raise TimeoutError(f"Missing {marker!r}; reply: {bytes(response[-512:])!r}")


def start_script(camera, filename: str, marker: str) -> str:
    camera.stop_script()
    time.sleep(0.1)
    camera.enable_framebuffer(False)
    camera.read_log()  # Discard the previous program's output.
    camera.execute((SCRIPTS / filename).read_text(encoding="utf-8"))
    deadline = time.monotonic() + 10
    output = ""
    while time.monotonic() < deadline:
        output += camera.read_log()
        if marker in output:
            return output
        if "Traceback" in output:
            raise RuntimeError(output)
        time.sleep(0.05)
    raise TimeoutError(f"OpenMV script not ready: {output[-2000:]}")


def request_frame(port, decoder, timeout: float = 8):
    if port.write(b"J") != 1:
        raise OSError("Could not request diagnostic image")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        chunk = port.read(max(1, min(port.in_waiting, 8192)))
        frames = decoder.feed(chunk)
        if frames:
            return frames[0]
    raise TimeoutError("No valid image returned over ESP32 USB diagnostic; check UART and camera logs")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--camera-port", required=True)
    parser.add_argument("--gateway-port", required=True)
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--pings", type=int, default=5)
    parser.add_argument("--report", type=Path, required=True, help="new JSON file, must not exist")
    parser.add_argument("--keep-camera-running", action="store_true",
                        help="leave UART capture in RAM for subsequent Wi-Fi testing")
    args = parser.parse_args()
    if args.camera_port.upper() == args.gateway_port.upper():
        parser.error("camera and gateway ports must differ")
    if not 1 <= args.frames <= 10000 or not 1 <= args.pings <= 100:
        parser.error("frames must be 1..10000 and pings 1..100")
    if args.report.exists():
        parser.error("report already exists; choose a new filename")
    report = {"started_utc": datetime.now(timezone.utc).isoformat(), "pings_ok": 0,
              "frames": [], "error": None, "cleanup_error": None,
              "source": "OpenMV UART3 -> ESP32 UART1 -> ESP32 USB diagnostic",
              "wifi_verified": False, "camera_script_left_running": False}
    try:
        with serial.Serial(args.camera_port, 921600, timeout=0.1, write_timeout=2) as camera_port, \
             serial.Serial(args.gateway_port, 115200, timeout=0.1, write_timeout=2) as gateway:
            camera = OpenMVUSB(camera_port)
            report["camera"] = camera.info()
            try:
                gateway.reset_input_buffer()
                # Opening a native USB/JTAG serial port can reboot the S3.
                # Repeat this read-only request while firmware finishes booting.
                deadline = time.monotonic() + 12
                gateway_reply = bytearray()
                while time.monotonic() < deadline and b"USB diagnostic:" not in gateway_reply:
                    gateway.write(b"?")
                    try:
                        gateway_reply.extend(read_until(gateway, b"USB diagnostic:", timeout=0.5))
                    except TimeoutError:
                        pass
                if b"USB diagnostic:" not in gateway_reply:
                    raise TimeoutError(f"ESP32 did not report startup status: {bytes(gateway_reply[-512:])!r}")
                report["gateway"] = gateway_reply.decode(errors="replace")
                print(report["gateway"], flush=True)
                print(start_script(camera, "uart_wiring_test.py", "RESCUE_UART_WIRING_READY"), flush=True)
                for number in range(args.pings):
                    gateway.write(b"T")
                    read_until(gateway, b"UART_TEST_OK")
                    report["pings_ok"] += 1
                    print(f"Wiring ping {number + 1}/{args.pings}: OK", flush=True)
                print(start_script(camera, "camera_uart.py", "RESCUE_UART_CAMERA_READY"), flush=True)
                decoder = ImageFrameDecoder()
                for number in range(args.frames):
                    started = time.perf_counter()
                    frame = request_frame(gateway, decoder)
                    image = decode_jpeg(frame)
                    if image.shape[:2] != (frame.height, frame.width):
                        raise ValueError("JPEG size differs from image header")
                    elapsed = (time.perf_counter() - started) * 1000
                    report["frames"].append({"seq": frame.seq, "width": frame.width,
                        "height": frame.height, "jpeg_bytes": len(frame.jpeg), "request_ms": elapsed})
                    print(f"Frame {number + 1}/{args.frames}: seq={frame.seq}, "
                          f"{frame.width}x{frame.height}, {len(frame.jpeg)} bytes, {elapsed:.1f} ms", flush=True)
                report["bad_headers"] = decoder.bad_headers
                report["bad_payloads"] = decoder.bad_payloads
                if decoder.bad_headers or decoder.bad_payloads:
                    raise ValueError("Corrupt frames observed during bench check")
                if args.keep_camera_running:
                    report["camera_script_left_running"] = True
            finally:
                if not report["camera_script_left_running"]:
                    try:
                        camera.stop_script()
                    except OSError as error:
                        report["cleanup_error"] = str(error)
    except (OSError, ValueError, RuntimeError) as error:
        report["error"] = str(error)
        print(f"Bench check failed: {error}", file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        report["error"] = "Interrupted by operator"
    args.report.parent.mkdir(parents=True, exist_ok=True)
    with args.report.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
    print(f"Report: {args.report}; pings={report['pings_ok']}, frames={len(report['frames'])}")
    return 1 if report["error"] or report["cleanup_error"] else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    raise SystemExit(main())
