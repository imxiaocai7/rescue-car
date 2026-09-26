"""USB-only OpenMV preview/capture. No ESP32 connection or motion commands.

--probe only reads firmware information. --start-preview explicitly replaces the
running RAM script with our camera-only script; main.py and firmware are untouched.
"""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

import cv2
import serial
from serial.tools import list_ports

from camera_capture import CaptureSession
from openmv_usb import OpenMVUSB
from rescue_transport import ImageFrame
from rescue_vision import annotate_frame, run_detector
from view_camera import load_yolo


PREVIEW_SCRIPT = Path(__file__).resolve().parents[1] / "openmv" / "usb_preview.py"
WINDOW = "OpenMV USB - S: save, Q: quit"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list ports without opening them")
    parser.add_argument("--port", help="explicit OpenMV port, e.g. COM10; never auto-select Bluetooth")
    parser.add_argument("--probe", action="store_true", help="read board/firmware only")
    parser.add_argument("--start-preview", action="store_true", help="run camera-only script in RAM")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--frames", type=int, default=0, help="stop after N frames; 0 is unlimited")
    parser.add_argument("--duration", type=float, default=0, help="stop after N seconds")
    parser.add_argument("--save-dir", type=Path)
    parser.add_argument("--save-every", type=int, default=0, help="save every Nth frame; 0 is manual S")
    parser.add_argument("--notes", default="", help="lighting, scene, mounting, lens and distance notes")
    parser.add_argument("--model", type=Path, help="optional local rescue YOLO weights")
    parser.add_argument("--device", help="YOLO device (cpu or 0)")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=416)
    args = parser.parse_args()
    if args.list:
        for port in list_ports.comports():
            print(f"{port.device}: {port.description} [{port.hwid}]")
        return 0
    if not args.port:
        parser.error("Use --list, then explicitly select --port COMx")
    if args.frames < 0 or not math.isfinite(args.duration) or args.duration < 0 or args.save_every < 0:
        parser.error("frames, duration and save-every must be non-negative")
    if args.headless and not args.probe and not (args.frames or args.duration):
        parser.error("headless preview requires --frames or --duration")
    if args.save_every and args.save_dir is None:
        parser.error("--save-every requires --save-dir")
    if not 0 <= args.conf <= 1 or args.imgsz <= 0:
        parser.error("invalid YOLO confidence/image size")
    if args.model and not args.model.is_file():
        parser.error("model must be an existing local weight file")

    session = None
    count = 0
    first_received = last_received = None
    transfer_sum = transfer_max = 0.0
    started_script = False
    framebuffer_enabled = False
    script_logs = []
    failure = None
    window_created = False
    cleanup_errors = []
    stop_command_sent = False
    try:
        model = load_yolo(args.model) if args.model and not args.probe else None
        with serial.Serial(args.port, 921600, timeout=0.1, write_timeout=2) as port:
            camera = OpenMVUSB(port)
            info = camera.info()
            print(json.dumps(info, ensure_ascii=False), flush=True)
            if args.probe:
                return 0
            if args.save_dir:
                session = CaptureSession(args.save_dir, {**info, "port": args.port,
                    "source": "openmv_usbdbg", "notes": args.notes,
                    "ram_preview_started": args.start_preview,
                    "preview_script": PREVIEW_SCRIPT.read_text(encoding="utf-8") if args.start_preview else None})
                print(f"Capture session: {session.path}", flush=True)
            try:
                if args.start_preview:
                    script = PREVIEW_SCRIPT.read_text(encoding="utf-8")
                    camera.stop_script()
                    time.sleep(0.1)
                    started_script = True
                    # Drain the previous script's logs, keeping sessions distinct.
                    previous_log = camera.read_log()
                    if previous_log:
                        print("Previous script log:\n" + previous_log, flush=True)
                    camera.execute(script)
                camera.enable_framebuffer(True)
                framebuffer_enabled = True
                if not args.headless:
                    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
                    window_created = True
                    cv2.resizeWindow(WINDOW, 960, 720)
                start = last_frame = last_log = time.monotonic()
                ready = not args.start_preview
                startup_log = ""
                print("USB preview only; no motor connection. S saves original image, Q/Esc exits.", flush=True)
                while not args.frames or count < args.frames:
                    now = time.monotonic()
                    if args.duration and now - start >= args.duration:
                        break
                    before = time.perf_counter()
                    frame = camera.read_frame()
                    elapsed_ms = (time.perf_counter() - before) * 1000
                    received = time.monotonic()
                    received_utc = datetime.now(timezone.utc).isoformat()
                    key = -1 if args.headless else cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q"), ord("Q")):
                        break
                    if not args.headless and cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                        break
                    if received - last_log >= (1 if ready else 0.25):
                        log = camera.read_log()
                        if log:
                            script_logs.append(log)
                            print(log, end="", flush=True)
                            if not ready:
                                startup_log = (startup_log + log)[-8192:]
                                if "RESCUE_USB_READY" in startup_log:
                                    ready = True
                                    last_frame = received
                                    # Drop this image: it may precede the readiness marker.
                                    continue
                        last_log = received
                    if not ready:
                        if received - start > 10:
                            raise TimeoutError("Preview did not finish camera initialization; inspect script log")
                        time.sleep(0.01)
                        continue
                    if frame is None:
                        if received - last_frame > 10:
                            raise TimeoutError("No image for 10 seconds. Check script log; try --start-preview.")
                        time.sleep(0.01)
                        continue
                    last_frame = received
                    count += 1
                    fps = None if last_received is None else 1 / max(received - last_received, 1e-6)
                    first_received = received if first_received is None else first_received
                    last_received = received
                    transfer_sum += elapsed_ms
                    transfer_max = max(transfer_max, elapsed_ms)
                    if session and ((args.save_every and count % args.save_every == 0) or key in (ord("s"), ord("S"))):
                        path = session.save(frame, sequence=count,
                            received_utc=received_utc,
                            received_monotonic_ns=int(received * 1e9), transfer_ms=elapsed_ms)
                        print(f"Saved {path}", flush=True)
                    detections = run_detector(frame.image, model, confidence=args.conf,
                        image_size=args.imgsz, device=args.device) if model else []
                    if not args.headless:
                        # Timestamp 0 is only a display adapter, never localization data.
                        label_frame = ImageFrame(count, 0, frame.width, frame.height, b"")
                        display = annotate_frame(frame.image, label_frame, detections,
                            fps=fps, detector_enabled=model is not None)
                        cv2.imshow(WINDOW, display)
                    if count == 1 or count % 30 == 0:
                        print(f"frame={count} {frame.width}x{frame.height} {frame.format} "
                              f"bytes={len(frame.payload)} usb_read_ms={elapsed_ms:.1f}", flush=True)
                if count == 0:
                    raise TimeoutError("Preview finished without a ready image")
            finally:
                if framebuffer_enabled:
                    try:
                        camera.enable_framebuffer(False)
                    except OSError as error:
                        cleanup_errors.append(f"disable framebuffer: {error}")
                if started_script:
                    try:
                        camera.stop_script()
                        stop_command_sent = True
                    except OSError as error:
                        cleanup_errors.append(f"stop RAM script: {error}")
                    print("RAM stop command sent." if stop_command_sent else
                          "Could not send RAM stop command; reconnect/reboot the camera.", flush=True)
                    print("main.py unchanged; reboot restores startup script.", flush=True)
    except (OSError, ValueError, RuntimeError, cv2.error) as error:
        failure = str(error)
        print(f"Camera error: {failure}", file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        print("Preview interrupted.", flush=True)
    finally:
        if cleanup_errors and failure is None:
            failure = "; ".join(cleanup_errors)
        if window_created:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        if not args.probe:
            summary = {"frames": count, "saved": session.saved if session else 0,
                       "mean_receive_fps": (count - 1) / (last_received - first_received)
                       if count > 1 and last_received > first_received else None,
                       "mean_usb_read_ms": transfer_sum / count if count else None,
                       "max_usb_read_ms": transfer_max if count else None,
                       "error": failure, "script_log": script_logs,
                       "cleanup_errors": cleanup_errors, "ram_stop_command_sent": stop_command_sent,
                       "note": "USB read time is not sensor exposure-to-PC latency; no camera timestamp."}
            if session:
                (session.path / "summary.json").write_text(
                    json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
            print(json.dumps(summary, ensure_ascii=False), flush=True)
    return 1 if failure else 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    raise SystemExit(main())
