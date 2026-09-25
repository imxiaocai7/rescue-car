"""Live preview of the OpenMV -> ESP32 -> PC stream, with optional YOLO."""

import argparse
from pathlib import Path
import socket
import time

import cv2

from rescue_transport import ImageFrameDecoder, encode_frame_ack
from rescue_vision import annotate_frame, decode_jpeg, run_detector


def load_yolo(weights: Path):
    try:
        from ultralytics import YOLO
    except ImportError as error:
        raise RuntimeError(
            "YOLO inference requires the optional 'ultralytics' package; "
            "install software/pc/requirements-yolo.txt first"
        ) from error

    model = YOLO(str(weights))
    return model


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", nargs="?", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--model", type=Path, help="trained rescue-target .pt weights")
    parser.add_argument("--conf", type=float, default=0.35)
    parser.add_argument("--imgsz", type=int, default=416)
    parser.add_argument("--device", help="Ultralytics device, e.g. 0 or cpu; default is auto")
    parser.add_argument("--save-dir", type=Path, help="optional directory for frames saved by pressing S")
    args = parser.parse_args()

    if not 0.0 <= args.conf <= 1.0:
        parser.error("--conf must be between 0 and 1")
    if args.imgsz <= 0:
        parser.error("--imgsz must be positive")
    if args.model is not None and not args.model.is_file():
        parser.error(f"model file does not exist: {args.model}")

    model = load_yolo(args.model) if args.model is not None else None
    if model is None:
        print("Preview-only mode: no rescue-target model loaded.")
    else:
        print(f"YOLO model: {args.model} (device={args.device or 'auto'})")
    if args.save_dir is not None:
        args.save_dir.mkdir(parents=True, exist_ok=True)

    decoder = ImageFrameDecoder()
    count = 0
    last_frame_at: float | None = None
    cv2.namedWindow("Rescue robot camera", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Rescue robot camera", 960, 720)
    print(f"Connecting to camera gateway {args.host}:{args.port}; press Q/Esc to quit, S to save a frame.")

    try:
        with socket.create_connection((args.host, args.port), timeout=5) as connection:
            connection.settimeout(10)
            while True:
                chunk = connection.recv(8192)
                if not chunk:
                    raise ConnectionError("camera connection closed")

                for frame in decoder.feed(chunk):
                    image = decode_jpeg(frame)
                    now = time.perf_counter()
                    fps = None if last_frame_at is None else 1.0 / max(now - last_frame_at, 1e-6)
                    last_frame_at = now
                    detections = (
                        run_detector(
                            image,
                            model,
                            confidence=args.conf,
                            image_size=args.imgsz,
                            device=args.device,
                        )
                        if model is not None
                        else []
                    )
                    display = annotate_frame(
                        image,
                        frame,
                        detections,
                        fps=fps,
                        detector_enabled=model is not None,
                    )
                    cv2.imshow("Rescue robot camera", display)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        return
                    if key in (ord("s"), ord("S")):
                        if args.save_dir is None:
                            print("Set --save-dir to enable frame capture.")
                        else:
                            image_path = args.save_dir / f"frame_{frame.seq:08d}_{frame.camera_ticks_ms:010d}.jpg"
                            if cv2.imwrite(str(image_path), image):
                                print(f"Saved {image_path}")
                            else:
                                print(f"Failed to save {image_path}")

                    count += 1
                    print(
                        f"seq={frame.seq} detections={len(detections)} "
                        f"bad_header={decoder.bad_headers} bad_payload={decoder.bad_payloads}",
                        flush=True,
                    )
                    connection.sendall(encode_frame_ack(frame.seq))
    finally:
        cv2.destroyAllWindows()
        print(f"Viewer stopped after {count} frames.")


if __name__ == "__main__":
    main()
