"""Small, testable vision helpers for the live rescue-camera viewer."""

from dataclasses import dataclass
from typing import Any, Mapping, Sequence

import cv2
import numpy as np

from rescue_transport import ImageFrame


@dataclass(frozen=True)
class Detection:
    class_id: int
    label: str
    confidence: float
    xyxy: tuple[int, int, int, int]


def decode_jpeg(frame: ImageFrame) -> np.ndarray:
    """Decode one protocol frame to an OpenCV BGR image."""
    encoded = np.frombuffer(frame.jpeg, dtype=np.uint8)
    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"cannot decode JPEG in camera frame {frame.seq}")
    return image


def _class_label(names: Mapping[int, str] | Sequence[str], class_id: int) -> str:
    if isinstance(names, Mapping):
        return str(names.get(class_id, f"class_{class_id}"))
    if 0 <= class_id < len(names):
        return str(names[class_id])
    return f"class_{class_id}"


def run_detector(
    image: np.ndarray,
    model: Any,
    *,
    confidence: float = 0.35,
    image_size: int = 416,
    device: str | None = None,
) -> list[Detection]:
    """Run an Ultralytics YOLO model and normalize boxes to plain Python data."""
    predict_options: dict[str, Any] = {
        "source": image,
        "conf": confidence,
        "imgsz": image_size,
        "verbose": False,
    }
    if device is not None:
        predict_options["device"] = device

    results = model.predict(**predict_options)
    detections: list[Detection] = []
    for result in results:
        names = result.names
        boxes = result.boxes
        if boxes is None:
            continue
        for box in boxes:
            class_id = int(box.cls[0].item())
            score = float(box.conf[0].item())
            coords = box.xyxy[0].tolist()
            x1, y1, x2, y2 = (int(round(value)) for value in coords)
            detections.append(
                Detection(
                    class_id=class_id,
                    label=_class_label(names, class_id),
                    confidence=score,
                    xyxy=(x1, y1, x2, y2),
                )
            )
    return detections


def annotate_frame(
    image: np.ndarray,
    frame: ImageFrame,
    detections: Sequence[Detection] = (),
    *,
    fps: float | None = None,
    detector_enabled: bool = False,
) -> np.ndarray:
    """Return a display copy with status text and any detections drawn."""
    display = image.copy()
    height, width = display.shape[:2]
    cv2.rectangle(display, (0, 0), (width, 28), (24, 28, 35), thickness=-1)
    status = f"SEQ {frame.seq}  {width}x{height}"
    if fps is not None:
        status += f"  {fps:.1f} FPS"
    status += "  YOLO" if detector_enabled else "  PREVIEW ONLY - no model"
    cv2.putText(
        display,
        status,
        (7, 19),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.48,
        (235, 240, 245),
        1,
        cv2.LINE_AA,
    )

    for detection in detections:
        x1, y1, x2, y2 = detection.xyxy
        color = (
            80 + (37 * detection.class_id) % 160,
            100 + (67 * detection.class_id) % 140,
            120 + (97 * detection.class_id) % 120,
        )
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
        label = f"{detection.label} {detection.confidence:.2f}"
        text_y = max(y1 - 6, 42)
        cv2.putText(
            display,
            label,
            (x1, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
            cv2.LINE_AA,
        )
    return display
