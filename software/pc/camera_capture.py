"""Lossless storage of USB JPEGs (PNG for raw frames), plus capture provenance."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import uuid

import cv2

from openmv_usb import USBFrame


class CaptureSession:
    def __init__(self, root: Path, metadata: dict):
        session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
        self.path = root / session_id
        self.path.mkdir(parents=True, exist_ok=False)
        self.saved = 0
        self._recorded: set[int] = set()
        session = {**metadata, "session_id": session_id,
                   "created_utc": datetime.now(timezone.utc).isoformat(),
                   "camera_timestamp_available": False,
                   "split": "unassigned", "labels": "not_annotated"}
        (self.path / "session.json").write_text(
            json.dumps(session, ensure_ascii=False, indent=2), encoding="utf-8")

    def save(self, frame: USBFrame, *, sequence: int, received_utc: str,
             received_monotonic_ns: int, transfer_ms: float) -> Path:
        if sequence < 0 or sequence in self._recorded:
            raise ValueError("Capture sequence must be non-negative and unique")
        if frame.format == "JPEG":
            data, suffix = frame.payload, ".jpg"
        else:
            ok, encoded = cv2.imencode(".png", frame.image)
            if not ok:
                raise ValueError("PNG encoding failed")
            data, suffix = encoded.tobytes(), ".png"
        target = self.path / f"frame_{sequence:08d}{suffix}"
        # Exclusive create protects existing images and supports Unicode Windows paths.
        with target.open("xb") as stream:
            stream.write(data)
        record = {"file": target.name, "sequence": sequence,
                  "received_utc": received_utc, "received_monotonic_ns": received_monotonic_ns,
                  "transfer_ms": transfer_ms, "camera_ticks_ms": None,
                  "width": frame.width, "height": frame.height, "usb_format": frame.format,
                  "usb_payload_bytes": len(frame.payload),
                  "sha256": hashlib.sha256(data).hexdigest()}
        with (self.path / "frames.jsonl").open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._recorded.add(sequence)
        self.saved += 1
        return target
