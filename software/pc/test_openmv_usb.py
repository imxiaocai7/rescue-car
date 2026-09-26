import contextlib
import hashlib
import io
import itertools
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import cv2
import numpy as np

from camera_capture import CaptureSession
from openmv_usb import COMMAND, OpenMVUSB, decode_frame, frame_bytes
import view_openmv_usb


class FragmentedPort:
    def __init__(self, data=b"", step=2):
        self.data = bytearray(data)
        self.written = bytearray()
        self.step = step

    def write(self, data):
        size = min(len(data), self.step)
        self.written.extend(data[:size])
        return size

    def read(self, count):
        size = min(count, len(self.data), self.step)
        result = bytes(self.data[:size])
        del self.data[:size]
        return result


def jpeg_frame():
    image = np.full((24, 32, 3), (20, 70, 180), np.uint8)
    ok, encoded = cv2.imencode(".jpg", image)
    assert ok
    data = encoded.tobytes()
    return decode_frame(32, 24, len(data), data)


class USBProtocolTests(unittest.TestCase):
    def test_fragmented_firmware_query_is_read_only(self):
        port = FragmentedPort(struct.pack("<III", 4, 7, 0) + b"OMV4P".ljust(64, b"\0"))
        self.assertEqual(OpenMVUSB(port).info(), {"firmware": "4.7.0", "board": "OMV4P"})
        self.assertEqual(port.written, COMMAND.pack(0x30, 0x80, 12) + COMMAND.pack(0x30, 0x83, 64))

    def test_unsupported_firmware_fails_before_other_commands(self):
        port = FragmentedPort(struct.pack("<III", 5, 0, 0))
        with self.assertRaises(RuntimeError):
            OpenMVUSB(port).info()
        self.assertEqual(len(port.written), COMMAND.size)

    def test_missing_reply_times_out(self):
        with self.assertRaises(TimeoutError):
            OpenMVUSB(FragmentedPort(), timeout=0.002).info()

    def test_zero_write_fails(self):
        with self.assertRaises(TimeoutError):
            OpenMVUSB(FragmentedPort(step=0)).info()

    def test_reject_invalid_timeout(self):
        for timeout in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                OpenMVUSB(FragmentedPort(), timeout=timeout)

    def test_not_ready_frame(self):
        port = FragmentedPort(bytes(12))
        self.assertIsNone(OpenMVUSB(port).read_frame())
        self.assertEqual(port.written, COMMAND.pack(0x30, 0x81, 12))

    def test_fragmented_jpeg_frame(self):
        frame = jpeg_frame()
        port = FragmentedPort(struct.pack("<III", 32, 24, len(frame.payload)) + frame.payload)
        decoded = OpenMVUSB(port).read_frame()
        self.assertEqual(decoded.payload, frame.payload)
        np.testing.assert_array_equal(decoded.image, frame.image)

    def test_invalid_header_never_requests_payload(self):
        port = FragmentedPort(struct.pack("<III", 640, 480, 99999999))
        with self.assertRaises(ValueError):
            OpenMVUSB(port).read_frame()
        self.assertEqual(len(port.written), COMMAND.size)

    def test_frame_limits(self):
        for args in ((0, 10, 1), (1, 4097, 2), (1, 1, 0), (4096, 4096, 2)):
            with self.assertRaises(ValueError):
                frame_bytes(*args)

    def test_grayscale(self):
        frame = decode_frame(2, 1, 1, bytes((0, 255)))
        np.testing.assert_array_equal(frame.image, [[[0, 0, 0], [255, 255, 255]]])

    def test_rgb565_color_order(self):
        frame = decode_frame(3, 1, 2, struct.pack("<HHH", 0xF800, 0x07E0, 0x001F))
        np.testing.assert_array_equal(frame.image, [[[0, 0, 255], [0, 255, 0], [255, 0, 0]]])

    def test_corrupt_or_wrong_sized_jpeg(self):
        frame = jpeg_frame()
        for width, data in ((31, frame.payload), (32, b"bad jpeg data"), (32, frame.payload[:-1])):
            with self.assertRaises(ValueError):
                decode_frame(width, 24, len(frame.payload), data)

    def test_execute_uses_utf8_byte_count_and_no_flash_commands(self):
        port = FragmentedPort()
        camera = OpenMVUSB(port)
        camera.execute("print('相机')")
        data = "print('相机')".encode()
        self.assertEqual(port.written, COMMAND.pack(0x30, 5, len(data)) + data)

    def test_log_read(self):
        log = "SENSOR_ID 0x56\n".encode()
        port = FragmentedPort(struct.pack("<I", len(log)) + log)
        self.assertEqual(OpenMVUSB(port).read_log(), log.decode())

    def test_log_limit(self):
        with self.assertRaises(ValueError):
            OpenMVUSB(FragmentedPort(struct.pack("<I", 0xFFFFFFFF))).read_log()


class CaptureTests(unittest.TestCase):
    def save(self, session, frame):
        return session.save(frame, sequence=1, received_utc="2026-09-26T00:00:00Z",
                            received_monotonic_ns=123, transfer_ms=1.2)

    def test_original_jpeg_and_provenance_unicode_path(self):
        with tempfile.TemporaryDirectory() as temp:
            session = CaptureSession(Path(temp) / "采图", {"notes": "测试"})
            frame = jpeg_frame()
            path = self.save(session, frame)
            self.assertEqual(path.read_bytes(), frame.payload)
            record = json.loads((session.path / "frames.jsonl").read_text(encoding="utf-8"))
            self.assertEqual(record["sha256"], hashlib.sha256(frame.payload).hexdigest())
            self.assertIsNone(record["camera_ticks_ms"])
            self.assertEqual(session.saved, 1)
            with self.assertRaises(ValueError):
                self.save(session, frame)
            self.assertEqual(path.read_bytes(), frame.payload)

    def test_raw_frame_saved_losslessly_as_png(self):
        with tempfile.TemporaryDirectory() as temp:
            session = CaptureSession(Path(temp), {})
            frame = decode_frame(2, 1, 1, bytes((10, 200)))
            path = self.save(session, frame)
            self.assertEqual(path.suffix, ".png")
            restored = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), cv2.IMREAD_COLOR)
            np.testing.assert_array_equal(restored, frame.image)

    def test_sessions_never_reuse_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            self.assertNotEqual(CaptureSession(Path(temp), {}).path, CaptureSession(Path(temp), {}).path)


class USBCommandTests(unittest.TestCase):
    def test_disconnect_does_not_claim_stop_command_sent(self):
        camera = MagicMock()
        camera.info.return_value = {"firmware": "4.7.0"}
        camera.read_log.return_value = ""
        camera.stop_script.side_effect = [None, OSError("disconnected")]
        with tempfile.TemporaryDirectory() as temp, \
             patch("sys.argv", ["usb", "--port", "COM10", "--start-preview", "--save-dir", temp]), \
             patch.object(view_openmv_usb.serial, "Serial"), \
             patch.object(view_openmv_usb, "OpenMVUSB", return_value=camera), \
             patch.object(view_openmv_usb.time, "sleep"), \
             patch.object(view_openmv_usb.cv2, "namedWindow", side_effect=cv2.error("no GUI")), \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(view_openmv_usb.main(), 1)
            summary = json.loads(next(Path(temp).glob("*/summary.json")).read_text(encoding="utf-8"))
            self.assertFalse(summary["ram_stop_command_sent"])
            self.assertEqual(summary["cleanup_errors"], ["stop RAM script: disconnected"])

    def test_warmup_frames_are_discarded_and_own_script_is_stopped(self):
        camera = MagicMock()
        camera.info.return_value = {"firmware": "4.7.0"}
        camera.read_frame.return_value = jpeg_frame()
        camera.read_log.side_effect = ["previous log", "RESCUE_USB_READY\n"]
        ticks = itertools.count(0, 0.1)
        with tempfile.TemporaryDirectory() as temp, \
             patch("sys.argv", ["usb", "--port", "COM10", "--headless", "--start-preview",
                                "--frames", "1", "--save-every", "1", "--save-dir", temp]), \
             patch.object(view_openmv_usb.serial, "Serial"), \
             patch.object(view_openmv_usb, "OpenMVUSB", return_value=camera), \
             patch.object(view_openmv_usb.time, "monotonic", side_effect=lambda: next(ticks)), \
             patch.object(view_openmv_usb.time, "sleep"), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(view_openmv_usb.main(), 0)
            self.assertEqual(len(list(Path(temp).glob("*/*.jpg"))), 1)
        self.assertGreater(camera.read_frame.call_count, 1)
        self.assertEqual(camera.stop_script.call_count, 2)
        camera.execute.assert_called_once()

    def test_gui_init_failure_still_writes_summary_and_cleans_up(self):
        camera = MagicMock()
        camera.info.return_value = {"firmware": "4.7.0"}
        camera.read_log.return_value = ""
        with tempfile.TemporaryDirectory() as temp, \
             patch("sys.argv", ["usb", "--port", "COM10", "--start-preview", "--save-dir", temp]), \
             patch.object(view_openmv_usb.serial, "Serial"), \
             patch.object(view_openmv_usb, "OpenMVUSB", return_value=camera), \
             patch.object(view_openmv_usb.time, "sleep"), \
             patch.object(view_openmv_usb.cv2, "namedWindow", side_effect=cv2.error("headless")), \
             patch.object(view_openmv_usb.cv2, "destroyAllWindows") as destroy, \
             contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(view_openmv_usb.main(), 1)
            summary = json.loads(next(Path(temp).glob("*/summary.json")).read_text(encoding="utf-8"))
            self.assertIn("headless", summary["error"])
        destroy.assert_not_called()
        camera.enable_framebuffer.assert_any_call(False)
        self.assertEqual(camera.stop_script.call_count, 2)

    def test_probe_does_not_start_script_or_enable_framebuffer(self):
        camera = MagicMock()
        camera.info.return_value = {"firmware": "4.7.0"}
        with patch("sys.argv", ["usb", "--port", "COM10", "--probe"]), \
             patch.object(view_openmv_usb.serial, "Serial"), \
             patch.object(view_openmv_usb, "OpenMVUSB", return_value=camera), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(view_openmv_usb.main(), 0)
        camera.execute.assert_not_called()
        camera.stop_script.assert_not_called()
        camera.enable_framebuffer.assert_not_called()

    def test_headless_capture_and_cleanup_without_stopping_existing_script(self):
        camera = MagicMock()
        camera.info.return_value = {"firmware": "4.7.0"}
        camera.read_frame.return_value = jpeg_frame()
        with tempfile.TemporaryDirectory() as temp, \
             patch("sys.argv", ["usb", "--port", "COM10", "--headless", "--frames", "2",
                                "--save-every", "1", "--save-dir", temp]), \
             patch.object(view_openmv_usb.serial, "Serial"), \
             patch.object(view_openmv_usb, "OpenMVUSB", return_value=camera), \
             contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(view_openmv_usb.main(), 0)
            summary = json.loads(next(Path(temp).glob("*/summary.json")).read_text(encoding="utf-8"))
            self.assertEqual((summary["frames"], summary["saved"]), (2, 2))
        camera.enable_framebuffer.assert_any_call(False)
        camera.stop_script.assert_not_called()


if __name__ == "__main__":
    unittest.main()
