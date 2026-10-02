"""Native C and Python wire compatibility; never accesses a board or port."""

from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

from motion_protocol import ActuatorCommand, MotionCommand, encode_motion_command
from rover_telemetry import TelemetryDecoder, TelemetryStatus


class Stm32CommissioningTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise unittest.SkipTest("native GCC not installed; ARM build is separate")
        cls.directory = tempfile.TemporaryDirectory(prefix="rover-stm32-native-")
        cls.addClassCleanup(cls.directory.cleanup)
        root = Path(__file__).resolve().parents[1]
        include = root / "stm32" / "include"
        source = root / "stm32" / "src"
        cls.application_exe = Path(cls.directory.name) / "commissioning.exe"
        cls.queue_exe = Path(cls.directory.name) / "queue.exe"
        for executable, sources in (
            (cls.application_exe, [source / name for name in (
                "motion_protocol.c", "motion_command_guard.c", "rover_telemetry.c",
                "commissioning_application.c",
            )] + [root / "tests" / "commissioning_application_test.c"]),
            (cls.queue_exe, [source / "uart_rx_queue.c", root / "tests" / "uart_rx_queue_test.c"]),
        ):
            subprocess.run(
                [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                 "-I", str(include), *(str(path) for path in sources), "-o", str(executable)],
                check=True, capture_output=True, timeout=30,
            )

    def wire(self, payload):
        result = subprocess.run([str(self.application_exe), "--wire"], input=payload,
                                check=True, capture_output=True, timeout=3)
        wire = bytes.fromhex(result.stdout.decode("ascii"))
        frames = TelemetryDecoder().feed(wire)
        self.assertEqual(len(frames), 1)
        frame = frames[0]
        self.assertTrue(frame.status & TelemetryStatus.EMERGENCY_STOP)
        self.assertFalse(frame.status & (TelemetryStatus.LEFT_ENCODER_VALID | TelemetryStatus.RIGHT_ENCODER_VALID))
        return frame

    def test_native_application_assertions(self):
        subprocess.run([str(self.application_exe)], check=True, capture_output=True, timeout=3)

    def test_native_queue_assertions(self):
        subprocess.run([str(self.queue_exe)], check=True, capture_output=True, timeout=3)

    def test_python_zero_command_is_acknowledged_by_c(self):
        frame = self.wire(encode_motion_command(MotionCommand(0x12345678, 300, 0, 0)))
        self.assertEqual(frame.last_command_sequence, 0x12345678)
        self.assertTrue(frame.status & TelemetryStatus.LAST_COMMAND_VALID)
        self.assertEqual(frame.stm32_ticks_ms, 42)

    def test_unsafe_motion_and_servo_are_rejected_by_c(self):
        for command in (MotionCommand(1, 300, 0, 1), MotionCommand(1, 300, -1, 0),
                        MotionCommand(1, 300, 0, 0, ActuatorCommand.OPEN),
                        MotionCommand(1, 300, 0, 0, ActuatorCommand.CLOSE)):
            with self.subTest(command=command):
                frame = self.wire(encode_motion_command(command))
                self.assertFalse(frame.status & TelemetryStatus.LAST_COMMAND_VALID)

    def test_noise_bad_crc_and_replay_recover(self):
        damaged = bytearray(encode_motion_command(MotionCommand(100, 300, 0, 0)))
        damaged[-1] ^= 0x80
        payload = b"noise" + damaged + encode_motion_command(MotionCommand(50, 300, 0, 0))
        payload += encode_motion_command(MotionCommand(49, 300, 0, 0))
        self.assertEqual(self.wire(payload).last_command_sequence, 50)

    def test_sequence_wrap_compatible(self):
        payload = encode_motion_command(MotionCommand(0xFFFFFFFF, 300, 0, 0))
        payload += encode_motion_command(MotionCommand(0, 300, 0, 0))
        self.assertEqual(self.wire(payload).last_command_sequence, 0)

    def test_all_byte_values_in_sequence_survive_native_stdin(self):
        payload = b"".join(encode_motion_command(MotionCommand(seq, 300, 0, 0)) for seq in range(256))
        self.assertEqual(self.wire(payload).last_command_sequence, 255)


if __name__ == "__main__":
    unittest.main()
