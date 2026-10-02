import socket
import time
import unittest

from chassis_simulator import ChassisSimulation, ChassisSimulatorServer
from motion_protocol import MotionCommand, encode_motion_command
from rover_telemetry import TelemetryDecoder, TelemetryStatus


class ChassisSimulationTests(unittest.TestCase):
    def test_lease_integrates_only_until_expiry(self):
        model = ChassisSimulation()
        self.assertTrue(model.accept(MotionCommand(1, 300, 100, -100), 0.0))
        model.advance(0.1)
        self.assertEqual(model.snapshot().left_mm_s, 100)
        model.advance(1.0)
        wheels = model.snapshot()
        self.assertEqual((wheels.left_mm_s, wheels.right_mm_s), (0, 0))
        self.assertEqual(wheels.left_counts, round(30 * model.counts_per_mm))
        self.assertEqual(wheels.right_counts, -wheels.left_counts)

    def test_replay_does_not_extend_lease(self):
        model = ChassisSimulation()
        model.accept(MotionCommand(10, 300, 100, 100), 0)
        self.assertFalse(model.accept(MotionCommand(10, 300, 200, 200), 0.2))
        self.assertFalse(model.accept(MotionCommand(9, 300, 200, 200), 0.25))
        model.advance(0.301)
        self.assertEqual(model.snapshot().left_mm_s, 0)
        self.assertEqual(model.last_sequence, 10)

    def test_sequence_wrap_and_counter_wrap(self):
        model = ChassisSimulation()
        model.left_count = 2**31 + 5
        model.accept(MotionCommand(0xFFFFFFFF, 300, 0, 0), 0)
        self.assertTrue(model.accept(MotionCommand(0, 300, 0, 0), 0))
        frame = model.telemetry()
        self.assertEqual(frame.left_encoder_total, -(2**31) + 5)
        self.assertEqual(frame.last_command_sequence, 0)
        self.assertTrue(frame.status & TelemetryStatus.LAST_COMMAND_VALID)

    def test_faults_stop_and_do_not_resume_old_command(self):
        for attribute in ("emergency_stop", "encoders_valid"):
            with self.subTest(attribute=attribute):
                model = ChassisSimulation()
                model.accept(MotionCommand(1, 300, 100, 100), 0)
                setattr(model, attribute, attribute == "emergency_stop")
                model.advance(0.1)
                self.assertEqual(model.snapshot().left_mm_s, 0)
                setattr(model, attribute, attribute != "emergency_stop")
                model.advance(0.2)
                self.assertEqual(model.snapshot().left_mm_s, 0)

    def test_invalid_geometry_and_clock(self):
        for diameter in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                ChassisSimulation(diameter)
        model = ChassisSimulation()
        model.advance(1)
        for now in (0.5, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                model.advance(now)


class SimulatorSocketTests(unittest.TestCase):
    def read_frame(self, client, predicate=lambda _: True):
        decoder = TelemetryDecoder()
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            for frame in decoder.feed(client.recv(4096)):
                if predicate(frame):
                    return frame
        self.fail("telemetry did not arrive")

    def test_socket_stream_reconnect_preserves_command_sequence(self):
        with ChassisSimulatorServer() as server:
            with socket.create_connection((server.host, server.port), timeout=1) as client:
                first = self.read_frame(client)
                self.assertFalse(first.status & TelemetryStatus.LAST_COMMAND_VALID)
                wire = encode_motion_command(MotionCommand(1234, 100, 50, 50))
                client.sendall(b"noise" + wire[:7])
                client.sendall(wire[7:])
                self.read_frame(client, lambda frame: frame.last_command_sequence == 1234)
            with socket.create_connection((server.host, server.port), timeout=1) as client:
                frame = self.read_frame(client)
                self.assertEqual(frame.last_command_sequence, 1234)
                self.assertTrue(frame.status & TelemetryStatus.LAST_COMMAND_VALID)
            time.sleep(0.15)
            self.assertEqual(server.snapshot().left_mm_s, 0)
            self.assertIsNone(server.error)

    def test_dropped_commands_are_not_acknowledged(self):
        with ChassisSimulatorServer(drop_commands=True) as server:
            with socket.create_connection((server.host, server.port), timeout=1) as client:
                client.sendall(encode_motion_command(MotionCommand(1, 300, 50, 50)))
                frame = self.read_frame(client)
                self.assertFalse(frame.status & TelemetryStatus.LAST_COMMAND_VALID)
                self.assertEqual(server.snapshot().left_mm_s, 0)


if __name__ == "__main__":
    unittest.main()
