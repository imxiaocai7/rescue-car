import io
import unittest
from contextlib import redirect_stderr

from chassis_simulator import ChassisSimulatorServer
from motion_link import MotionLinkClient
from motion_protocol import ActuatorCommand, MotionCommand
from probe_motion import CommunicationOnlySimulation, ZeroProbeSession, make_demo_server, parse_args, run_zero_probe
from rover_telemetry import INVALID_RANGE_MM, RoverTelemetry, TelemetryStatus


def frame(sequence=1, tick=0, ack=None, status=TelemetryStatus.EMERGENCY_STOP):
    if ack is not None:
        status |= TelemetryStatus.LAST_COMMAND_VALID
    return RoverTelemetry(sequence, tick, ack if ack is not None else 0, 0, 0,
                          0, INVALID_RANGE_MM, INVALID_RANGE_MM, 0, status)


class ZeroProbeTests(unittest.TestCase):
    def test_session_only_generates_zero_and_waits_for_ack(self):
        session = ZeroProbeSession(0)
        self.assertIsNone(session.poll(0))
        session.observe(frame(ack=987), 0)
        command = session.poll(0)
        self.assertEqual(command, MotionCommand(988, 300, 0, 0, ActuatorCommand.HOLD))
        self.assertIsNone(session.poll(0.05))
        session.observe(frame(2, 50, 988), 0.05)
        self.assertEqual(session.acknowledged_zero_frames, 1)
        self.assertEqual(session.max_ack_round_trip_ms, 50)
        self.assertIsNotNone(session.poll(0.05))

    def test_first_and_ack_timeouts(self):
        session = ZeroProbeSession(0)
        with self.assertRaises(TimeoutError):
            session.poll(0.81)
        session = ZeroProbeSession(0)
        session.observe(frame(), 0)
        session.poll(0)
        with self.assertRaises(TimeoutError):
            session.observe(frame(2, 201, 1), 0.201)

    def test_wrong_profile_rejected_before_any_command(self):
        for status in (TelemetryStatus(0), TelemetryStatus.EMERGENCY_STOP | TelemetryStatus.LEFT_ENCODER_VALID):
            session = ZeroProbeSession(0)
            with self.assertRaises(RuntimeError):
                session.observe(frame(status=status), 0)
            self.assertIsNone(session.next_sequence)

    def test_reset_duplicate_and_sequence_wrap(self):
        session = ZeroProbeSession(0)
        session.observe(frame(0xFFFFFFFF, 0xFFFFFFF0, 0xFFFFFFFF), 0)
        command = session.poll(0)
        self.assertEqual(command.sequence, 0)
        session.observe(frame(0, 34, 0), 0.05)
        session.observe(frame(0, 34, 0), 0.1)
        self.assertEqual(session.received_frames, 2)
        self.assertEqual(session.received_s, 0.05)
        with self.assertRaises(RuntimeError):
            session.observe(frame(0, 0), 0.15)

    def test_other_writer_rejected(self):
        session = ZeroProbeSession(0)
        session.observe(frame(), 0)
        session.poll(0)
        with self.assertRaises(RuntimeError):
            session.observe(frame(2, 50, 1000), 0.05)

    def test_demo_refuses_unsafe_commands(self):
        simulation = CommunicationOnlySimulation()
        self.assertFalse(simulation.accept(MotionCommand(1, 300, 1, 0), 0))
        self.assertFalse(simulation.accept(MotionCommand(1, 300, 0, 0, ActuatorCommand.OPEN), 0))
        self.assertIsNone(simulation.last_sequence)
        self.assertTrue(simulation.accept(MotionCommand(1, 300, 0, 0), 0))
        self.assertEqual(simulation.snapshot().left_mm_s, 0)

    def test_cli_hardware_gate_and_bad_arguments(self):
        self.assertIsNone(parse_args([]).host)
        for args in (["--host", "192.168.4.1"], ["--duration", "nan"],
                     ["--duration", "0"], ["--port", "5001"]):
            with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(args)
        self.assertEqual(parse_args(["--host", "192.168.4.1", "--allow-hardware"]).host, "192.168.4.1")

    def test_local_tcp_probe_and_reconnect(self):
        with make_demo_server() as server:
            previous_ack = None
            for _ in range(2):
                report = run_zero_probe(MotionLinkClient(server.host, server.port), 0.3)
                self.assertGreater(report.acknowledged_zero_frames, 0)
                self.assertEqual(report.sent_zero_frames, report.acknowledged_zero_frames)
                self.assertFalse(report.hardware_verified)
                if previous_ack is not None:
                    self.assertGreater(report.last_confirmed_sequence, previous_ack)
                previous_ack = report.last_confirmed_sequence
                self.assertEqual(server.snapshot().left_mm_s, 0)

    def test_live_tcp_without_ack_fails(self):
        with make_demo_server(drop_commands=True) as server:
            with self.assertRaisesRegex(TimeoutError, "not acknowledged"):
                run_zero_probe(MotionLinkClient(server.host, server.port), 0.4)

    def test_regular_motion_simulator_is_not_commissioning_profile(self):
        with ChassisSimulatorServer() as server:
            with self.assertRaisesRegex(RuntimeError, "profile"):
                run_zero_probe(MotionLinkClient(server.host, server.port), 0.3)
            self.assertIsNone(server.snapshot().last_command_sequence)

    def test_motion_ui_must_remain_locked_with_phase2_profile(self):
        from manual_chassis import ControlState, ManualChassisController
        controller = ManualChassisController()
        controller.begin_connection(0)
        controller.observe(frame(), 0)
        self.assertEqual(controller.state, ControlState.FAULT)
        self.assertFalse(controller.arm(0))
        self.assertIsNone(controller.poll(0))

    def test_live_socket_with_no_telemetry_times_out_and_closes(self):
        with make_demo_server(mute_telemetry_after_s=0) as server:
            link = MotionLinkClient(server.host, server.port)
            with self.assertRaisesRegex(TimeoutError, "no STM32 telemetry"):
                run_zero_probe(link, 1.0)
            self.assertFalse(link.connected)
            self.assertIsNone(server.snapshot().last_command_sequence)


if __name__ == "__main__":
    unittest.main()
