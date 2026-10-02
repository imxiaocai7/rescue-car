import io
import socket
import time
import unittest
from contextlib import redirect_stderr
from unittest.mock import Mock

from chassis_simulator import ChassisSimulatorServer
from control_chassis import parse_args
from manual_chassis import ControlState, ManualChassisController, ManualMotionWorker
from motion_link import MotionLinkClient
from rover_telemetry import INVALID_RANGE_MM, RoverTelemetry, TelemetryStatus


ENCODERS = TelemetryStatus.LEFT_ENCODER_VALID | TelemetryStatus.RIGHT_ENCODER_VALID


def telemetry(seq=1, tick=0, ack=None, *, status=ENCODERS, left=0, right=0):
    if ack is not None:
        status |= TelemetryStatus.LAST_COMMAND_VALID
    return RoverTelemetry(seq, tick, ack if ack is not None else 0, left, right,
                          0, INVALID_RANGE_MM, INVALID_RANGE_MM, 0, status)


def ready_controller(**kwargs):
    controller = ManualChassisController(**kwargs)
    controller.begin_connection(0)
    controller.observe(telemetry(), 0)
    zero = controller.poll(0)
    controller.observe(telemetry(2, 50, zero.sequence), 0.05)
    return controller


class ManualControllerTests(unittest.TestCase):
    def test_connection_requires_zero_ack_and_explicit_arm(self):
        controller = ManualChassisController()
        controller.begin_connection(0)
        controller.press("w")
        self.assertIsNone(controller.poll(0))
        self.assertFalse(controller.arm(0))
        controller.observe(telemetry(ack=9000), 0.01)
        zero = controller.poll(0.01)
        self.assertEqual((zero.sequence, zero.left_mm_s, zero.right_mm_s), (9001, 0, 0))
        self.assertFalse(controller.arm(0.02))
        controller.observe(telemetry(2, 50, zero.sequence), 0.05)
        self.assertTrue(controller.status(0.05).can_arm)
        self.assertTrue(controller.arm(0.05))
        self.assertEqual(controller.desired_speeds(), (0, 0))

    def test_wasd_chords_and_opposing_keys(self):
        controller = ready_controller(speed_mm_s=80)
        controller.arm(0.05)
        for keys, expected in (
            ("w", (80, 80)), ("s", (-80, -80)), ("a", (-80, 80)), ("d", (80, -80)),
            ("wa", (0, 80)), ("wd", (80, 0)), ("sa", (-80, 0)), ("sd", (0, -80)),
            ("ws", (0, 0)), ("ad", (0, 0)), ("wsa", (0, 0)), ("", (0, 0)),
        ):
            with self.subTest(keys=keys):
                controller.keys.clear()
                for key in keys:
                    controller.press(key)
                self.assertEqual(controller.desired_speeds(), expected)

    def test_speed_validation_and_unmeasured_feedback(self):
        for speed in (0, -1, 301, 80.5, True):
            with self.assertRaises(ValueError):
                ManualChassisController(speed)
        for diameter in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                ManualChassisController(wheel_diameter_mm=diameter)
        controller = ready_controller()
        controller.observe(telemetry(3, 100, 1, left=10, right=10), 0.1)
        self.assertIsNone(controller.measured_speeds)

    def test_speed_estimate_uses_ticks_and_signed_count_wrap(self):
        controller = ready_controller(wheel_diameter_mm=75)
        controller.observe(telemetry(3, 100, 1, left=2**31 - 5, right=0), 0.1)
        controller.observe(telemetry(4, 150, 1, left=-(2**31) + 5, right=-10), 0.15)
        expected = 10 * controller.mm_per_count / 0.05
        self.assertAlmostEqual(controller.measured_speeds[0], expected)
        self.assertAlmostEqual(controller.measured_speeds[1], -expected)

    def test_no_unacknowledged_motion_queue_and_release_preempts(self):
        controller = ready_controller()
        controller.arm(0.05)
        controller.press("w")
        motion = controller.poll(0.05)
        self.assertEqual((motion.left_mm_s, motion.right_mm_s), (80, 80))
        self.assertIsNone(controller.poll(0.10))
        self.assertIsNone(controller.poll(0.15))
        controller.release("w")
        zero = controller.poll(0.151)
        self.assertEqual((zero.left_mm_s, zero.right_mm_s), (0, 0))
        self.assertEqual(zero.sequence, motion.sequence + 1)
        controller.observe(telemetry(3, 160, motion.sequence), 0.16)
        self.assertEqual(controller.pending.command.sequence, zero.sequence)
        controller.observe(telemetry(4, 200, zero.sequence), 0.20)
        self.assertIsNone(controller.pending)

    def test_disarm_clears_keys_and_requires_new_zero_ack(self):
        controller = ready_controller()
        controller.arm(0.05)
        controller.press("w")
        controller.poll(0.05)
        controller.disarm("窗口失去焦点")
        self.assertEqual(controller.desired_speeds(), (0, 0))
        self.assertFalse(controller.arm(0.06))
        zero = controller.poll(0.06)
        controller.observe(telemetry(3, 100, zero.sequence), 0.1)
        self.assertTrue(controller.arm(0.1))
        self.assertEqual(controller.desired_speeds(), (0, 0))

    def test_missing_telemetry_and_duplicate_freshness(self):
        controller = ManualChassisController()
        controller.begin_connection(0)
        self.assertIsNone(controller.poll(0.81))
        self.assertEqual(controller.state, ControlState.FAULT)
        controller = ready_controller()
        controller.observe(telemetry(2, 50, 1), 0.3)
        self.assertEqual(controller.received_s, 0.05)
        controller.poll(0.36)
        self.assertEqual(controller.state, ControlState.FAULT)
        self.assertIn("遥测超时", controller.reason)

    def test_command_ack_timeout_despite_live_telemetry(self):
        controller = ready_controller()
        controller.arm(0.05)
        controller.press("w")
        controller.poll(0.05)
        controller.observe(telemetry(3, 240, 1), 0.24)
        controller.input_alive(0.24)
        self.assertIsNone(controller.poll(0.26))
        self.assertEqual(controller.state, ControlState.FAULT)
        self.assertIn("未被 STM32 确认", controller.reason)
        self.assertEqual(controller.final_zero().left_mm_s, 0)

    def test_ui_heartbeat_missing_stops_even_with_fresh_ack(self):
        controller = ready_controller()
        controller.arm(0.05)
        controller.press("w")
        motion = controller.poll(0.05)
        controller.observe(telemetry(3, 100, motion.sequence), 0.1)
        controller.observe(telemetry(4, 320, motion.sequence), 0.32)
        controller.poll(0.32)
        self.assertEqual(controller.state, ControlState.FAULT)
        self.assertIn("界面无响应", controller.reason)

    def test_late_ack_or_feedback_cannot_clear_timeout(self):
        controller = ready_controller()
        controller.poll(0.05)
        controller.observe(telemetry(3, 270, 2), 0.27)
        self.assertEqual(controller.state, ControlState.FAULT)
        controller = ready_controller()
        controller.observe(telemetry(3, 400, 1), 0.4)
        self.assertEqual(controller.state, ControlState.FAULT)

    def test_held_key_auto_repeat_after_arm_is_blocked_until_release(self):
        controller = ready_controller()
        controller.press("w")
        self.assertTrue(controller.arm(0.05))
        controller.press("w")
        self.assertEqual(controller.desired_speeds(), (0, 0))
        controller.release("w")
        controller.press("w")
        self.assertEqual(controller.desired_speeds(), (80, 80))

    def test_emergency_and_invalid_encoders_block_arm(self):
        for flags in (ENCODERS | TelemetryStatus.EMERGENCY_STOP,
                      TelemetryStatus.LEFT_ENCODER_VALID, TelemetryStatus(0)):
            with self.subTest(flags=flags):
                controller = ManualChassisController()
                controller.begin_connection(0)
                controller.observe(telemetry(status=flags), 0.01)
                self.assertEqual(controller.state, ControlState.FAULT)
                self.assertFalse(controller.arm(0.01))
                self.assertIsNone(controller.poll(0.01))

    def test_emergency_during_motion_clears_target_and_latches_fault(self):
        controller = ready_controller()
        controller.arm(0.05)
        controller.press("w")
        motion = controller.poll(0.05)
        controller.observe(telemetry(3, 100, motion.sequence,
                                     status=ENCODERS | TelemetryStatus.EMERGENCY_STOP), 0.1)
        self.assertEqual(controller.state, ControlState.FAULT)
        self.assertEqual(controller.desired_speeds(), (0, 0))
        controller.observe(telemetry(4, 150, motion.sequence), 0.15)
        self.assertFalse(controller.arm(0.15))

    def test_reset_locks_and_reconnect_reseeds_without_motion(self):
        controller = ready_controller()
        controller.arm(0.05)
        controller.press("w")
        controller.observe(telemetry(0, 0), 0.1)
        self.assertEqual(controller.state, ControlState.FAULT)
        self.assertEqual(controller.desired_speeds(), (0, 0))
        controller.begin_connection(1.0)
        controller.observe(telemetry(10, 1000, 4321), 1.0)
        zero = controller.poll(1.0)
        self.assertEqual(zero.sequence, 4322)
        self.assertEqual(zero.left_mm_s, 0)
        controller.observe(telemetry(11, 1050, zero.sequence), 1.05)
        self.assertEqual(controller.state, ControlState.DISARMED)

    def test_sequence_and_tick_wrap_are_not_reset(self):
        controller = ManualChassisController()
        controller.begin_connection(0)
        controller.observe(telemetry(0xFFFFFFFF, 0xFFFFFFF0, 0xFFFFFFFF), 0)
        zero = controller.poll(0)
        self.assertEqual(zero.sequence, 0)
        controller.observe(telemetry(0, 34, 0), 0.05)
        self.assertEqual(controller.state, ControlState.DISARMED)
        self.assertTrue(controller.arm(0.05))

    def test_another_command_writer_locks_control(self):
        controller = ready_controller()
        controller.poll(0.05)
        controller.observe(telemetry(3, 100, 1000), 0.1)
        self.assertEqual(controller.state, ControlState.FAULT)

    def test_send_rate_not_exceeded_with_fast_ack(self):
        controller = ready_controller()
        command = controller.poll(0.05)
        controller.observe(telemetry(3, 60, command.sequence), 0.06)
        self.assertIsNone(controller.poll(0.07))
        self.assertIsNone(controller.poll(0.099))
        self.assertIsNotNone(controller.poll(0.101))


class BoundedSendTests(unittest.TestCase):
    def test_invalid_send_timeout(self):
        from motion_protocol import MotionCommand
        for timeout in (0, -1, float("inf"), float("nan")):
            with self.assertRaises(ValueError):
                MotionLinkClient().send_command(MotionCommand(1, 300, 0, 0), timeout_s=timeout)

    def test_timeout_restored_on_success(self):
        link = MotionLinkClient()
        connection = Mock()
        connection.gettimeout.return_value = None
        link._socket = connection
        from motion_protocol import MotionCommand
        link.send_command(MotionCommand(1, 300, 0, 0), timeout_s=0.05)
        self.assertEqual(connection.settimeout.call_args_list[0].args, (0.05,))
        self.assertEqual(connection.settimeout.call_args_list[-1].args, (None,))
        self.assertTrue(link.connected)

    def test_send_timeout_closes_connection(self):
        link = MotionLinkClient()
        connection = Mock()
        connection.gettimeout.return_value = None
        connection.sendall.side_effect = socket.timeout("blocked send")
        link._socket = connection
        from motion_protocol import MotionCommand
        with self.assertRaises(socket.timeout):
            link.send_command(MotionCommand(1, 300, 0, 0), timeout_s=0.05)
        self.assertFalse(link.connected)
        connection.close.assert_called_once()


class CliSafetyTests(unittest.TestCase):
    def test_default_is_demo_and_nonloopback_needs_explicit_flag(self):
        self.assertIsNone(parse_args([]).host)
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--host", "192.168.4.1"])
        self.assertEqual(parse_args(["--host", "192.168.4.1", "--allow-hardware"]).host, "192.168.4.1")
        self.assertEqual(parse_args(["--host", "127.0.0.1", "--port", "5501"]).port, 5501)

    def test_invalid_arguments(self):
        for args in (["--port", "5001"], ["--allow-hardware"], ["--speed-mm-s", "301"],
                     ["--wheel-diameter-mm", "nan"], ["--host", "example.com"],
                     ["--host", "127.0.0.1", "--port", "0"]):
            with self.subTest(args=args), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(args)


class ManualWorkerIntegrationTests(unittest.TestCase):
    def wait_until(self, worker, predicate, *, heartbeat=True, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with worker.lock:
                if heartbeat:
                    worker.controller.input_alive(time.monotonic())
                if predicate(worker.controller):
                    return
            time.sleep(0.01)
        with worker.lock:
            self.fail(f"condition timed out: {worker.controller.state}, {worker.controller.reason}")

    def start_ready(self, server):
        worker = ManualMotionWorker(ManualChassisController(wheel_diameter_mm=75), server.host, server.port)
        self.addCleanup(worker.close)
        worker.start()
        self.wait_until(worker, lambda controller: controller.status(time.monotonic()).can_arm)
        return worker

    def test_real_local_tcp_move_release_disconnect_and_reconnect(self):
        with ChassisSimulatorServer() as server:
            worker = self.start_ready(server)
            with worker.lock:
                self.assertTrue(worker.controller.arm(time.monotonic()))
                worker.controller.press("w")
            self.wait_until(worker, lambda _: server.snapshot().left_mm_s == 80)
            self.wait_until(worker, lambda controller: controller.measured_speeds is not None
                            and controller.measured_speeds[0] > 0)
            with worker.lock:
                worker.controller.release("w")
            self.wait_until(worker, lambda _: server.snapshot().left_mm_s == 0)
            previous_seq = server.snapshot().last_command_sequence
            server.disconnect_client()
            self.wait_until(worker, lambda controller: controller.state == ControlState.FAULT)
            worker.close()
            worker.start()
            self.wait_until(worker, lambda controller: controller.status(time.monotonic()).can_arm)
            self.assertEqual(worker.controller.state, ControlState.DISARMED)
            self.assertEqual(server.snapshot().left_mm_s, 0)
            self.assertGreater(server.snapshot().last_command_sequence, previous_seq)
            worker.close()

    def test_muted_feedback_stops_and_locks(self):
        with ChassisSimulatorServer() as server:
            worker = self.start_ready(server)
            with worker.lock:
                worker.controller.arm(time.monotonic())
                worker.controller.press("w")
            self.wait_until(worker, lambda _: server.snapshot().left_mm_s == 80)
            server.mute_telemetry.set()
            self.wait_until(worker, lambda controller: controller.state == ControlState.FAULT)
            self.wait_until(worker, lambda _: server.snapshot().left_mm_s == 0)
            self.assertFalse(worker.controller.arm(time.monotonic()))
            worker.close()

    def test_missing_ack_never_enables_motion(self):
        with ChassisSimulatorServer(drop_commands=True) as server:
            worker = ManualMotionWorker(ManualChassisController(), server.host, server.port)
            self.addCleanup(worker.close)
            worker.start()
            self.wait_until(worker, lambda controller: controller.state == ControlState.FAULT)
            self.assertIn("未被 STM32 确认", worker.controller.reason)
            self.assertEqual(server.snapshot().left_mm_s, 0)
            worker.close()

    def test_hung_ui_stops_with_live_tcp(self):
        with ChassisSimulatorServer() as server:
            worker = self.start_ready(server)
            with worker.lock:
                worker.controller.arm(time.monotonic())
                worker.controller.press("w")
            self.wait_until(worker, lambda _: server.snapshot().left_mm_s == 80)
            self.wait_until(worker, lambda controller: controller.state == ControlState.FAULT, heartbeat=False)
            self.assertIn("界面无响应", worker.controller.reason)
            self.wait_until(worker, lambda _: server.snapshot().left_mm_s == 0)
            worker.close()


if __name__ == "__main__":
    unittest.main()
