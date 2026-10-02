"""Hidden Tk smoke tests, always connected to a localhost simulator only."""

import time
import tkinter as tk
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from chassis_simulator import ChassisSimulatorServer
from control_chassis import ManualControlApp
from manual_chassis import ControlState


class ManualControlGuiTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(f"Tk display unavailable: {exc}")
        self.root.withdraw()
        self.server = ChassisSimulatorServer().start()
        self.app = ManualControlApp(
            self.root, self.server.host, self.server.port, demo=True,
            wheel_diameter_mm=75, simulator=self.server,
        )

    def tearDown(self):
        if hasattr(self, "app"):
            self.app.close()
            self.wait_until(lambda: self.app._closed)
            self.app.worker.close()
            self.server.close()

    def wait_until(self, predicate, timeout=2):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.root.update()
            if predicate():
                return
            time.sleep(0.01)
        with self.app.worker.lock:
            status = self.app.controller.status(time.monotonic())
        self.fail(f"GUI/simulator condition timed out: {status}")

    def ready(self):
        self.wait_until(lambda: str(self.app.arm_button.cget("state")) == "normal")
        self.assertIn("未允许运动", self.app.state_text.get())
        self.app.arm_button.invoke()
        self.assertEqual(str(self.app.arm_button.cget("state")), "disabled")
        self.wait_until(lambda: self.app.controller.state == ControlState.ARMED)

    def test_keyboard_mouse_speed_stop_and_status(self):
        self.ready()
        self.app._key_press(SimpleNamespace(keysym="w"))
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 80)
        self.assertIn("mm/s", self.app.targets_text.get())
        self.app._key_release(SimpleNamespace(keysym="w"))
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 0)
        self.app._change_speed("100")
        self.app.direction_buttons[1].event_generate("<ButtonPress-1>")
        self.wait_until(lambda: self.server.snapshot().left_mm_s == -100)
        self.app.direction_buttons[1].event_generate("<ButtonRelease-1>")
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 0)
        self.app._key_press(SimpleNamespace(keysym="d"))
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 100)
        self.app._key_press(SimpleNamespace(keysym="space"))
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 0)
        self.assertEqual(self.app.controller.state, ControlState.DISARMED)

    def test_focus_loss_and_close_while_moving(self):
        self.ready()
        self.app._key_press(SimpleNamespace(keysym="w"))
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 80)
        with patch.object(self.root, "focus_displayof", return_value=None):
            self.app._focus_out(None)
            self.wait_until(lambda: self.app.controller.state == ControlState.DISARMED)
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 0)
        self.app._key_release(SimpleNamespace(keysym="w"))
        self.wait_until(lambda: str(self.app.arm_button.cget("state")) == "normal"
                        and self.app.controller.status(time.monotonic()).can_arm)
        self.app.arm_button.invoke()
        self.assertEqual(self.app.controller.state, ControlState.ARMED)
        self.app._key_press(SimpleNamespace(keysym="w"))
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 80)
        self.app.close()
        self.wait_until(lambda: self.app._closed)
        # Closing a stream does not guarantee the final zero was delivered;
        # the independent simulated lease must also stop the old target.
        self.wait_until(lambda: self.server.snapshot().left_mm_s == 0)


if __name__ == "__main__":
    unittest.main()
