from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from motion import Client, Op, Request
from drive import KeyboardState


class MotionBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory(prefix="chassis-motion-bridge-")
        cls.exe = Path(cls.tmp.name) / "bridge.exe"
        compiler = shutil.which("gcc")
        if not compiler:
            raise RuntimeError("gcc required")
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                        "-I", str(ROOT / "firmware/common/include"),
                        *(str(ROOT / f"firmware/common/src/{name}.c") for name in
                          ("diagnostic_protocol", "motion_protocol", "motion_control", "motion_bridge")),
                        str(ROOT / "tests/motion_bridge_native.c"), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_actual_bridge_session_replay_timeout_reset_and_controller(self):
        result = subprocess.run([str(self.exe)], capture_output=True, text=True, check=True)
        self.assertIn("bridge tests passed", result.stdout)

    def run_transport(self, action):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)
        listener.settimeout(3)
        observed, errors = [], []

        def server():
            try:
                with subprocess.Popen([str(self.exe), "--stream"], stdin=subprocess.PIPE,
                                      stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as core:
                    connection, _ = listener.accept()
                    with connection:
                        connection.settimeout(2)
                        while True:
                            packet = bytearray()
                            while len(packet) < 32:
                                part = connection.recv(32 - len(packet))
                                if not part:
                                    return
                                packet.extend(part)
                            observed.append(Request.decode(packet))
                            core.stdin.write(packet.hex() + "\n")
                            core.stdin.flush()
                            line = core.stdout.readline().strip()
                            if line == "DROP":
                                return
                            reply = bytes.fromhex(line)
                            if len(reply) != 40:
                                raise RuntimeError("native bridge failed: " + line)
                            connection.sendall(reply[:7])
                            connection.sendall(reply[7:])
                    core.stdin.close()
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        peer = Client(*listener.getsockname())
        try:
            action(peer)
        finally:
            peer.close()
            listener.close()
            thread.join(timeout=3)
        self.assertFalse(thread.is_alive(), "transport thread did not stop")
        self.assertFalse(errors, errors)
        return observed

    def test_pc_to_bridge_to_stm32_all_directions_and_stop(self):
        def action(peer):
            peer.hello()
            self.assertFalse(peer.armed)
            peer.arm()
            state = KeyboardState()
            state.enabled = True
            for key in ("w", "s", "a", "d"):
                state.keys = {key}
                left, right = state.target(100)
                reply = peer.request(Op.DRIVE, left, right)
                self.assertEqual(reply.result, 0)
                self.assertTrue(peer.armed)
            reply = peer.stop()
            self.assertEqual((reply.left, reply.right, reply.token, reply.flags), (0, 0, 0, 0))

        observed = self.run_transport(action)
        self.assertEqual([(r.left, r.right) for r in observed if r.opcode == Op.DRIVE],
                         [(100, 100), (-100, -100), (-100, 100), (100, -100)])

    def test_pc_cannot_arm_before_hello(self):
        def action(peer):
            with self.assertRaises((ConnectionError, OSError)):
                peer.request(Op.ARM)
        self.run_transport(action)

    def test_arrow_mouse_release_and_ui_stall(self):
        state = KeyboardState()
        state.enabled = True
        for key, target in (("up", (100, 100)), ("down", (-100, -100)),
                            ("left", (-100, 100)), ("right", (100, -100)),
                            ("mouse:w", (100, 100))):
            state.keys = {key}
            self.assertEqual(state.target(100), target)
        state.keys = {"up", "w"}
        state.keys.remove("w")
        self.assertEqual(state.target(100), (100, 100))
        state.keys.add("mouse:s")
        self.assertEqual(state.target(100), (0, 0))
        state.keys.clear()
        self.assertEqual(state.target(100), (0, 0))
        state.keys.add("w")
        state.last_ui_poll = 10
        self.assertFalse(state.expire_ui(10.19))
        self.assertTrue(state.expire_ui(10.21))
        self.assertFalse(state.enabled)
        self.assertFalse(state.keys)
        self.assertEqual(state.target(100), (0, 0))


if __name__ == "__main__":
    unittest.main()
