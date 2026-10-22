import contextlib
import io
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
from motion import Client, FIRMWARE_ID, Op, Reply, Request
from drive import KeyboardState, parse_args


class MotionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if not compiler:
            raise RuntimeError("gcc required")
        cls.tmp = tempfile.TemporaryDirectory(prefix="chassis-motion-")
        cls.exe = Path(cls.tmp.name) / "motion.exe"
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                        "-I", str(ROOT / "firmware/common/include"),
                        *(str(ROOT / f"firmware/common/src/{name}.c") for name in
                          ("diagnostic_protocol", "motion_protocol", "motion_control")),
                        str(ROOT / "tests/motion_native.c"), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_actual_firmware_core_failures_and_ramp(self):
        result = subprocess.run([str(self.exe)], capture_output=True, text=True, check=True)
        self.assertIn("tests passed", result.stdout)

    def test_python_c_wire_compatibility(self):
        for seq in (0, 0xFFFFFFFF, 0x1A0D0AFF):
            packet = Request(seq, 42, 0, Op.HELLO)
            self.assertEqual(Request.decode(packet.encode()), packet)
            result = subprocess.run([str(self.exe), packet.encode().hex()], capture_output=True, text=True, check=True)
            reply = Reply.decode(bytes.fromhex(result.stdout))
            self.assertEqual((reply.sequence, reply.session, reply.token, reply.result), (seq, 42, 1235, 0))
            self.assertEqual(reply.encode(), bytes.fromhex(result.stdout))

    def test_invalid_values_and_crc(self):
        for value in (-251, 251, True, 1.5):
            with self.assertRaises(ValueError):
                Request(1, 2, 3, Op.DRIVE, value, 0).encode()
        for op in (Op.ARM, Op.HELLO, Op.STATUS):
            with self.assertRaises(ValueError):
                Request(1, 2, 3, op, 1, 0).encode()
        packet = Request(1, 2, 3, Op.DRIVE, -250, 250).encode()
        for i in range(len(packet)):
            bad = bytearray(packet)
            bad[i] ^= 1
            with self.assertRaises(ValueError):
                Request.decode(bad)

    def test_keyboard_release_focus_loss_and_contradictory_keys(self):
        state = KeyboardState()
        state.keys.add("w")
        self.assertEqual(state.target(100), (0, 0))
        state.enabled = True
        self.assertEqual(state.target(100), (100, 100))
        state.keys.add("s")
        self.assertEqual(state.target(100), (0, 0))
        state.keys = {"d"}
        self.assertEqual(state.target(100), (100, -100))
        state.keys.clear()
        self.assertEqual(state.target(100), (0, 0))
        state.keys.add("w")
        state.stop()
        self.assertFalse(state.enabled)
        self.assertFalse(state.keys)

    def test_hardware_and_motion_opt_in(self):
        for args in (["--host", "192.168.4.1"], ["--host", "127.0.0.1", "--pulse"],
                     ["--host", "127.0.0.1", "--keyboard"],
                     ["--host", "127.0.0.1", "--duration", "nan"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                parse_args(args)
        args = parse_args(["--host", "127.0.0.1"])
        self.assertFalse(args.pulse or args.keyboard or args.allow_motion)

    def test_client_never_arms_on_connect_and_rejects_bad_confirmation(self):
        observed = []
        errors = []
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen(1)

        def server():
            try:
                connection, _ = listener.accept()
                with connection:
                    while True:
                        data = bytearray()
                        while len(data) < 32:
                            part = connection.recv(32 - len(data))
                            if not part:
                                return
                            data.extend(part)
                        request = Request.decode(data)
                        observed.append(request.opcode)
                        token = 123 if request.opcode == Op.HELLO else 0
                        sequence = request.sequence + (request.opcode == Op.STATUS)
                        reply = Reply(sequence & 0xFFFFFFFF, request.session, 50, FIRMWARE_ID,
                                      0, token, 0, 0, 0).encode()
                        connection.sendall(reply[:5])
                        connection.sendall(reply[5:])
            except Exception as exc:
                errors.append(exc)

        thread = threading.Thread(target=server, daemon=True)
        thread.start()
        peer = Client(*listener.getsockname())
        try:
            peer.hello()
            self.assertFalse(peer.armed)
            with self.assertRaisesRegex(RuntimeError, "sequence"):
                peer.request(Op.STATUS)
        finally:
            peer.close()
            listener.close()
            thread.join(timeout=1)
        self.assertEqual(observed, [Op.HELLO, Op.STATUS, Op.STOP])
        self.assertFalse(errors)


if __name__ == "__main__":
    unittest.main()
