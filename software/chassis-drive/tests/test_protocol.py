import contextlib
import io
from pathlib import Path
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from protocol import Opcode, Reply, Request, StreamParser, with_crc
from probe import DemoPeer, parse_args, probe


class ProtocolTests(unittest.TestCase):
    def test_request_round_trip(self):
        for seq in (0, 1, 0x1A0D0AFF, 0xFFFFFFFF):
            for opcode in Opcode:
                request = Request(seq, opcode)
                self.assertEqual(Request.decode(request.encode()), request)

    def test_reply_round_trip(self):
        reply = Reply(0xFFFFFFFF, 0xFFFFFFFF)
        self.assertEqual(Reply.decode(reply.encode()), reply)

    def test_request_every_byte_corruption(self):
        packet = Request(7).encode()
        for index in range(len(packet)):
            corrupt = bytearray(packet)
            corrupt[index] ^= 1
            with self.assertRaises(ValueError):
                Request.decode(corrupt)

    def test_reply_every_byte_corruption(self):
        packet = Reply(7, 3).encode()
        for index in range(len(packet)):
            corrupt = bytearray(packet)
            corrupt[index] ^= 1
            with self.assertRaises(ValueError):
                Reply.decode(corrupt)

    def test_unknown_opcode_and_reserved_rejected_even_with_correct_crc(self):
        for index, value in ((8, 3), (9, 1), (10, 1), (11, 1)):
            body = bytearray(Request(7).encode()[:-4])
            body[index] = value
            with self.assertRaises(ValueError):
                Request.decode(with_crc(body))

    def test_invalid_fields(self):
        for sequence in (-1, 0x100000000, True, 2.5):
            with self.assertRaises(ValueError):
                Request(sequence).encode()
        with self.assertRaises(ValueError):
            Request(1, 3).encode()

    def test_crc_known_vector(self):
        self.assertEqual(zlib.crc32(b"123456789"), 0xCBF43926)

    def test_split_concatenated_garbage_and_bad_crc(self):
        parser = StreamParser(Request)
        good = Request(5).encode()
        bad = bytearray(good)
        bad[-1] ^= 1
        stream = b"garbage" + bad + good + Request(6, Opcode.STOP).encode()
        results = []
        for byte in stream:
            results.extend(parser.feed(bytes([byte])))
        self.assertEqual(results, [Request(5), Request(6, Opcode.STOP)])

    def test_bounded_parser(self):
        parser = StreamParser(Reply)
        self.assertEqual(parser.feed(b"x" * 100000), [])
        self.assertLess(len(parser.buffer), 24)

    def test_hardware_gate(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--host", "192.168.4.1"])
        self.assertTrue(parse_args(["--host", "192.168.4.1", "--allow-hardware"]).allow_hardware)

    def test_default_is_demo(self):
        self.assertIsNone(parse_args([]).host)

    def test_local_tcp_and_reconnect(self):
        with DemoPeer() as peer:
            for _ in range(2):
                report = probe(peer.host, peer.port, count=2)
                self.assertEqual(report["confirmed_requests"], 2)
                self.assertFalse(report["hardware_verified"])
                self.assertFalse(report["motor_control_implemented"])

    def test_bad_probe_arguments(self):
        for kwargs in ({"count": 0}, {"count": 601}, {"timeout_s": float("nan")}):
            with self.assertRaises(ValueError):
                probe("127.0.0.1", 1, **kwargs)


class NativeCompatibilityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = shutil.which("gcc")
        if compiler is None:
            raise RuntimeError("gcc required for C compatibility verification")
        cls.temporary = tempfile.TemporaryDirectory(prefix="chassis-native-")
        cls.exe = Path(cls.temporary.name) / "protocol_test.exe"
        subprocess.run([compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic",
                        "-I", str(ROOT / "firmware/common/include"),
                        str(ROOT / "firmware/common/src/diagnostic_protocol.c"),
                        str(ROOT / "tests/protocol_native.c"), "-o", str(cls.exe)], check=True)

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_native_assertions(self):
        result = subprocess.run([str(self.exe)], capture_output=True, text=True, check=True)
        self.assertIn("tests passed", result.stdout)

    def test_python_requests_c_replies(self):
        for sequence in (0, 1, 0xFFFFFFFF, 0x1A0D0AFF):
            for opcode in Opcode:
                result = subprocess.run([str(self.exe), Request(sequence, opcode).encode().hex()],
                                        capture_output=True, text=True, check=True)
                self.assertEqual(Reply.decode(bytes.fromhex(result.stdout)), Reply(sequence, 1234))

    def test_c_rejects_unknown_opcode(self):
        body = bytearray(Request(1).encode()[:-4])
        body[8] = 3
        result = subprocess.run([str(self.exe), with_crc(body).hex()], capture_output=True)
        self.assertEqual(result.returncode, 2)


if __name__ == "__main__":
    unittest.main()
