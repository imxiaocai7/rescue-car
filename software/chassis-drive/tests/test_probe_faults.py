import contextlib
from pathlib import Path
import socket
import sys
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "pc"))
from protocol import Opcode, Reply, StreamParser, Request
from probe import probe


@contextlib.contextmanager
def scripted_peer(responder):
    listener = socket.socket()
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    listener.settimeout(1)
    stopped = threading.Event()
    errors = []

    def serve():
        try:
            client, _ = listener.accept()
            with client:
                client.settimeout(0.05)
                parser = StreamParser(Request)
                index = 0
                while not stopped.is_set():
                    try:
                        data = client.recv(256)
                    except socket.timeout:
                        continue
                    if not data:
                        break
                    for request in parser.feed(data):
                        answer = responder(request, index)
                        index += 1
                        if answer == "close":
                            return
                        if answer is not None:
                            client.sendall(answer)
        except OSError:
            if not stopped.is_set():
                errors.append("unexpected socket failure")
        except Exception as exc:
            errors.append(repr(exc))

    thread = threading.Thread(target=serve, daemon=True)
    thread.start()
    try:
        yield listener.getsockname()
    finally:
        stopped.set()
        listener.close()
        thread.join(timeout=1)
        if thread.is_alive() or errors:
            raise RuntimeError(f"scripted peer failed: {errors}")


class ProbeFaultTests(unittest.TestCase):
    def test_no_confirmation_times_out(self):
        with scripted_peer(lambda request, index: None) as address:
            with self.assertRaises(TimeoutError):
                probe(*address, count=1, timeout_s=0.05)

    def test_disconnect_fails(self):
        with scripted_peer(lambda request, index: "close") as address:
            with self.assertRaises(ConnectionError):
                probe(*address, count=1)

    def test_wrong_sequence_fails(self):
        def reply(request, index):
            return Reply((request.sequence + 1) & 0xFFFFFFFF, 0).encode()
        with scripted_peer(reply) as address:
            with self.assertRaisesRegex(RuntimeError, "sequence"):
                probe(*address, count=1)

    def test_wrong_firmware_fails(self):
        with scripted_peer(lambda request, index: Reply(request.sequence, 0, firmware_id=0).encode()) as address:
            with self.assertRaisesRegex(RuntimeError, "firmware"):
                probe(*address, count=1)

    def test_wrong_flags_fails(self):
        with scripted_peer(lambda request, index: Reply(request.sequence, 0, flags=0).encode()) as address:
            with self.assertRaisesRegex(RuntimeError, "firmware"):
                probe(*address, count=1)

    def test_reset_fails(self):
        with scripted_peer(lambda request, index: Reply(request.sequence, 100 if index == 0 else 0).encode()) as address:
            with self.assertRaisesRegex(RuntimeError, "restarted"):
                probe(*address, count=2)

    def test_garbage_and_bad_crc_then_valid(self):
        def reply(request, index):
            packet = Reply(request.sequence, index * 100).encode()
            corrupt = bytearray(packet)
            corrupt[-1] ^= 1
            return b"garbage" + corrupt + packet
        with scripted_peer(reply) as address:
            self.assertEqual(probe(*address, count=2)["confirmed_requests"], 2)

    def test_last_request_is_stop(self):
        observed = []
        def reply(request, index):
            observed.append(request.opcode)
            return Reply(request.sequence, index).encode()
        with scripted_peer(reply) as address:
            probe(*address, count=2)
        self.assertEqual(observed, [Opcode.PING, Opcode.STOP])


if __name__ == "__main__":
    unittest.main()
