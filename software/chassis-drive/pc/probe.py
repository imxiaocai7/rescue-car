"""Ping/stop-only TCP commissioning; no arguments means localhost simulation."""

import argparse
import ipaddress
import json
import secrets
import socket
import threading
import time

from protocol import FIRMWARE_ID, NO_MOTOR_IMPLEMENTATION, Opcode, Reply, Request, StreamParser
from protocol import MOTOR_FIRMWARE_ID, MOTOR_IMPLEMENTED


class DemoPeer:
    def __init__(self):
        self.listener = socket.socket()
        self.listener.bind(("127.0.0.1", 0))
        self.listener.listen(1)
        self.listener.settimeout(0.1)
        self.host, self.port = self.listener.getsockname()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.serve, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def serve(self):
        started = time.monotonic()
        while not self.stopping.is_set():
            try:
                client, _ = self.listener.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            with client:
                client.settimeout(0.1)
                parser = StreamParser(Request)
                while not self.stopping.is_set():
                    try:
                        data = client.recv(256)
                        if not data:
                            break
                        for request in parser.feed(data):
                            ticks = int((time.monotonic() - started) * 1000) & 0xFFFFFFFF
                            client.sendall(Reply(request.sequence, ticks).encode())
                    except socket.timeout:
                        continue
                    except OSError:
                        break

    def __exit__(self, *args):
        self.stopping.set()
        self.listener.close()
        self.thread.join(timeout=1)


def probe(host, port, count=10, timeout_s=0.5):
    if not isinstance(count, int) or not 1 <= count <= 600:
        raise ValueError("count must be 1..600")
    if not 0.05 <= timeout_s <= 5:
        raise ValueError("timeout must be 0.05..5 seconds")
    parser = StreamParser(Reply)
    sequence = secrets.randbits(32)
    last_ticks = None
    rtts = []
    implemented = False
    with socket.create_connection((host, port), timeout=1) as client:
        # Last request is STOP, though this firmware has no motor implementation.
        for index in range(count):
            request = Request(sequence, Opcode.STOP if index == count - 1 else Opcode.PING)
            started = time.monotonic()
            deadline = started + timeout_s
            client.settimeout(timeout_s)
            client.sendall(request.encode())
            received = None
            while received is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError("STM32 confirmation timed out")
                client.settimeout(remaining)
                data = client.recv(256)
                if not data:
                    raise ConnectionError("peer disconnected before confirmation")
                for reply in parser.feed(data):
                    if reply.sequence != sequence:
                        raise RuntimeError("unexpected sequence; stop and reconnect")
                    if (reply.firmware_id, reply.flags) not in (
                        (FIRMWARE_ID, NO_MOTOR_IMPLEMENTATION), (MOTOR_FIRMWARE_ID, MOTOR_IMPLEMENTED)
                    ):
                        raise RuntimeError("not a supported chassis firmware")
                    implemented = reply.flags == MOTOR_IMPLEMENTED
                    if received is not None:
                        raise RuntimeError("duplicate confirmation")
                    received = reply
                if time.monotonic() >= deadline:
                    raise TimeoutError("late confirmation")
            if last_ticks is not None and ((received.uptime_ms - last_ticks) & 0xFFFFFFFF) >= 0x80000000:
                raise RuntimeError("STM32 restarted or clock regressed")
            last_ticks = received.uptime_ms
            rtts.append((time.monotonic() - started) * 1000)
            sequence = (sequence + 1) & 0xFFFFFFFF
            if index != count - 1:
                time.sleep(0.1)
    return {"confirmed_requests": len(rtts), "last_stm32_uptime_ms": last_ticks,
            "max_round_trip_ms": round(max(rtts), 3), "hardware_verified": False,
            "motor_control_implemented": implemented}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    peer = parser.add_mutually_exclusive_group()
    peer.add_argument("--demo", action="store_true")
    peer.add_argument("--host")
    parser.add_argument("--allow-hardware", action="store_true")
    parser.add_argument("--port", type=int, default=6001)
    parser.add_argument("--count", type=int, default=10)
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535 or not 1 <= args.count <= 600:
        parser.error("invalid port/count")
    if args.host:
        try:
            address = ipaddress.IPv4Address(args.host)
        except ipaddress.AddressValueError:
            parser.error("host must be numeric IPv4")
        if not address.is_loopback and not args.allow_hardware:
            parser.error("non-loopback connection requires --allow-hardware")
    elif args.allow_hardware:
        parser.error("--allow-hardware requires --host")
    return args


def main():
    args = parse_args()
    try:
        if args.host:
            report = probe(args.host, args.port, args.count)
            report["mode"] = "external-peer"
        else:
            with DemoPeer() as peer:
                report = probe(peer.host, peer.port, args.count)
            report["mode"] = "simulation"
        print(json.dumps(report, indent=2))
    except (OSError, RuntimeError, ValueError) as exc:
        raise SystemExit(f"Diagnostic FAILED: {exc}") from None


if __name__ == "__main__":
    main()
