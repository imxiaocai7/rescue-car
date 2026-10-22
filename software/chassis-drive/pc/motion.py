"""Explicit-session motion client. This does not certify physical motor safety."""
from dataclasses import dataclass
from enum import IntEnum
import secrets
import socket
import struct
import time
from protocol import checked_body, with_crc, u32

FIRMWARE_ID = 0x20261011
MAX_DUTY = 250
ARMED = 1
REQUEST_SIZE = 32
REPLY_SIZE = 40


class Op(IntEnum):
    HELLO = 1
    ARM = 2
    DRIVE = 3
    STOP = 4
    STATUS = 5


def duty(value):
    if isinstance(value, bool) or not isinstance(value, int) or not -MAX_DUTY <= value <= MAX_DUTY:
        raise ValueError("duty must be an integer in -250..250 permille")
    return value


@dataclass(frozen=True)
class Request:
    sequence: int
    session: int
    token: int
    opcode: Op
    left: int = 0
    right: int = 0

    def encode(self):
        opcode = Op(self.opcode)
        if opcode != Op.DRIVE and (self.left or self.right):
            raise ValueError("non-DRIVE commands require zero duty")
        return with_crc(struct.pack("<4sIIB3xIhh4x", b"DCM2", u32(self.sequence),
                                   u32(self.session), opcode, u32(self.token),
                                   duty(self.left), duty(self.right)))

    @classmethod
    def decode(cls, packet):
        body = checked_body(packet, b"DCM2", REQUEST_SIZE)
        if any(body[13:16]) or any(body[24:28]):
            raise ValueError("nonzero reserved bytes")
        sequence, session, opcode, token, left, right = struct.unpack_from("<IIB3xIhh", body, 4)
        result = cls(sequence, session, token, Op(opcode), left, right)
        result.encode()
        return result


@dataclass(frozen=True)
class Reply:
    sequence: int
    session: int
    uptime_ms: int
    firmware_id: int
    flags: int
    token: int
    left: int
    right: int
    result: int

    def encode(self):
        return with_crc(struct.pack("<4sIIIIIIhhB3x", b"DCS2", u32(self.sequence),
                                   u32(self.session), u32(self.uptime_ms), u32(self.firmware_id),
                                   u32(self.flags), u32(self.token), duty(self.left), duty(self.right), self.result))

    @classmethod
    def decode(cls, packet):
        body = checked_body(packet, b"DCS2", REPLY_SIZE)
        if any(body[33:36]) or body[32] > 4:
            raise ValueError("invalid result/reserved bytes")
        values = struct.unpack_from("<IIIIIIhhB", body, 4)
        result = cls(*values)
        duty(result.left)
        duty(result.right)
        if result.flags & ~7:
            raise ValueError("unknown motion flags")
        return result


class Client:
    def __init__(self, host, port=6002, timeout=0.2):
        self.socket = socket.create_connection((host, port), timeout=1)
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.timeout = timeout
        self.session = secrets.randbelow(0xFFFFFFFF) + 1
        self.sequence = secrets.randbits(32)
        self.token = 0
        self.last_ticks = None
        self.armed = False

    def request(self, opcode, left=0, right=0):
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        request = Request(self.sequence, self.session, self.token, opcode, left, right)
        packet = request.encode()
        deadline = time.monotonic() + self.timeout
        self.socket.settimeout(self.timeout)
        self.socket.sendall(packet)
        data = bytearray()
        while len(data) < REPLY_SIZE:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("STM32 motion reply timed out")
            self.socket.settimeout(remaining)
            chunk = self.socket.recv(REPLY_SIZE - len(data))
            if not chunk:
                raise ConnectionError("motion bridge disconnected")
            data.extend(chunk)
        if time.monotonic() >= deadline:
            raise TimeoutError("late STM32 motion reply")
        reply = Reply.decode(data)
        if (reply.sequence, reply.session, reply.firmware_id) != (self.sequence, self.session, FIRMWARE_ID):
            raise RuntimeError("wrong sequence/session/firmware; reconnect explicitly")
        if self.last_ticks is not None and ((reply.uptime_ms - self.last_ticks) & 0xFFFFFFFF) >= 0x80000000:
            raise RuntimeError("STM32 restarted; reconnect explicitly")
        self.last_ticks = reply.uptime_ms
        self.armed = bool(reply.flags & ARMED)
        if reply.result:
            raise RuntimeError(f"STM32 rejected command: result={reply.result}, flags={reply.flags}")
        if opcode == Op.HELLO:
            if not reply.token or reply.flags & ARMED or reply.left or reply.right:
                raise RuntimeError("HELLO did not return a disabled fresh session")
            self.token = reply.token
        elif opcode == Op.STOP:
            self.token = 0
            if reply.flags & ARMED or reply.left or reply.right:
                raise RuntimeError("STOP did not disable outputs")
        elif reply.token != self.token:
            raise RuntimeError("motion session expired; explicit rearm required")
        if opcode in (Op.ARM, Op.DRIVE) and not self.armed:
            raise RuntimeError("STM32 did not confirm enable")
        return reply

    def hello(self):
        self.token = 0
        return self.request(Op.HELLO)

    def arm(self):
        self.hello()
        return self.request(Op.ARM)

    def stop(self):
        return self.request(Op.STOP)

    def close(self):
        try:
            self.stop()
        except (OSError, ValueError, RuntimeError):
            pass  # No synthesized ACK; STM32 lease remains the fallback.
        finally:
            self.socket.close()
            self.armed = False


def pulse(client, left, right, duration):
    duty(left)
    duty(right)
    if not 0 < duration <= 1:
        raise ValueError("first-stage pulses must be >0 and <=1 second")
    try:
        client.arm()
        started = time.monotonic()
        while time.monotonic() - started < duration:
            cycle = time.monotonic()
            client.request(Op.DRIVE, left, right)
            time.sleep(max(0, 0.05 - (time.monotonic() - cycle)))
    finally:
        client.stop()
