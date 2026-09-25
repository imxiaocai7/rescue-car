# Motion command protocol v1

This byte protocol is the contract for PC planner → ESP32-S3 → STM32 motor-control commands. It is transport-independent: the ESP32 may carry the same frame over Wi-Fi/TCP and forward it unchanged over its STM32 link. Exact UART pins/baud, STM32 pin mapping, motor-loop rates, and servo endpoints remain hardware bring-up items and are deliberately not specified here.

Every frame is 20 bytes, little-endian:

| Offset | Size | Field | Meaning |
|---:|---:|---|---|
| 0 | 4 | Magic | ASCII `RCM1` |
| 4 | 1 | Version | `1` |
| 5 | 1 | Type | `1` = motion command |
| 6 | 4 | Sequence | Unsigned command sequence, wraps at 2³² |
| 10 | 2 | Lease | Command validity in milliseconds; 1–1000 |
| 12 | 2 | Left target | Signed mm/s, int16 two's complement |
| 14 | 2 | Right target | Signed mm/s, int16 two's complement |
| 16 | 1 | Actuator | `0` hold, `1` close gate, `2` open gate |
| 17 | 1 | Reserved | Must be zero in v1 |
| 18 | 2 | CRC | CRC-16/CCITT-FALSE over bytes 0–17, little-endian |

The canonical Python encoder/stream decoder and watchdog helpers are in [`pc/motion_protocol.py`](pc/motion_protocol.py). CRC check value for ASCII `123456789` is `0x29B1`. Receivers must reject unsupported headers, invalid actuator values, nonzero reserved bytes, out-of-range leases, CRC failures, and replayed/out-of-order sequences. Sequence comparison uses the half-range rule so `0` follows `0xFFFFFFFF`.

The receiver must stop both wheels when a valid command's lease expires, on communication loss, or on an explicit upstream safety stop. It should not continue the last nonzero target indefinitely. `HOLD` leaves the gate where it is; safe startup gate position and gate open/closed angles must be established during physical commissioning. This lease is a motion watchdog, not an emergency-stop circuit.

Wheel targets are signed velocities, not PWM duty values. STM32 firmware is responsible for closed-loop wheel-speed control using verified encoder direction/counts; these must not be guessed from product photos. The host-side path follower can convert body velocity to wheel linear speeds after measured track width is configured; conversion to wheel RPM additionally requires measured loaded wheel radius. Until those values are established, protocol tests use synthetic values only.
