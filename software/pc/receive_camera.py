"""Read and measure the OpenMV/ESP32 TCP image stream (no motor commands)."""

import argparse
import socket
import time

from rescue_transport import ImageFrameDecoder


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="ESP32 IP address")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    decoder = ImageFrameDecoder()
    count = 0
    started = time.monotonic()
    last_seq = None
    with socket.create_connection((args.host, args.port), timeout=5) as connection:
        connection.settimeout(10)
        while True:
            chunk = connection.recv(8192)
            if not chunk:
                raise ConnectionError("camera connection closed")
            for frame in decoder.feed(chunk):
                count += 1
                gap = 0 if last_seq is None else (frame.seq - last_seq - 1) & 0xFFFFFFFF
                last_seq = frame.seq
                elapsed = max(time.monotonic() - started, 0.001)
                print(
                    f"seq={frame.seq} {frame.width}x{frame.height} "
                    f"jpeg={len(frame.jpeg)}B avg_fps={count / elapsed:.2f} "
                    f"seq_gap={gap} bad_header={decoder.bad_headers} "
                    f"bad_payload={decoder.bad_payloads}",
                    flush=True,
                )


if __name__ == "__main__":
    main()
