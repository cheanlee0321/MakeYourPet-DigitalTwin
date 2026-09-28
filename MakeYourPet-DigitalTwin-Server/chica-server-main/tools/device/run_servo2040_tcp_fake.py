#!/usr/bin/env python3
"""Run a host-side Servo2040 protocol fake for Android emulator tests."""

from __future__ import annotations

import argparse
import json
import select
import socket
import time
from pathlib import Path

from servo2040_protocol_emulator import Servo2040ProtocolEmulator
from run_servo2040_socket_fake import exchange_complete


def record_servo_frames(request: bytes, servo_times: list[float] | None) -> None:
    if servo_times is None:
        return
    offset = 0
    while offset + 3 <= len(request):
        command = request[offset]
        count = request[offset + 2]
        length = 3 + (count * 2) if command == 0xD3 else 3
        if offset + length > len(request):
            return
        if command == 0xD3 and count == 18:
            servo_times.append(time.time())
        offset += length


def run(
    host: str,
    port: int,
    verbose: bool,
    reply_delay_ms: float,
    timing_out: Path | None,
) -> None:
    servo_times: list[float] | None = [] if timing_out is not None else None
    emu = Servo2040ProtocolEmulator()
    emu.set_analog(
        touches=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        voltage=7.4,
        current_amps=2.6,
    )
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((host, port))
            listener.listen(1)
            print(f"servo2040 tcp fake listening on {host}:{port}", flush=True)
            while True:
                conn, addr = listener.accept()
                with conn:
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    print(f"servo2040 tcp fake connected from {addr[0]}:{addr[1]}", flush=True)
                    pending = bytearray()
                    while True:
                        readable, _, _ = select.select([conn], [], [], 0.1)
                        if not readable:
                            continue
                        chunk = conn.recv(4096)
                        if not chunk:
                            print("servo2040 tcp fake disconnected", flush=True)
                            break
                        pending.extend(chunk)
                        request, reply = exchange_complete(emu, pending)
                        record_servo_frames(request, servo_times)
                        if verbose and request:
                            print(f"rx={request.hex()} tx={reply.hex()}", flush=True)
                        if reply:
                            if reply_delay_ms > 0.0:
                                time.sleep(reply_delay_ms / 1000.0)
                            conn.sendall(reply)
    except KeyboardInterrupt:
        print("servo2040 tcp fake stopped", flush=True)
    finally:
        if timing_out is not None and servo_times is not None:
            with timing_out.open("w", encoding="utf-8") as stream:
                for timestamp in servo_times:
                    stream.write(json.dumps({"type": "servo_packet", "time": timestamp}) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18712)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--reply-delay-ms", type=float, default=0.0)
    parser.add_argument("--timing-out", type=Path)
    args = parser.parse_args()
    if args.timing_out is not None:
        args.timing_out.unlink(missing_ok=True)
    run(args.host, args.port, args.verbose, args.reply_delay_ms, args.timing_out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
