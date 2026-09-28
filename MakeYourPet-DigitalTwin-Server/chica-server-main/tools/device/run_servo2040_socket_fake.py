#!/usr/bin/env python3
"""Run a Servo2040 protocol fake against a QEMU unix-socket chardev."""

from __future__ import annotations

import argparse
import select
import socket
import time
from pathlib import Path

from servo2040_protocol_emulator import Servo2040ProtocolEmulator


def connect(path: Path, timeout: float) -> socket.socket:
    deadline = time.monotonic() + timeout
    while True:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.connect(str(path))
            return sock
        except OSError:
            sock.close()
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.1)


def run(path: Path, timeout: float, verbose: bool, ftdi_status: bool) -> None:
    emu = Servo2040ProtocolEmulator()
    emu.set_analog(
        touches=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        voltage=7.4,
        current_amps=2.6,
    )
    print(f"servo2040 fake waiting for {path}", flush=True)
    sock = connect(path, timeout)
    print("servo2040 fake connected", flush=True)
    pending = bytearray()
    try:
        while True:
            readable, _, _ = select.select([sock], [], [], 0.1)
            if not readable:
                continue
            chunk = sock.recv(4096)
            if not chunk:
                print("servo2040 fake disconnected", flush=True)
                return
            pending.extend(chunk)
            request, reply = exchange_complete(emu, pending)
            if verbose:
                print(f"rx={request.hex()} tx={reply.hex()}", flush=True)
            if reply:
                if ftdi_status:
                    reply = b"\x01\x60" + reply
                sock.sendall(reply)
    finally:
        sock.close()


def exchange_complete(emu: Servo2040ProtocolEmulator, pending: bytearray) -> tuple[bytes, bytes]:
    consumed = 0
    request = bytearray()
    reply = bytearray()
    while consumed < len(pending):
        cmd = pending[consumed]
        if (cmd & 0x80) == 0:
            consumed += 1
            continue
        if consumed + 3 > len(pending):
            break
        count = pending[consumed + 2]
        if cmd == 0xD3:
            length = 3 + (count * 2)
        elif cmd == 0xC7:
            length = 3
        else:
            consumed += 1
            continue
        if consumed + length > len(pending):
            break
        frame = bytes(pending[consumed:consumed + length])
        request.extend(frame)
        reply.extend(emu.exchange(frame))
        consumed += length
    if consumed:
        del pending[:consumed]
    return bytes(request), bytes(reply)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--socket", default="/tmp/chica-servo2040.sock")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--verbose", action="store_true")
    parser.add_argument("--ftdi-status", action="store_true",
                        help="prefix host-to-guest replies with FTDI modem status bytes")
    args = parser.parse_args()
    run(Path(args.socket), args.timeout, args.verbose, args.ftdi_status)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
