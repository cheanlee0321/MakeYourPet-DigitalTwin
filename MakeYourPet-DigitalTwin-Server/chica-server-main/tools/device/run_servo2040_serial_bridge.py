#!/usr/bin/env python3
"""Bridge the Android emulator to a REAL Servo2040 over the host's USB serial.

The rebuilt app's SocketServo2040Backend connects to 10.0.2.2:18712 (host
loopback from the emulator) and speaks the raw Servo2040 byte protocol. This
bridge listens there and transparently pumps bytes to/from the physical board
on a host serial device (e.g. /dev/cu.usbmodem1101), so the emulator can drive
real hardware with no phone and no USB passthrough.

Transparent byte pump in both directions; no protocol parsing, so SET frames
(no reply) and GET polls (count*5 reply) both flow through unchanged.
"""
from __future__ import annotations

import argparse
import glob
import select
import socket
import sys

import serial


def resolve_device(device: str | None) -> str:
    """Use the given device, else auto-detect the first /dev/cu.usbmodem* (the
    path changes across re-plugs, e.g. usbmodem1101 -> usbmodem101) or COM port on Windows."""
    if device:
        return device
    matches = sorted(glob.glob("/dev/cu.usbmodem*"))
    if matches:
        return matches[0]
    if sys.platform == "win32":
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            if ports:
                return ports[0].device
        except Exception:
            pass
    raise SystemExit("no serial device found; pass --device")


def pump(listener: socket.socket, conn: socket.socket, port: serial.Serial,
         verbose: bool) -> socket.socket | None:
    # Returns a new client socket if one preempted this one, else None (peer
    # closed). The listener is selected too so a relaunched app takes over
    # without touching (and resetting) the serial port.
    is_windows = sys.platform == "win32"
    while True:
        if is_windows:
            readable, _, _ = select.select([listener, conn], [], [], 0.02)
            has_serial_data = (port.in_waiting or 0) > 0
        else:
            readable, _, _ = select.select([listener, conn, port.fileno()], [], [], 1.0)
            has_serial_data = port.fileno() in readable

        if listener in readable:
            new_conn, addr = listener.accept()
            new_conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"client preempted by {addr[0]}:{addr[1]}", flush=True)
            conn.close()
            return new_conn
        if conn in readable:
            data = conn.recv(4096)
            if data == b"":
                return None
            port.write(data)
            if verbose:
                print(f"app->board {data.hex()}", flush=True)
        if has_serial_data:
            waiting = port.in_waiting or 1
            data = port.read(waiting)
            if data:
                conn.sendall(data)
                if verbose:
                    print(f"board->app {data.hex()}", flush=True)


def run(host: str, port_num: int, device: str | None, baud: int, verbose: bool) -> None:
    device = resolve_device(device)
    port = serial.Serial(device, baudrate=baud, timeout=0)
    print(f"serial open {device} @ {baud}", flush=True)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind((host, port_num))
        listener.listen(1)
        print(f"servo2040 serial bridge listening on {host}:{port_num}", flush=True)
        try:
            conn, addr = listener.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"client connected {addr[0]}:{addr[1]}", flush=True)
            while conn is not None:
                port.reset_input_buffer()
                port.reset_output_buffer()
                try:
                    conn = pump(listener, conn, port, verbose)
                    if conn is None:
                        print("client disconnected; waiting for next", flush=True)
                        conn = listener.accept()[0]
                        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        print("client connected", flush=True)
                except (ConnectionResetError, BrokenPipeError, OSError) as error:
                    print(f"client error: {error}; waiting for next", flush=True)
                    conn = listener.accept()[0]
        finally:
            port.close()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18712)
    parser.add_argument("--device", default=None,
                        help="serial device; default auto-detects /dev/cu.usbmodem*")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    run(args.host, args.port, args.device, args.baud, args.verbose)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
