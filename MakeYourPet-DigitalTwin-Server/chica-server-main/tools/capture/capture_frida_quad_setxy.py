#!/usr/bin/env python3
"""Capture quad setxy runtime calls by hooking the original APK with Frida."""

from __future__ import annotations

import argparse
import json
import queue
import socket
import subprocess
import threading
import time
from pathlib import Path


PACKAGE = "com.makeyourpet.chicaserver"
PORT = 18711


HOOK_SCRIPT = Path(__file__).with_name("frida_chica_hooks.js")


def adb(*args: str, check: bool = True) -> str:
    proc = subprocess.run(["adb", *args], text=True, capture_output=True, check=check)
    return proc.stdout.strip()


def install_apk(apk: Path | None) -> None:
    if apk is None:
        return
    proc = subprocess.run(["adb", "install", "-r", str(apk)], text=True, capture_output=True)
    if proc.returncode == 0:
        return
    adb("uninstall", PACKAGE, check=False)
    subprocess.run(["adb", "install", str(apk)], check=True)


def start_app() -> None:
    adb("shell", "am", "force-stop", PACKAGE, check=False)
    time.sleep(0.3)
    adb("shell", "monkey", "-p", PACKAGE, "-c", "android.intent.category.LAUNCHER", "1")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if adb("shell", "pidof", PACKAGE, check=False):
            return
        time.sleep(0.25)
    raise RuntimeError(f"{PACKAGE} did not start")


def connect_tcp() -> tuple[socket.socket, list[dict]]:
    adb("forward", f"tcp:{PORT}", f"tcp:{PORT}")
    deadline = time.time() + 12.0
    last_error: OSError | None = None
    while time.time() < deadline:
        try:
            sock = socket.create_connection(("127.0.0.1", PORT), timeout=2.0)
            sock.settimeout(0.05)
            try:
                greeting = sock.recv(8192)
            except socket.timeout:
                greeting = b""
            records = [
                {"type": "tcp", "line": line, "hostTime": time.time()}
                for line in greeting.decode(errors="replace").splitlines()
            ]
            if any(record["line"].startswith("ready:") for record in records):
                return sock, records
            sock.close()
        except OSError as error:
            last_error = error
        time.sleep(0.25)
    raise RuntimeError(f"TCP server did not become ready: {last_error}")


def drain_tcp(sock: socket.socket, records: list[dict]) -> None:
    while True:
        try:
            data = sock.recv(8192)
        except socket.timeout:
            return
        except OSError:
            return
        if not data:
            return
        for line in data.decode(errors="replace").splitlines():
            records.append({"type": "tcp", "line": line, "hostTime": time.time()})


def drive_tcp(quad: str, walk: str, ack_interval: float) -> list[dict]:
    steps = [
        ("autosit", 0.7),
        ("keep", 1.4),
        ("torque", 3.3),
        ("sit", 1.7),
        (f"quad:{quad}", 8.5),
        (walk, 3.2),
        ("walkclear", 1.5),
    ]
    sock, records = connect_tcp()
    drain_tcp(sock, records)
    last_ack = 0.0
    for command, delay in steps:
        drain_tcp(sock, records)
        records.append({"type": "command", "command": command, "hostTime": time.time()})
        sock.sendall((command + "\n").encode())
        deadline = time.time() + delay
        while time.time() < deadline:
            now = time.time()
            if now - last_ack >= ack_interval:
                sock.sendall(b"ack\n")
                last_ack = now
            drain_tcp(sock, records)
            time.sleep(0.01)
    drain_tcp(sock, records)
    sock.close()
    return records


def start_frida_cli(records: queue.Queue[dict]) -> subprocess.Popen:
    proc = subprocess.Popen(
        [
            "frida",
            "-U",
            "-N",
            PACKAGE,
            "-l",
            str(HOOK_SCRIPT),
            "-q",
            "-t",
            "inf",
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
    )

    def reader() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            line = line.rstrip()
            if line.startswith("CHICA_FRIDA "):
                try:
                    record = json.loads(line[len("CHICA_FRIDA "):])
                    record.setdefault("hostTime", time.time())
                    records.put(record)
                except json.JSONDecodeError:
                    records.put({"type": "frida_parse_error", "hostTime": time.time(), "line": line})
            elif line:
                records.put({"type": "frida_log", "hostTime": time.time(), "line": line})

    threading.Thread(target=reader, daemon=True).start()
    return proc


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--apk", type=Path)
    parser.add_argument("--quad", required=True)
    parser.add_argument("--walk", default="setxy:0,0.9")
    parser.add_argument("--ack-interval", type=float, default=0.10)
    args = parser.parse_args()

    install_apk(args.apk)
    start_app()
    time.sleep(3.0)

    messages: queue.Queue[dict] = queue.Queue()
    proc = start_frida_cli(messages)
    time.sleep(1.0)

    try:
        tcp_records = drive_tcp(args.quad, args.walk, args.ack_interval)
        time.sleep(1.0)
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()

    frida_records: list[dict] = []
    while True:
        try:
            frida_records.append(messages.get_nowait())
        except queue.Empty:
            break

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("w", encoding="utf-8") as out:
        for record in tcp_records + frida_records:
            out.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
