#!/usr/bin/env python3
"""Capture original APK quad-mode walking traces from logcat while driving TCP."""

from __future__ import annotations

import argparse
import ast
import json
import queue
import re
import socket
import subprocess
import threading
import time
from pathlib import Path


PACKAGE = "com.makeyourpet.chicaserver"
PORT = 18711
LOG_RE = re.compile(r"^\s*(\d+\.\d+)\s+\d+\s+\d+\s+\w\s+([^:]+):\s?(.*)$")


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
    adb("logcat", "-c", check=False)
    adb("shell", "monkey", "-p", PACKAGE, "-c", "android.intent.category.LAUNCHER", "1")
    deadline = time.time() + 10.0
    while time.time() < deadline:
        if adb("shell", "pidof", PACKAGE, check=False):
            return
        time.sleep(0.25)
    raise RuntimeError(f"{PACKAGE} did not start")


def parse_log_line(raw: str) -> dict | None:
    match = LOG_RE.match(raw.rstrip())
    if not match:
        return None
    timestamp = float(match.group(1))
    tag = match.group(2).strip()
    message = match.group(3).strip()
    if tag == "CHICA_SERVO":
        try:
            return {"type": "servo_values", "time": timestamp, "values": ast.literal_eval(message)}
        except (SyntaxError, ValueError):
            return {"type": "log", "time": timestamp, "tag": tag, "message": message}
    if message.startswith("{"):
        try:
            record = json.loads(message)
            record.setdefault("time", timestamp)
            if tag == "CHICA_GAIT":
                record.setdefault("type", "gait")
            elif tag == "CHICA_CONTROL":
                record.setdefault("type", "control")
            elif tag == "CHICA_COMMAND":
                record.setdefault("type", "device_command")
            return record
        except json.JSONDecodeError:
            pass
    if tag == "CHICA_MARK":
        return {"type": "mark", "time": timestamp, "message": message}
    return {"type": "log", "time": timestamp, "tag": tag, "message": message}


def start_logcat(records: queue.Queue[dict], stop: threading.Event) -> subprocess.Popen:
    proc = subprocess.Popen(
        [
            "adb",
            "logcat",
            "-v",
            "epoch",
            "CHICA_GAIT:I",
            "CHICA_SERVO:I",
            "CHICA_CONTROL:I",
            "CHICA_COMMAND:I",
            "CHICA_MARK:I",
            "*:S",
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        bufsize=1,
    )

    def reader() -> None:
        assert proc.stdout is not None
        for line in proc.stdout:
            if stop.is_set():
                break
            record = parse_log_line(line)
            if record is not None:
                records.put(record)

    threading.Thread(target=reader, daemon=True).start()
    return proc


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


def drive_tcp(steps: list[tuple[str, float]], ack_interval: float) -> list[dict]:
    sock, records = connect_tcp()
    drain_tcp(sock, records)
    last_ack = 0.0
    for command, delay in steps:
        drain_tcp(sock, records)
        records.append({"type": "command", "command": command, "hostTime": time.time()})
        adb("shell", "log", "-t", "CHICA_MARK", f"send:{command}", check=False)
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


def flush_queue(records: queue.Queue[dict]) -> list[dict]:
    out: list[dict] = []
    while True:
        try:
            out.append(records.get_nowait())
        except queue.Empty:
            return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--apk", type=Path)
    parser.add_argument("--quad", required=True, help="Disabled leg pair, e.g. 1,4")
    parser.add_argument("--walk", default="walk3:0.25,0.50,3")
    parser.add_argument("--ack-interval", type=float, default=0.10)
    args = parser.parse_args()

    install_apk(args.apk)
    start_app()
    log_records: queue.Queue[dict] = queue.Queue()
    stop = threading.Event()
    proc = start_logcat(log_records, stop)
    time.sleep(3.0)
    steps = [
        ("autosit", 0.7),
        ("keep", 1.4),
        ("torque", 3.3),
        ("sit", 1.7),
        (f"quad:{args.quad}", 8.5),
        (args.walk, 3.2),
        ("walkclear", 1.5),
    ]
    tcp_records = drive_tcp(steps, args.ack_interval)
    time.sleep(1.0)
    stop.set()
    proc.terminate()
    try:
        proc.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()

    args.out.parent.mkdir(parents=True, exist_ok=True)
    records = tcp_records + flush_queue(log_records)
    with args.out.open("w", encoding="utf-8") as out:
        for record in records:
            out.write(json.dumps(record, separators=(",", ":"), sort_keys=True) + "\n")
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
