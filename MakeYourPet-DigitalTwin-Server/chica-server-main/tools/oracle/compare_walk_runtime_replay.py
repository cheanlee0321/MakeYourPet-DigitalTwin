#!/usr/bin/env python3
"""Replay original live gait logs through the native port and compare pulses.

The instrumented APK logs CHICA_GAIT immediately after the servo write for a
runtime walk frame.  Pairing each gait record with the preceding CHICA_SERVO
record gives the exact pulse oracle for that frame while avoiding wall-clock
jitter between separate emulator runs.  The printed gait vector is rounded, so
runtime walk vectors are reconstructed from the original command stream.
"""

from __future__ import annotations

import argparse
import ast
import json
import pathlib
import re
import shutil
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD_DIR = ROOT / "build"
PROBE = BUILD_DIR / ("gait_probe.exe" if sys.platform == "win32" else "gait_probe")
NUMBER_RE = re.compile(r"[-+]?\d*\.\d+(?:e[-+]?\d+)?|[-+]?\d+(?:e[-+]?\d+)?", re.I)
PULSES_RE = re.compile(r"pulses=(\[.*\])")


def find_cxx_compiler() -> str:
    candidates = ["c++", "g++", "clang++"]
    if sys.platform == "win32":
        candidates.extend([
            r"C:\Program Files\LLVM\bin\clang++.exe",
            r"C:\msys64\ucrt64\bin\g++.exe",
            r"C:\msys64\mingw64\bin\g++.exe",
        ])
    for cand in candidates:
        if pathlib.Path(cand).is_file():
            return cand
        found = shutil.which(cand)
        if found:
            return found
    return "c++"


def build_probe() -> None:
    BUILD_DIR.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            find_cxx_compiler(),
            "-std=c++17",
            "-Iapp/src/main/cpp",
            "tools/oracle/gait_probe.cpp",
            "app/src/main/cpp/apk_model.cpp",
            "app/src/main/cpp/pulse_conversion.cpp",
            "-o",
            str(PROBE),
        ],
        cwd=ROOT,
        check=True,
    )


def parse_vector(text: str) -> list[float]:
    values = [float(value) for value in NUMBER_RE.findall(text)]
    if len(values) != 3:
        raise ValueError(f"expected 3 values in gait cmd, got {text!r}")
    return values


def parse_walk_target(command: str, crab_mode: bool) -> list[float] | None:
    if not command.startswith("walk") or command.startswith("walkclear"):
        return None
    if ":" not in command:
        return None
    values = [float(value) for value in NUMBER_RE.findall(command.split(":", 1)[1])]
    if len(values) < 3:
        return None
    if crab_mode:
        return [values[1], values[0], 0.0]
    return [values[1], 0.0, values[0]]


def command_target(records: list[dict]) -> list[float] | None:
    crab_mode = False
    target: list[float] | None = None
    for record in records:
        if record.get("type") not in {"command", "device_command"}:
            continue
        command = str(record.get("command", ""))
        if command.startswith("crab"):
            crab_mode = not crab_mode
        next_target = parse_walk_target(command, crab_mode)
        if next_target is not None:
            target = next_target
    return target


def parse_trace(path: pathlib.Path) -> tuple[list[tuple], list[list[int]]]:
    records = [
        json.loads(line)
        for line in path.read_text().splitlines()
        if line.strip()
    ]
    target = command_target(records)
    filtered = [0.0, 0.0, 0.0]
    records = [record for record in records if "time" in record]
    records.sort(key=lambda record: record["time"])

    frames: list[tuple] = []
    observed: list[list[int]] = []
    latest_servo: list[int] | None = None
    stopping = False
    for record in records:
        record_type = record.get("type")
        if record_type == "servo_values":
            latest_servo = record["values"]
            continue
        if record_type != "gait":
            continue
        if latest_servo is None:
            raise ValueError(f"{path}: gait record before any servo output")
        printed = parse_vector(str(record.get("cmd", "")))
        if target is None:
            forward, left, turn = printed
        else:
            if not stopping:
                next_filtered = [
                    current + ((desired - current) * 0.05)
                    for current, desired in zip(filtered, target)
                ]
                if vector_delta(next_filtered, printed) > 0.01:
                    stopping = True
                else:
                    filtered = next_filtered
            if stopping:
                filtered = [current * 0.9 for current in filtered]
            forward, left, turn = filtered
        frames.append((
            int(record["gait"]),
            int(record["style"]),
            float(record["dt"]),
            1 if record.get("allow") else 0,
            forward,
            left,
            turn,
        ))
        observed.append(latest_servo)
        if stopping and not record.get("allow"):
            break

    return frames, observed


def vector_delta(left: list[float], right: list[float]) -> float:
    return max(abs(a - b) for a, b in zip(left, right))


def run_probe(frames: list[tuple]) -> list[list[int]]:
    command = [str(PROBE)]
    for frame in frames:
        command += ["--frame", ",".join(str(value) for value in frame)]
    proc = subprocess.run(command, text=True, capture_output=True, check=True)
    pulses: list[list[int]] = []
    for line in proc.stdout.splitlines():
        match = PULSES_RE.search(line)
        if match:
            pulses.append(ast.literal_eval(match.group(1)))
    return pulses


def compare_file(path: pathlib.Path) -> bool:
    frames, observed = parse_trace(path)
    if not frames:
        print(f"{path.name}: no runtime gait frames")
        return False
    expected = run_probe(frames)
    exact = len(observed) == len(expected) and all(a == b for a, b in zip(observed, expected))
    print(f"{path.name}: frames={len(frames)} exact={exact}")
    for index, (actual, rebuilt) in enumerate(zip(observed, expected)):
        if actual == rebuilt:
            continue
        diffs = [abs(a - b) for a, b in zip(actual, rebuilt)]
        print(f"  frame {index}: max={max(diffs)} sum={sum(diffs)}")
        print(f"    original={actual}")
        print(f"    rebuilt ={rebuilt}")
        break
    if len(observed) != len(expected):
        print(f"  frame count mismatch: original={len(observed)} rebuilt={len(expected)}")
    return exact


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("trace", nargs="+", type=pathlib.Path)
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()

    if not args.skip_build:
        build_probe()

    ok = True
    for trace in args.trace:
        ok = compare_file(trace) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
