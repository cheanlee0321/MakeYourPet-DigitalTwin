#!/usr/bin/env python3
"""Compare original/rebuilt Chica TCP control replay traces."""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any


FLAGS_RE = re.compile(r"FLAGS=([0-9]+)")
FLOAT_RE = re.compile(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?")


def load(path: Path) -> list[dict[str, Any]]:
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def flag_transitions(records: list[dict[str, Any]]) -> list[str]:
    transitions: list[str] = []
    last: str | None = None
    for record in records:
        if record.get("type") != "tcp":
            continue
        match = FLAGS_RE.search(record.get("line", ""))
        if not match:
            continue
        value = match.group(1)
        if value != last:
            transitions.append(value)
            last = value
    return transitions


def commands(records: list[dict[str, Any]]) -> list[str]:
    return [record["command"] for record in records if record.get("type") == "command"]


def gait_signature(records: list[dict[str, Any]], include_dt: bool) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for record in records:
        if record.get("type") != "gait":
            continue
        signature = {
            "allow": record.get("allow"),
            "gait": record.get("gait"),
            "style": record.get("style"),
            "cmd": normalized_command(record.get("cmd")),
        }
        if include_dt:
            signature["dt"] = round(float(record.get("dt", 0.0)), 6)
        out.append(signature)
    return out


def normalized_command(value: Any) -> list[float] | Any:
    if isinstance(value, list):
        return [round(float(item), 5) for item in value]
    if isinstance(value, str):
        return [round(float(item), 5) for item in FLOAT_RE.findall(value)]
    return value


def servo_frames(records: list[dict[str, Any]]) -> list[list[int]]:
    return [record["values"] for record in records if record.get("type") == "servo_values"]


def first_servo_diffs(oracle: list[list[int]], rebuilt: list[list[int]], limit: int) -> list[str]:
    lines: list[str] = []
    for index, (left, right) in enumerate(zip(oracle, rebuilt)):
        if left == right:
            continue
        max_delta = max(abs(a - b) for a, b in zip(left, right))
        lines.append(f"frame {index}: max_delta={max_delta} oracle={left} rebuilt={right}")
        if len(lines) >= limit:
            break
    return lines


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("oracle", type=Path)
    parser.add_argument("rebuilt", type=Path)
    parser.add_argument("--strict", action="store_true",
            help="Exit non-zero when any compared signature differs.")
    parser.add_argument("--ignore-dt", action="store_true",
            help="Ignore runtime gait dt values. Useful when replay scheduling is not synchronized.")
    parser.add_argument("--diff-limit", type=int, default=5)
    args = parser.parse_args()

    oracle = load(args.oracle)
    rebuilt = load(args.rebuilt)

    oracle_flags = flag_transitions(oracle)
    rebuilt_flags = flag_transitions(rebuilt)
    oracle_commands = commands(oracle)
    rebuilt_commands = commands(rebuilt)
    oracle_gait = gait_signature(oracle, include_dt=not args.ignore_dt)
    rebuilt_gait = gait_signature(rebuilt, include_dt=not args.ignore_dt)
    oracle_servo = servo_frames(oracle)
    rebuilt_servo = servo_frames(rebuilt)

    mismatches: list[str] = []
    if oracle_commands != rebuilt_commands:
        mismatches.append("commands")
    if oracle_flags != rebuilt_flags:
        mismatches.append("flag transitions")
    if oracle_gait != rebuilt_gait:
        mismatches.append("gait signature")
    if oracle_servo != rebuilt_servo:
        mismatches.append("servo frames")

    print(f"oracle_commands={oracle_commands}")
    print(f"rebuilt_commands={rebuilt_commands}")
    print(f"oracle_flag_transitions={oracle_flags}")
    print(f"rebuilt_flag_transitions={rebuilt_flags}")
    print(f"oracle_gait_frames={len(oracle_gait)} rebuilt_gait_frames={len(rebuilt_gait)}")
    print(f"oracle_servo_frames={len(oracle_servo)} rebuilt_servo_frames={len(rebuilt_servo)}")

    if oracle_gait != rebuilt_gait:
        print("gait_signature_diff:")
        for index, (left, right) in enumerate(zip(oracle_gait, rebuilt_gait)):
            if left != right:
                print(f"  frame {index}: oracle={left} rebuilt={right}")
                break
        if len(oracle_gait) != len(rebuilt_gait):
            print(f"  length differs: oracle={len(oracle_gait)} rebuilt={len(rebuilt_gait)}")

    servo_diffs = first_servo_diffs(oracle_servo, rebuilt_servo, args.diff_limit)
    if servo_diffs:
        print("servo_diffs:")
        for line in servo_diffs:
            print(f"  {line}")
    if len(oracle_servo) != len(rebuilt_servo):
        print(f"servo_length_diff=oracle:{len(oracle_servo)} rebuilt:{len(rebuilt_servo)}")

    if mismatches:
        print("mismatches=" + ",".join(mismatches))
        return 1 if args.strict else 0
    print("exact=True")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
