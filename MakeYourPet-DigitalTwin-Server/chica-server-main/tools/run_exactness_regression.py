#!/usr/bin/env python3
"""Run the currently proven Chica exactness checks."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(label: str, command: list[str]) -> bool:
    print(f"\n== {label} ==")
    result = subprocess.run(command, cwd=ROOT, text=True)
    if result.returncode != 0:
        print(f"{label}: failed with exit code {result.returncode}")
        return False
    return True


def main() -> int:
    (ROOT / "build").mkdir(exist_ok=True)
    cxx = os.environ.get("CXX")
    if not cxx:
        for candidate in ("c++", "clang++", "g++"):
            if shutil.which(candidate):
                cxx = candidate
                break
    if not cxx:
        cxx = "c++"
    binary_out = "build/animation_replay.exe" if sys.platform == "win32" else "build/animation_replay"
    if not run(
        "build-animation-replay",
        [
            cxx,
            "-std=c++17",
            "-O2",
            "-Iapp/src/main/cpp",
            "tools/oracle/animation_replay.cpp",
            "app/src/main/cpp/apk_model.cpp",
            "app/src/main/cpp/pulse_conversion.cpp",
            "-o",
            binary_out,
        ],
    ):
        return 1

    checks = [
        (
            "runtime-timing-contract",
            [sys.executable, "tools/oracle/verify_runtime_timing_contract.py"],
        ),
        (
            "gait-oracles",
            [
                sys.executable,
                "tools/oracle/compare_gait_oracle.py",
                "../oracle/api35_walk3_025_050_3_gaittrace.jsonl",
                "../oracle/api35_walk2_025_050_2_gaittrace.jsonl",
                "../oracle/api35_walk1_025_050_1_gaittrace.jsonl",
                "../oracle/api35_walk15_025_050_4_gaittrace.jsonl",
                "../oracle/api35_walk25_025_050_5_gaittrace.jsonl",
                "../oracle/api35_walkwave_025_050_6_gaittrace.jsonl",
                "../oracle/api35_crab_walk3_025_050_3_gaittrace.jsonl",
            ],
        ),
        (
            "quad-runtime",
            [sys.executable, "tools/oracle/compare_quad_runtime.py"],
        ),
        (
            "walk-runtime-replay",
            [
                sys.executable,
                "tools/oracle/compare_walk_runtime_replay.py",
                "../oracle/controltrace_walk_runtime_virtualtouch_original.jsonl",
                "../oracle/controltrace_walkclear_dense_virtualtouch_original.jsonl",
            ],
        ),
        (
            "animation-torque-sit",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_torque_sit_virtualhw_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-quad-14",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_quad_14_keep_autositoff_logcat_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-quad-03",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_quad_03_verify_logcat_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-quad-25",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_quad_25_verify_logcat_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-bounce-jump",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_bounce_jump_autositoff_virtualhw_anim.jsonl",
                "--scenario",
                "impulse",
                "--strict",
            ],
        ),
        (
            "calibration-virtual-touch",
            [sys.executable, "tools/oracle/compare_calibration_virtualtouch.py"],
        ),
        (
            "ack-stand-ramp",
            [sys.executable, "tools/oracle/compare_ack_stand_ramp.py"],
        ),
        (
            "no-hardware-servo",
            [sys.executable, "tools/device/verify_no_hardware_servo.py"],
        ),
        (
            "servo2040-protocol",
            [sys.executable, "tools/device/verify_servo2040_protocol.py"],
        ),
        (
            "pololu-protocol",
            [sys.executable, "tools/device/verify_pololu_protocol.py"],
        ),
    ]

    ok = True
    for label, command in checks:
        if label in ("quad-runtime", "calibration-virtual-touch", "ack-stand-ramp"):
            default_oracle_dir = ROOT.parent / "oracle"
            if not default_oracle_dir.exists():
                print(f"\n== {label} ==")
                print(f"{label}: skipped (oracle directory not found at {default_oracle_dir})")
                continue
        elif any(arg.endswith(".jsonl") for arg in command):
            missing_jsonl = [
                arg for arg in command
                if isinstance(arg, str) and arg.endswith(".jsonl")
                and not (ROOT / arg).exists() and not Path(arg).exists()
            ]
            if missing_jsonl:
                print(f"\n== {label} ==")
                print(f"{label}: skipped (oracle trace data not present on host: {missing_jsonl[0]})")
                continue

        ok = run(label, command) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
