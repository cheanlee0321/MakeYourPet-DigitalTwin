#!/usr/bin/env python3
"""Capture the previously-untested command scenarios against one APK build.

Runs each scenario through capture_control_logcat.py and writes one jsonl per
scenario into <outdir>/<label>_<scenario>.jsonl. Run once with the instrumented
original APK and once with the rebuilt debug APK, then diff with coverage_diff.py.
"""
import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent

# Each scenario: name -> list of "command:delay" steps. virtualtouch first to
# enable the touch+battery fixture in the rebuilt (the original build bakes it
# in but also accepts the command as a no-op).
SCENARIOS = {
    "block":     ["virtualtouch:0.5", "torque:0.5", "block:1.0", "block:1.5"],
    "calibpos":  ["virtualtouch:0.5", "torque:0.5", "calibpos:1.5", "calibpos:1.5"],
    "clear":     ["virtualtouch:0.5", "autosit:0.41", "walk3:0.25,0.50,3:1.69", "clear:3.0"],
    "level":     ["virtualtouch:0.5", "torque:0.5", "sit:1.5", "level:3.0", "level:1.5"],
    "beep":      ["virtualtouch:0.5", "beep:1.0"],
    "setdive":   ["virtualtouch:0.5", "torque:0.5", "sit:1.5", "setdive:0.0,0.9:2.5", "setclear:1.5"],
    "setrotate": ["virtualtouch:0.5", "torque:0.5", "sit:1.5", "setrotate:0.0,0.9:2.5", "setclear:1.5"],
    "setvw":     ["virtualtouch:0.5", "torque:0.5", "sit:1.5", "setvw:0.0,0.9:2.5", "setclear:1.5"],
    "setzu":     ["virtualtouch:0.5", "torque:0.5", "sit:1.5", "setzu:0.0,0.9:2.5", "setclear:1.5"],
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apk", type=Path, required=True)
    ap.add_argument("--label", required=True, help="original | rebuilt")
    ap.add_argument("--outdir", type=Path, required=True)
    ap.add_argument("--only", help="comma-separated subset of scenario names")
    args = ap.parse_args()
    args.outdir.mkdir(parents=True, exist_ok=True)

    names = list(SCENARIOS)
    if args.only:
        names = [n.strip() for n in args.only.split(",")]

    package = "com.makeyourpet.chicaserver"

    first = True
    for name in names:
        steps = SCENARIOS[name]
        out = args.outdir / f"{args.label}_{name}.jsonl"
        cmd = [sys.executable, str(ROOT / "capture_control_logcat.py"),
               "--out", str(out), "--package", package]
        if first:
            cmd += ["--apk", str(args.apk)]  # install once; reuse for the rest
            first = False
        for s in steps:
            cmd += ["--step", s]
        print(f"[{args.label}] {name} ...", flush=True)
        r = subprocess.run(cmd)
        if r.returncode != 0:
            print(f"  FAILED {name}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
