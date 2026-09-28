#!/usr/bin/env python3
"""Compare Chica animation servo traces without assuming identical sampling.

The original APK uses wall-clock animation loops, so an emulator or virtual
backend can sample the same curve at different times. This tool reports:

* exact same-index equality
* exact ordered subsequence matches
* nearest rebuilt frame delta for each oracle frame
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def servo_frames(path: Path) -> list[list[int]]:
    frames: list[list[int]] = []
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        if record.get("type") == "servo_values":
            frames.append(record["values"])
    return frames


def max_delta(left: list[int], right: list[int]) -> int:
    return max(abs(a - b) for a, b in zip(left, right))


def subsequence_matches(oracle: list[list[int]], rebuilt: list[list[int]]) -> tuple[list[tuple[int, int]], list[int]]:
    matches: list[tuple[int, int]] = []
    missing: list[int] = []
    pos = -1
    for oracle_index, frame in enumerate(oracle):
        found = None
        for rebuilt_index in range(pos + 1, len(rebuilt)):
            if rebuilt[rebuilt_index] == frame:
                found = rebuilt_index
                break
        if found is None:
            missing.append(oracle_index)
        else:
            matches.append((oracle_index, found))
            pos = found
    return matches, missing


def nearest_deltas(oracle: list[list[int]], rebuilt: list[list[int]]) -> list[tuple[int, int]]:
    out: list[tuple[int, int]] = []
    for frame in oracle:
        best_delta = min(max_delta(frame, candidate) for candidate in rebuilt)
        out.append((best_delta, sum(1 for candidate in rebuilt if max_delta(frame, candidate) == best_delta)))
    return out


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("oracle", type=Path)
    parser.add_argument("rebuilt", type=Path)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--diff-limit", type=int, default=12)
    args = parser.parse_args()

    oracle = servo_frames(args.oracle)
    rebuilt = servo_frames(args.rebuilt)
    same_index = sum(1 for left, right in zip(oracle, rebuilt) if left == right)
    matches, missing = subsequence_matches(oracle, rebuilt)
    nearest = nearest_deltas(oracle, rebuilt)
    exact_nearest = sum(1 for delta, _ in nearest if delta == 0)
    worst_nearest = max((delta for delta, _ in nearest), default=0)
    avg_nearest = sum(delta for delta, _ in nearest) / len(nearest) if nearest else 0.0

    print(f"oracle_frames={len(oracle)} rebuilt_frames={len(rebuilt)}")
    print(f"same_index_exact={same_index}")
    print(f"ordered_subsequence_exact={len(matches)}")
    print(f"nearest_exact={exact_nearest}")
    print(f"nearest_worst_delta={worst_nearest}")
    print(f"nearest_avg_delta={avg_nearest:.3f}")
    if missing:
        print("first_missing_nearest:")
        for oracle_index in missing[:args.diff_limit]:
            best_delta = min(max_delta(oracle[oracle_index], candidate) for candidate in rebuilt)
            best_index = next(i for i, candidate in enumerate(rebuilt) if max_delta(oracle[oracle_index], candidate) == best_delta)
            print(f"  oracle_frame={oracle_index} best_rebuilt_frame={best_index} max_delta={best_delta}")

    exact = len(oracle) == len(rebuilt) and oracle == rebuilt
    if exact:
        print("exact=True")
    elif args.strict:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
