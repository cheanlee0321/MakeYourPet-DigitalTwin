#!/usr/bin/env python3
"""Replay original animation elapsed samples through the rebuilt model.

This compares only frames produced by an original CHICA_ANIM sample. The
original APK logs an anim record immediately before the matching servo write,
so pairing by the next servo_values record removes wall-clock cadence from the
comparison.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass
class Segment:
    method: str
    duration: float
    samples: list[tuple[float, list[int]]]


def pair_segments(path: Path) -> list[Segment]:
    rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    pending: dict | None = None
    segments: list[Segment] = []
    current: Segment | None = None
    previous_elapsed = -1.0

    for record in rows:
        kind = record.get("type")
        if kind == "anim":
            pending = record
        elif kind == "servo_values" and pending is not None:
            elapsed = float(pending["elapsed"])
            method = str(pending["method"])
            duration = float(pending["duration"])
            if (
                current is None
                or current.method != method
                or current.duration != duration
                or elapsed < previous_elapsed
            ):
                current = Segment(method=method, duration=duration, samples=[])
                segments.append(current)
            current.samples.append((elapsed, record["values"]))
            previous_elapsed = elapsed
            pending = None
    return segments


def replay_script(
    segments: list[Segment],
    m_threshold: float,
    m_lift: float,
    m_layer_blend: float,
    m_legs: str,
    scenario: str,
    quad_pair: tuple[int, int] | None,
) -> str:
    if scenario == "auto":
        scenario = infer_scenario(segments)
    if scenario == "standing-home":
        return standing_home_script(segments)
    if scenario == "quad":
        return quad_script(segments, quad_pair or (1, 4))
    if scenario == "impulse":
        return impulse_script(segments)
    if len(segments) < 4:
        raise ValueError(f"expected at least 4 animation segments, got {len(segments)}")
    lines = ["constructor"]
    # Startup in the original APK: constructor pose, wide/high shape, normal shape.
    lines.append(sample_line("shape 265 60 30 1.07", segments[0]))
    lines.append(sample_line("shape 220 -40 55 1.15", segments[1]))
    # Sit toggle while initially sitting: stand body lift, then sit body drop.
    lines.append(sample_line("bodyz 40", segments[2]))
    lines.append(sample_line("bodyz 0", segments[3]))
    if len(segments) >= 5 and segments[4].method == "M":
        lines.append(pose_sample_line(m_threshold, m_lift, m_layer_blend, m_legs, segments[4]))
    return "\n".join(lines) + "\n"


def infer_scenario(segments: list[Segment]) -> str:
    pattern = [(segment.method, int(segment.duration)) for segment in segments[:4]]
    if pattern == [("l", 400), ("M", 650), ("M", 650), ("l", 550)]:
        return "standing-home"
    if (
        len(segments) >= 5
        and [(segment.method, round(segment.duration, 1)) for segment in segments[:5]]
        == [("l", 400.0), ("l", 550.0), ("l", 700.0), ("M", 812.5), ("l", 500.0)]
    ):
        return "quad"
    if (
        len(segments) >= 7
        and [(segment.method, round(segment.duration, 1)) for segment in segments[:7]]
        == [("l", 800.0), ("l", 1200.0), ("l", 400.0), ("l", 550.0), ("l", 700.0), ("M", 812.5), ("l", 500.0)]
    ):
        return "quad"
    if (
        len(segments) >= 13
        and [(segment.method, round(segment.duration, 1)) for segment in segments[5:13]]
        == [("l", 168.0), ("l", 84.0), ("l", 168.0), ("l", 84.0), ("l", 200.0), ("l", 66.7), ("l", 66.7), ("l", 200.0)]
    ):
        return "impulse"
    return "startup-sit-home"


def standing_home_script(segments: list[Segment]) -> str:
    if len(segments) < 4:
        raise ValueError(f"expected at least 4 animation segments, got {len(segments)}")
    lines = ["constructor"]
    lines.append("shape 265 60 30 1.07 800")
    lines.append("shape 220 -40 55 1.15 1200")
    lines.append(sample_line("bodyz 40", segments[0]))
    lines.append(pose_sample_line(-1.0, 40.0, 1.0, "0,4,2", segments[1]))
    lines.append(pose_sample_line(-1.0, 40.0, 1.0, "3,1,5", segments[2]))
    lines.append(sample_line("bodyz 0", segments[3]))
    return "\n".join(lines) + "\n"


def quad_script(segments: list[Segment], disabled_pair: tuple[int, int]) -> str:
    offset = 0
    lines = ["constructor"]
    if len(segments) >= 2 and segments[0].method == "l" and segments[0].duration == 800.0 and segments[1].duration == 1200.0:
        lines.append(sample_line("shape 265 60 30 1.07", segments[0]))
        lines.append(sample_line("shape 220 -40 55 1.15", segments[1]))
        offset = 2
    else:
        lines.append("shape 265 60 30 1.07 800")
        lines.append("shape 220 -40 55 1.15 1200")
    if len(segments) < offset + 5:
        raise ValueError(f"expected at least {offset + 5} quad animation segments, got {len(segments)}")
    disabled_text = f"{disabled_pair[0]},{disabled_pair[1]}"
    active_text = ",".join(str(leg) for leg in active_leg_complement(disabled_pair))
    lines.append(sample_line("bodyz 40", segments[offset]))
    lines.append(sample_line("bodyz 0", segments[offset + 1]))
    lines.append(sample_line(f"shapelegs {disabled_text} 260 110 80 1", segments[offset + 2]))
    lines.append(f"active {active_text}")
    lines.append(f"configquadpair {disabled_text}")
    lines.append(pose_sample_line(-1.0, 15.0, 0.0, active_text, segments[offset + 3]))
    lines.append(sample_line("bodyz 45", segments[offset + 4]))
    return "\n".join(lines) + "\n"


def impulse_script(segments: list[Segment]) -> str:
    if len(segments) < 13:
        raise ValueError(f"expected at least 13 impulse animation segments, got {len(segments)}")
    lines = ["constructor"]
    lines.append(sample_line("shape 265 60 30 1.07", segments[0]))
    lines.append(sample_line("shape 220 -40 55 1.15", segments[1]))
    lines.append(sample_line("bodyz 40", segments[2]))
    lines.append(sample_line("bodyz 0", segments[3]))
    lines.append(sample_line("bodyz 40", segments[4]))
    lines.append(sample_line("bodyzdelta 10", segments[5]))
    lines.append(sample_line("bodyzdelta -10", segments[6]))
    lines.append(sample_line("bodyzdelta 10", segments[7]))
    lines.append(sample_line("bodyzdelta -10", segments[8]))
    lines.append(sample_line("bodyzdelta 18", segments[9]))
    lines.append(sample_line("bodyzdelta 126", segments[10]))
    lines.append(sample_line("bodyzdelta -216", segments[11]))
    lines.append(sample_line("bodyzdelta 72", segments[12]))
    return "\n".join(lines) + "\n"


def active_leg_complement(disabled_pair: tuple[int, int]) -> list[int]:
    disabled = set(disabled_pair)
    return [leg for leg in (5, 2, 1, 0, 3, 4) if leg not in disabled]


def quad_pair_from_trace(path: Path) -> tuple[int, int] | None:
    for line in path.read_text().splitlines():
        if not line.strip():
            continue
        record = json.loads(line)
        command = str(record.get("command", ""))
        if not command.startswith("quad:"):
            continue
        parts = command.split(":", 1)[1].split(",")
        if len(parts) < 2:
            continue
        try:
            return int(parts[0]), int(parts[1])
        except ValueError:
            continue
    return None


def sample_line(prefix: str, segment: Segment) -> str:
    elapsed = " ".join(format(value, ".17g") for value, _ in segment.samples)
    return f"{prefix} {format(segment.duration, '.17g')} {elapsed}"


def pose_sample_line(
    threshold: float,
    lift: float,
    layer_blend: float,
    legs: str,
    segment: Segment,
) -> str:
    elapsed = " ".join(format(value, ".17g") for value, _ in segment.samples)
    return (
        f"pose {threshold:g} {lift:g} {layer_blend:g} "
        f"{format(segment.duration, '.17g')} {legs} {elapsed}"
    )


def max_delta(left: list[int], right: list[int]) -> int:
    return max(abs(a - b) for a, b in zip(left, right))


def run_replay(binary: Path, script: str) -> list[list[int]]:
    if not binary.exists() and binary.with_suffix(".exe").exists():
        binary = binary.with_suffix(".exe")
    result = subprocess.run(
        [str(binary)],
        input=script,
        text=True,
        check=True,
        stdout=subprocess.PIPE,
    )
    frames: list[list[int]] = []
    for line in result.stdout.splitlines():
        record = json.loads(line)
        frames.append(record["values"])
    return frames


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("oracle", type=Path)
    parser.add_argument("--binary", type=Path, default=Path("build/animation_replay"))
    parser.add_argument("--dump-script", type=Path)
    parser.add_argument("--strict", action="store_true")
    parser.add_argument("--diff-limit", type=int, default=20)
    parser.add_argument("--m-threshold", type=float, default=-1.0)
    parser.add_argument("--m-lift", type=float, default=15.0)
    parser.add_argument("--m-layer-blend", type=float, default=0.0)
    parser.add_argument("--m-legs", default="all")
    parser.add_argument(
        "--scenario",
        choices=("auto", "startup-sit-home", "standing-home", "quad", "impulse"),
        default="auto",
    )
    args = parser.parse_args()

    segments = pair_segments(args.oracle)
    script = replay_script(
        segments,
        args.m_threshold,
        args.m_lift,
        args.m_layer_blend,
        args.m_legs,
        args.scenario,
        quad_pair_from_trace(args.oracle),
    )
    if args.dump_script:
        args.dump_script.write_text(script)

    inferred = infer_scenario(segments) if args.scenario == "auto" else args.scenario
    if inferred == "standing-home":
        replayed_segment_count = 4
    elif inferred == "quad":
        replayed_segment_count = 7 if len(segments) >= 7 and segments[0].duration == 800.0 else 5
    elif inferred == "impulse":
        replayed_segment_count = 13
    else:
        replayed_segment_count = 5 if len(segments) >= 5 and segments[4].method == "M" else 4
    expected = [frame for segment in segments[:replayed_segment_count] for _, frame in segment.samples]
    actual = run_replay(args.binary, script)
    deltas = [max_delta(left, right) for left, right in zip(expected, actual)]

    print(f"segments={len(segments)}")
    for index, segment in enumerate(segments[:replayed_segment_count]):
        print(
            f"segment_{index}=method:{segment.method} duration:{segment.duration:g} "
            f"samples:{len(segment.samples)} first:{segment.samples[0][0]:g} "
            f"last:{segment.samples[-1][0]:g}"
        )
    print(f"oracle_frames={len(expected)} replay_frames={len(actual)}")
    print(f"exact_frames={sum(1 for delta in deltas if delta == 0)}")
    print(f"worst_delta={max(deltas, default=0)}")
    print(f"avg_delta={(sum(deltas) / len(deltas)) if deltas else 0.0:.3f}")

    mismatches = [i for i, delta in enumerate(deltas) if delta != 0]
    for frame_index in mismatches[: args.diff_limit]:
        left = expected[frame_index]
        right = actual[frame_index]
        per_pin = [abs(a - b) for a, b in zip(left, right)]
        pin = max(range(len(per_pin)), key=per_pin.__getitem__)
        print(
            f"diff frame={frame_index} max_delta={per_pin[pin]} pin={pin} "
            f"oracle={left[pin]} replay={right[pin]}"
        )

    exact = len(expected) == len(actual) and not mismatches
    print(f"exact={str(exact).lower()}")
    return 1 if args.strict and not exact else 0


if __name__ == "__main__":
    raise SystemExit(main())
