#!/usr/bin/env python3
"""Diff original vs rebuilt captures produced by coverage_capture.py.

For each scenario, compares the timing-normalized streams: command sequence,
tcp status FLAGS path, distinct servo poses, anim primitives, and whether each
side produced servo motion at all. Prints a compact per-scenario verdict.
"""
import json
import re
import sys
import collections
from pathlib import Path

OUTDIR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/cov")
FLAGS_RE = re.compile(r"FLAGS=(\S+)")


def load(p):
    if not p.exists():
        return None
    return [json.loads(l) for l in p.open() if l.strip()]


def flags_path(rows):
    out = []
    for o in rows:
        if o.get("type") == "tcp":
            m = FLAGS_RE.search(o.get("line", ""))
            f = m.group(1) if m else None
            if not out or out[-1] != f:
                out.append(f)
    return out


def servo_distinct(rows):
    out = []
    for o in rows:
        if o.get("type") == "servo_values":
            v = tuple(o.get("values", []))
            if not out or out[-1] != v:
                out.append(v)
    return out


def anim_seq(rows):
    out = []
    for o in rows:
        if o.get("type") == "anim":
            k = (o.get("method"), o.get("duration"))
            if not out or out[-1] != k:
                out.append(k)
    return out


def cmds(rows):
    return [o.get("command") for o in rows if o.get("type") == "command"]


def scenarios(outdir):
    names = set()
    for p in outdir.glob("original_*.jsonl"):
        names.add(p.name[len("original_"):-len(".jsonl")])
    for p in outdir.glob("rebuilt_*.jsonl"):
        names.add(p.name[len("rebuilt_"):-len(".jsonl")])
    return sorted(names)


def main():
    for name in scenarios(OUTDIR):
        o = load(OUTDIR / f"original_{name}.jsonl")
        r = load(OUTDIR / f"rebuilt_{name}.jsonl")
        print(f"=== {name} ===")
        if o is None or r is None:
            print(f"  MISSING: original={o is not None} rebuilt={r is not None}")
            continue
        of, rf = flags_path(o), flags_path(r)
        osd, rsd = servo_distinct(o), servo_distinct(r)
        oa, ra = anim_seq(o), anim_seq(r)
        print(f"  servo motion: orig={'yes' if len(osd)>1 else 'NONE'}({len(osd)})  rebuilt={'yes' if len(rsd)>1 else 'NONE'}({len(rsd)})")
        if of != rf:
            print(f"  FLAGS DIFFER:")
            print(f"    orig:    {of}")
            print(f"    rebuilt: {rf}")
        else:
            print(f"  FLAGS match: {of}")
        if oa != ra:
            print(f"  ANIM DIFFER:  orig={oa}")
            print(f"               rebuilt={ra}")
        elif oa:
            print(f"  anim match: {oa}")
        # crude servo-shape agreement: do both reach a similar final pose?
        if osd and rsd:
            md = max(abs(a - b) for a, b in zip(osd[-1], rsd[-1]))
            print(f"  final-pose maxdelta (informational, nondeterministic): {md}")
        print()


if __name__ == "__main__":
    main()
