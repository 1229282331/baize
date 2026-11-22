"""Fit empty-chassis rolling geometry and report independent ground-reference errors."""
import argparse
import csv
import math
from itertools import groupby
from pathlib import Path


def read_rows(path):
    # Windows PowerShell 5 redirects native stdout to UTF-16LE with a BOM.
    with path.open("rb") as stream:
        encoding = "utf-16" if stream.read(2) in (b"\xff\xfe", b"\xfe\xff") else "utf-8-sig"
    with path.open(encoding=encoding, newline="") as stream:
        return list(csv.DictReader(stream))


def wheel_travel(run, settled):
    wheels = ("fl", "fr", "rl", "rr")
    rows = read_rows(run)
    if settled:
        rows.extend(read_rows(settled))
    if len(rows) < 2:
        raise ValueError(f"{run}: missing wheel samples")
    ticks = [int(r["board_ms"]) for r in rows]
    states = [int(r["drive_state"]) for r in rows]
    if any(not 0 < (b-a) % 2**32 < 2**31 for a, b in zip(ticks, ticks[1:])):
        raise ValueError(f"{run}: discontinuous board clock")
    if [state for state, _ in groupby(states)] != [0, 1, 0]:
        raise ValueError(f"{run}: expected one enabled interval between two disabled intervals")
    after_stop = len(states) - states[::-1].index(1)
    if any(int(r["stop_reason"]) != 2 for r in rows[after_stop:]):
        raise ValueError(f"{run}: unexpected stop or reset")

    def angles(row):
        return [float(row[w + "_rad"]) for w in wheels]

    def stationary(row):
        return row["drive_state"] == "0" and all(float(row[w + "_rad_s"]) == 0 for w in wheels)

    first, last = angles(rows[0]), angles(rows[-1])
    quiet_tick = ticks[-1]
    for row in reversed(rows):
        tick = int(row["board_ms"])
        if (quiet_tick-tick) % 2**32 > 40 or not stationary(row) or angles(row) != last:
            break
        quiet_tick = tick
    if not stationary(rows[0]) or (ticks[-1]-quiet_tick) % 2**32 < 100:
        raise ValueError(f"{run}: need a stationary start and at least 100 ms at rest after STOP")
    fl, fr, rl, rr = [b-a for a, b in zip(first, last)]
    return (fl+rl)/2, (fr+rr)/2


def calibrate(manifest):
    samples = {("cal", "straight"): [], ("cal", "turn"): [],
               ("check", "straight"): [], ("check", "turn"): []}
    used = set()
    for row in read_rows(manifest):
        key = row["split"], row["motion"]
        if key not in samples:
            raise ValueError(f"unknown split or motion: {key}")
        run = (manifest.parent / row["run"]).resolve()
        settled = (manifest.parent / row["settled"]).resolve() if row["settled"] else None
        paths = [run, settled] if settled else [run]
        if any(p in used for p in paths) or run == settled:
            raise ValueError(f"each run needs independent recordings: {run}")
        used.update(paths)
        left, right = wheel_travel(run, settled)
        straight = row["motion"] == "straight"
        wheel_angle = (left+right)/2 if straight else right-left
        ref = float(row["reference"])
        if not straight:
            ref = math.radians(ref)
        if not math.isfinite(ref*wheel_angle) or ref*wheel_angle <= 0:
            raise ValueError(f"{run}: check reference units and direction")
        samples[key].append((run.name, wheel_angle, ref))
    if not all(samples.values()):
        raise ValueError("supply independent calibration and check runs for both straight and turn")

    straight = samples["cal", "straight"]
    turns = samples["cal", "turn"]
    radius = sum(q*d for _, q, d in straight) / sum(q*q for _, q, _ in straight)
    track = radius * sum(yaw*q for _, q, yaw in turns) / sum(yaw*yaw for _, _, yaw in turns)
    print(f"wheel_radius_m={radius:.6f}  circumference_m={2*math.pi*radius:.6f}  wheel_separation_m={track:.6f}")
    for motion in ("straight", "turn"):
        for name, q, ref in samples["check", motion]:
            expected, observed, unit = radius*q, ref, "m"
            if motion == "turn":
                expected, observed, unit = math.degrees(radius*q/track), math.degrees(ref), "deg"
            print(f"{name}: reference={observed:.4f}  odometry={expected:.4f}  error={expected-observed:+.4f} {unit}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("manifest", nargs="?", type=Path)
    mode.add_argument("--turn-angle", nargs=3, metavar=("SIDE", "R", "D"),
                      help="signed degrees from two forward rays, equal radius R and chord D (same units)")
    args = parser.parse_args()
    try:
        if args.turn_angle:
            side, radius, chord = args.turn_angle
            r, d = float(radius), float(chord)
            if side not in ("left", "right") or not math.isfinite(r) or not math.isfinite(d) or not 0 < d <= 2*r:
                raise ValueError("use left or right, finite R > 0 and 0 < D <= 2R")
            degrees = math.degrees(2 * math.asin(d / (2*r)))
            print(f"{degrees if side == 'left' else -degrees:.6f}")
        else:
            calibrate(args.manifest)
    except (OSError, KeyError, ValueError) as error:
        parser.exit(1, f"{error}\n")
