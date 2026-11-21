"""Summarize constant-target wheel steps from 5 ms samples in a Baize bench ZIP."""

import csv
import io
import sys
from collections import defaultdict
from zipfile import ZipFile

WHEELS = ("fl", "fr", "rl", "rr")
SAMPLE_MS = 5
TAIL_SAMPLES = 100 // SAMPLE_MS
PWM_FULL_SCALE = 2000


def crossing_ms(times, directed_speeds, threshold):
    consecutive = 0
    for i, speed in enumerate(directed_speeds):
        consecutive = consecutive + 1 if speed >= threshold else 0
        if consecutive == 3:
            return times[i - 2]
    return None


def wheel_response(rows, start, wheel):
    target_key = f"target_mrad_s_{wheel}"
    active = [r for r in rows if int(r[target_key]) != 0]
    if len(active) < TAIL_SAMPLES:
        raise ValueError(f"Need 100 ms of feedback under the target for {wheel}")
    target = int(active[0][target_key])
    times = [(int(r["ms"]) - start) & 0xffffffff for r in active]
    if any(int(r[target_key]) != target for r in active) or any(b - a != SAMPLE_MS for a, b in zip(times, times[1:])):
        raise ValueError("Expected one constant target step sampled every 5 ms")

    sign = 1 if target > 0 else -1
    speed = [sign * int(r[f"speed_mrad_s_{wheel}"]) for r in active]
    magnitude = abs(target)
    peak = max(speed)
    tail = active[-TAIL_SAMPLES:]
    tail_effort = [abs(int(r[f"pwm_{wheel}"])) for r in tail]
    at_limit = sum(e == int(r["pwm_limit_counts"]) for e, r in zip(tail_effort, tail))
    return (
        target / 1000,
        crossing_ms(times, speed, magnitude * 0.1),
        crossing_ms(times, speed, magnitude * 0.9),
        sign * peak / 1000,
        round(100 * max(0, peak - magnitude) / magnitude, 1),
        sum(abs(v - magnitude) for v in speed[-TAIL_SAMPLES:]) / (TAIL_SAMPLES * 1000),
        100 * sum(tail_effort) / (TAIL_SAMPLES * PWM_FULL_SCALE),
        100 * at_limit / TAIL_SAMPLES,
    )


def summarize(path, output):
    trials = defaultdict(list)
    starts = {}
    with ZipFile(path) as archive:
        with io.TextIOWrapper(archive.open("samples.csv"), encoding="utf-8") as f:
            for row in csv.DictReader(f):
                trials[row["trial"]].append(row)
        with io.TextIOWrapper(archive.open("commands.csv"), encoding="utf-8") as f:
            for row in csv.DictReader(f):
                if row["op"] == "3":
                    for wheel in WHEELS:
                        if int(row[f"target_mrad_s_{wheel}"]):
                            starts.setdefault((row["trial"], wheel), int(row["ms"]))

    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(("trial", "wheel", "target_rad_s", "t10_ms", "t90_ms", "peak_rad_s",
                     "overshoot_pct", "tail_mae_rad_s", "tail_pwm_pct", "tail_at_limit_pct"))
    for (trial, wheel), start in starts.items():
        writer.writerow((trial, wheel, *wheel_response(trials[trial], start, wheel)))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: python wheel_response.py BENCH.zip")
    summarize(sys.argv[1], sys.stdout)
