"""Recompute one low-speed qualification row per wheel and trial in a Baize floor archive."""

import csv
import io
import sys
from collections import defaultdict
from itertools import accumulate
from math import ceil
from statistics import mean
from zipfile import ZipFile

from wheel_response import SAMPLE_MS, WHEELS, crossing_ms


def measure(rows, start_ms, target, mode):
    """All four wheels receive the same signed target in these tests."""
    sign = 1 if target > 0 else -1
    magnitude = abs(target)
    active = [r for r in rows if r["ms"] >= start_ms and r["target_mrad_s_fl"] == target]
    tail_n = 600 // SAMPLE_MS
    window = 100 // SAMPLE_MS
    if len(active) <= tail_n or any(b["ms"] - a["ms"] != SAMPLE_MS for a, b in zip(active, active[1:])):
        raise ValueError("Need a continuous target segment longer than 600 ms")
    times = [r["ms"] - start_ms for r in active]
    for wheel in WHEELS:
        if any(r["target_mrad_s_" + wheel] != target for r in active):
            raise ValueError("Expected equal wheel targets")
        speed = [sign * r["speed_mrad_s_" + wheel] for r in active]
        tail = speed[-tail_n:]
        errors = [abs(v - magnitude) for v in tail]
        prefix = list(accumulate(tail, initial=0))
        worst_window = max(abs((b - a) / window - magnitude) for a, b in zip(prefix, prefix[window:]))
        p95 = sorted(errors)[ceil(.95 * tail_n) - 1]
        # Speed is computed directly from signed encoder displacement over each 5 ms window.
        gap = longest = 0
        for v in tail:
            gap = gap + SAMPLE_MS if v <= 0 else 0
            longest = max(longest, gap)
        t50 = crossing_ms(times, speed, .5 * magnitude) if mode == "rest" else None
        t90 = crossing_ms(times, speed, .9 * magnitude) if mode == "rest" else None
        continuous = longest < 30 and worst_window <= .20 * magnitude
        effort = [r["pwm_" + wheel] for r in active[-(tail_n + 1):]]
        yield {
            "wheel": wheel, "t50_ms": t50, "t90_ms": t90,
            "max_no_progress_ms": longest, "worst_100ms_error_pct": 100 * worst_window / magnitude,
            "p95_error_pct": 100 * p95 / magnitude, "tail_mae_rad_s": mean(errors) / 1000,
            "overshoot_pct": 100 * max(0, max(speed) - magnitude) / magnitude,
            "mean_pwm_change": mean(abs(b - a) for a, b in zip(effort, effort[1:])),
            "start_ok": None if mode != "rest" else t50 is not None and t50 <= 250,
            "continuous_ok": continuous, "smooth_ok": continuous and p95 <= .25 * magnitude,
        }


def summarize(archive, output):
    with ZipFile(archive) as z:
        with io.TextIOWrapper(z.open("samples.csv"), encoding="ascii") as f:
            samples = defaultdict(list)
            for row in csv.DictReader(f):
                name = row.pop("trial")
                samples[name].append({key: int(val) for key, val in row.items()})
        with io.TextIOWrapper(z.open("trials.csv"), encoding="ascii") as f:
            writer = None
            for trial in csv.DictReader(f):
                name = trial["trial"]
                results = measure(samples[name], int(trial["start_ms"]), int(trial["target_mrad_s"]), trial["mode"])
                for result in results:
                    record = {**trial, **result}
                    if writer is None:
                        writer = csv.DictWriter(output, fieldnames=record, lineterminator="\n")
                        writer.writeheader()
                    writer.writerow(record)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("Usage: python wheel_floor.py BENCH.zip")
    summarize(sys.argv[1], sys.stdout)
