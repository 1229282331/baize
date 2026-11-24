"""Check the three-minute outgoing inspection for a four-wheel chassis fitted with an IMU."""
import argparse
import csv
import json
import math
from pathlib import Path


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def timing(rows, name):
    require(len(rows) >= 2, f"{name}: insufficient samples")
    ticks = [int(row["board_ms"]) for row in rows]
    rx = [int(row["host_elapsed_ms"]) for row in rows]
    gaps = [(b - a) % 2**32 for a, b in zip(ticks, ticks[1:])]
    host_gaps = [b - a for a, b in zip(rx, rx[1:])]
    require(all(0 < gap < 250 for gap in gaps), f"{name}: stale or unordered board time")
    require(all(0 <= gap < 250 for gap in host_gaps), f"{name}: host reception gap")
    hz = 1000 * len(gaps) / sum(gaps)
    require(48 <= hz <= 52, f"{name}: observed {hz:.2f} Hz")
    span = rx[-1] - rx[0]
    require(span > 0 and 48 <= 1000 * len(host_gaps) / span <= 52, f"{name}: host sampling span")
    return dict(samples=len(rows), first_host_ms=rx[0], last_host_ms=rx[-1],
                board_duration_ms=sum(gaps), frequency_hz=round(hz, 4),
                max_board_gap_ms=max(gaps), max_host_gap_ms=max(host_gaps))


def wheels(row, suffix):
    return tuple(float(row[w + suffix]) for w in ("fl", "fr", "rl", "rr"))


def inspect(folder):
    wheel = read_csv(folder / "stationary-wheels.csv")
    imu = read_csv(folder / "stationary-imu.csv")
    stats = {"wheel": timing(wheel, "wheels"), "imu": timing(imu, "IMU")}
    require(len(wheel) == 9000, "incomplete stationary wheel recording")
    for edge in ("first_host_ms", "last_host_ms"):
        require(abs(stats["wheel"][edge] - stats["imu"][edge]) < 250, f"IMU coverage: {edge}")
    require(all(row["drive_state"] == "0" and row["stop_reason"] == "2" for row in wheel),
            "drive left the requested stop state")
    require(all(math.isfinite(float(r[k])) for r in imu for k in
                ("ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s", "gz_rad_s")), "invalid IMU")
    angles = [wheels(row, "_rad") for row in wheel]
    speeds = [wheels(row, "_rad_s") for row in wheel]
    last_moving = max((i for i, v in enumerate(speeds) if any(v)), default=-1)
    last_change = max((i for i in range(1, len(angles)) if angles[i] != angles[i-1]), default=0)
    settled = max(last_moving + 1, last_change)
    require(settled < len(wheel) and int(wheel[settled]["host_elapsed_ms"]) < 1000, "wheels did not settle")
    stats["settled_from_host_ms"] = int(wheel[settled]["host_elapsed_ms"])
    stats["initial_velocity_rad_s"] = speeds[0]
    volts = [float(row["supply_v"]) for row in wheel]
    require(all(math.isfinite(v) and v > 0 for v in volts), "invalid supply telemetry")
    stats["voltage_v"] = [min(volts), max(volts)]
    stats["duration_s"] = (stats["wheel"]["last_host_ms"] - stats["wheel"]["first_host_ms"]) / 1000

    jog = read_csv(folder / "jog-wheels.csv")
    active = [row for row in jog if row["drive_state"] == "1"]
    require(len(active) >= 10 and jog[-1]["drive_state"] == "0" and jog[-1]["stop_reason"] == "2",
            "jog did not run and acknowledge STOP")
    means = [sum(v) / 5 for v in zip(*(wheels(row, "_rad_s") for row in active[-5:]))]
    require(all(0.35 <= v <= 0.65 for v in means), "wheel outside +0.50 rad/s inspection tolerance")
    stats["jog_target_rad_s"] = [0.5] * 4
    stats["jog_last_five_mean_rad_s"] = [round(v, 3) for v in means]
    return stats


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", type=Path)
    try:
        print(json.dumps(inspect(parser.parse_args().records), indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, csv.Error) as exc:
        parser.exit(1, f"Inspection failed: {exc}\n")
