"""Calibrate six static body poses and evaluate a separate level run from baize-monitor CSVs."""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, pstdev

G = 9.80665
FIELDS = ("ax_m_s2", "ay_m_s2", "az_m_s2", "gx_rad_s", "gy_rad_s", "gz_rad_s")


def read_pose(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    ticks = [int(r["board_ms"]) for r in rows]
    if len(rows) < 100 or any(not 0 < (b-a) % 2**32 < 250 for a, b in zip(ticks, ticks[1:])):
        raise ValueError(f"{path}: need at least 100 successive fresh IMU samples")
    columns = [[float(r[field]) for r in rows] for field in FIELDS]
    if not all(math.isfinite(v) for column in columns for v in column):
        raise ValueError(f"{path}: non-finite sample")
    return {"samples": len(rows), "first_board_ms": ticks[0], "last_board_ms": ticks[-1],
            "mean": [mean(c) for c in columns], "sd": [pstdev(c) for c in columns]}


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def unit(v):
    length = math.hypot(*v)
    if length < 1e-8:
        raise ValueError("pose markings do not define distinct axes")
    return [x/length for x in v]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def calibrate(folder):
    poses = {f"{axis}-{side}": read_pose(folder/f"{axis}-{side}.csv")
             for axis in "xyz" for side in ("up", "down")}
    bias = [mean(p["mean"][i] for p in poses.values()) for i in range(6)]
    # Each pair estimates one body axis expressed in uncorrected sensor coordinates.
    axes = [[(poses[f"{a}-up"]["mean"][i] - poses[f"{a}-down"]["mean"][i]) / (2*G)
             for i in range(3)] for a in "xyz"]
    gains = [math.hypot(*column) for column in zip(*axes)]
    if min(gains) < .1:
        raise ValueError("check all six body poses and the sensor axes")
    scale = [1/g for g in gains]
    x, y, z = [unit([v*k for v, k in zip(axis, scale)]) for axis in axes]
    # Level defines +Z. The front fixes heading. The left-hand side is measured independently.
    left = unit(cross(z, x))
    if dot(left, y) < .5:
        raise ValueError("body labels must form a right-handed forward, left, up frame")
    rotation = [cross(left, z), left, z]

    check = read_pose(folder/"check-z-up.csv")
    if check in poses.values():
        raise ValueError("record a separate level run for the independent check")
    accel = [(v-b)*k for v, b, k in zip(check["mean"][:3], bias[:3], scale)]
    body = [dot(row, accel) for row in rotation]
    gyro = [v-b for v, b in zip(check["mean"][3:], bias[3:])]
    return {"accel_bias_m_s2": bias[:3], "accel_scale": scale, "gyro_bias_rad_s": bias[3:],
            "sensor_to_body": rotation, "poses": poses, "check": {
                "samples": check["samples"], "accel_body_m_s2": body, "gravity_m_s2": math.hypot(*body),
                "tilt_deg": math.degrees(math.atan2(math.hypot(*body[:2]), body[2])),
                "gyro_body_rad_s": [dot(row, gyro) for row in rotation]}}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    try:
        print(json.dumps(calibrate(parser.parse_args().folder), indent=2, allow_nan=False))
    except (OSError, KeyError, ValueError) as error:
        parser.exit(1, f"{error}\n")
