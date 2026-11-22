"""Recover independent bias, unequal axis scales and a tilted sensor installation."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile

script = Path(__file__).resolve().parents[1] / "scripts/imu_calibration.py"
g = 9.80665
c, s = math.cos(.35), math.sin(.35)
body_axes_in_sensor = ((0, c, s), (-1, 0, 0), (0, -s, c))
offset = (.2, -.4, .3)
scale = (.98, 1.015, .955)
gyro = (-.09, .02, .035)

def record(path, axis, sign, start=2**32-1500):
    accel = [offset[i] + sign*g*scale[i]*body_axes_in_sensor[axis][i] for i in range(3)]
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(("board_ms","ax_m_s2","ay_m_s2","az_m_s2","gx_rad_s","gy_rad_s","gz_rad_s"))
        for n in range(120):
            w.writerow(((start+20*n) % 2**32, *accel, *gyro))

with tempfile.TemporaryDirectory() as temporary:
    folder = Path(temporary)
    for axis, name in enumerate("xyz"):
        for side, sign in (("up", 1), ("down", -1)):
            record(folder/f"{name}-{side}.csv", axis, sign)
    record(folder/"check-z-up.csv", 2, 1, start=10000)
    def run():
        return subprocess.run([sys.executable, "-B", str(script), str(folder)], text=True, capture_output=True)
    result = run()
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    for i in range(3):
        assert abs(data["accel_bias_m_s2"][i] - offset[i]) < 1e-6
        assert abs(data["accel_scale"][i] * scale[i] - 1) < 1e-6
        assert abs(data["gyro_bias_rad_s"][i] - gyro[i]) < 1e-6
        assert max(abs(a-b) for a,b in zip(data["sensor_to_body"][i],body_axes_in_sensor[i])) < 1e-6
    assert abs(data["check"]["gravity_m_s2"]-g) < 1e-6
    assert data["check"]["tilt_deg"] < .001
    record(folder/"check-z-up.csv", 2, 1)
    assert run().returncode != 0
    record(folder/"check-z-up.csv", 2, 1, start=10000)
    # Left-handed labels cannot define a ROS body frame.
    record(folder/"y-up.csv", 1, -1)
    record(folder/"y-down.csv", 1, 1)
    assert run().returncode != 0
