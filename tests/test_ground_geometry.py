"""Exercise the geometry fit with known independent distances and signed turns."""
import csv
import math
from pathlib import Path
import subprocess
import sys
import tempfile

SCRIPT = Path(__file__).resolve().parents[1] / "docs/measurements/ground_geometry.py"
WHEELS = ("fl", "fr", "rl", "rr")


def record(path, time, angles, state, encoding="utf-8"):
    with path.open("w", newline="", encoding=encoding) as f:
        writer = csv.writer(f)
        writer.writerow(["board_ms", "drive_state", "stop_reason",
                         *(w+"_rad" for w in WHEELS), *(w+"_rad_s" for w in WHEELS)])
        for tick, wheels, active in zip(time, angles, state):
            writer.writerow([tick % 2**32, active, 0 if active else 2, *wheels, *([active]*4)])


def run(directory):
    for direction, angle in (("left", 90), ("right", -90)):
        result = subprocess.run([sys.executable, str(SCRIPT), "--turn-angle", direction, "20",
                                 str(20*math.sqrt(2))], text=True, capture_output=True)
        assert result.returncode == 0, result.stderr
        assert math.isclose(float(result.stdout.strip()), angle, abs_tol=.0001)
    for args in (("left", "0", "1"), ("right", "5", "11"), ("left", "nan", "1")):
        assert subprocess.run([sys.executable, str(SCRIPT), "--turn-angle", *args],
                              capture_output=True).returncode != 0

    manifest = directory / "runs.csv"
    radius, track = .05, .25
    names = ["fit-straight", "fit-turn", "check-straight", "check-turn"]
    with manifest.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["split", "motion", "reference", "run", "settled"])
        for name, reference in zip(names, (1.0, 90.0, -.7, -45.0)):
            fit, motion = name.split("-")
            q = reference/radius if motion == "straight" else math.radians(reference)*track/(2*radius)
            angles = [q]*4 if motion == "straight" else [-q, q, -q, q]
            start = 2**32 - 50
            record(directory/(name+".csv"), [start, start+20, start+40],
                   [[0]*4, [a*.5 for a in angles], [a*.9 for a in angles]], [0, 1, 0])
            record(directory/(name+"-rest.csv"), range(start+200, start+321, 20),
                   [angles]*7, [0]*7, encoding="utf-16")
            writer.writerow(["cal" if fit == "fit" else fit, motion, reference, name+".csv", name+"-rest.csv"])

    def evaluate():
        return subprocess.run([sys.executable, str(SCRIPT), str(manifest)], capture_output=True, text=True)

    result = evaluate()
    assert result.returncode == 0, result.stderr
    assert "wheel_radius_m=0.050000" in result.stdout and "wheel_separation_m=0.250000" in result.stdout
    assert result.stdout.count("error=+0.0000") == 2
    original = manifest.read_bytes()
    manifest.write_bytes(original.replace(b"check-turn.csv", b"fit-turn.csv"))
    assert evaluate().returncode != 0
    manifest.write_bytes(original)
    record(directory/"check-turn-rest.csv", [210, 230], [[0]*4]*2, [0, 0])
    assert evaluate().returncode != 0


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as directory:
        run(Path(directory))
