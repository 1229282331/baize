"""Exercise the real monitor on a POSIX pseudo-terminal with independent v1 frames."""
import binascii
import csv
import io
import os
import pty
import select
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def wire(kind, payload):
    raw = bytes([1, kind]) + payload
    raw += struct.pack("<H", binascii.crc_hqx(raw, 0xffff))
    return b"".join(bytes([len(run) + 1]) + run for run in raw.split(b"\0")) + b"\0"


def status(ms, state=0, reason=2):
    return wire(0x81, struct.pack("<IBBH4q4i", ms, state, reason, 12400, *([0] * 8)))


def imu(ms):
    return wire(0x82, struct.pack("<I6i", ms, 100, -200, 9807, 0, -250, 500))


def run_case(imu_path, wheels):
    master, slave = pty.openpty()
    command = [sys.argv[1], os.ttyname(slave), *(["14"] if wheels else []), "--imu-csv", str(imu_path)]
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert select.select([master], [], [], 3)[0], "no startup STOP"
        assert os.read(master, 6) == bytes.fromhex("05 01 01 1f 3e 00")
        for i in range(13):
            if proc.poll() is not None:
                break
            # Duplicate IMU samples must age out even while valid wheel samples arrive.
            os.write(master, imu(0) + status(i * 100) if wheels else imu(i * 100))
            time.sleep(0.1)
        if wheels:
            resumed = imu(1299)
            os.write(master, resumed[:9])
            os.write(master, resumed[9:] + status(1300))
        out, err = proc.communicate(timeout=3)
        rows = list(csv.DictReader(io.StringIO(out)))
        with imu_path.open() as stream:
            samples = list(csv.DictReader(stream))
        if wheels:
            assert proc.returncode == 0 and len(rows) == 14, (err, rows)
            assert [int(row["board_ms"]) for row in rows] == list(range(0, 1400, 100))
            assert [row["board_ms"] for row in samples] == ["0", "1299"]
            assert list(samples[-1].values())[1:] == [
                "1299", "0.100", "-0.200", "9.807", "0.000", "-0.250", "0.500"]
            assert err.count("No valid IMU") == 1 and err.count("IMU telemetry resumed") == 1, err
        else:
            assert proc.returncode == 1 and samples and not rows, (err, rows)
            assert "No valid chassis status" in err, err
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as directory:
        run_case(Path(directory) / "wheel-and-imu.csv", wheels=True)
        run_case(Path(directory) / "imu-only.csv", wheels=False)
