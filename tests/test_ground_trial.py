"""Exercise both serial opens of the ruler workflow through the actual POSIX monitor."""
import json
import os
from pathlib import Path
import pty
import select
import struct
import subprocess
import sys
import tempfile
import time
import tty

from test_monitor import imu, wire

script = Path(__file__).resolve().parents[1] / "docs/measurements/ground_trial.py"
with tempfile.TemporaryDirectory() as temp:
    folder = Path(temp)/"forward"
    master, slave = pty.openpty()
    tty.setraw(slave)
    proc = subprocess.Popen([sys.executable, "-B", str(script), sys.argv[1], os.ttyname(slave),
                             str(folder), "700", ".5", ".5", ".5", ".5"],
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    pending = bytearray()
    stop, enable = wire(1, b""), wire(2, b"")
    target = wire(3, struct.pack("<4i", *([500]*4)))
    active = started = False
    angle = tick = speed = 0
    next_sample = time.monotonic()
    deadline = next_sample + 5
    try:
        proc.stdin.write("\n")
        proc.stdin.flush()
        while proc.poll() is None:
            assert time.monotonic() < deadline
            if select.select([master], [], [], .005)[0]:
                pending.extend(os.read(master, 256))
            while 0 in pending:
                end = pending.index(0)+1
                command = bytes(pending[:end])
                del pending[:end]
                if command == stop:
                    active, speed = False, 0
                elif command == enable:
                    assert not started
                    active = started = True
                else:
                    assert command == target and active
                    speed = 500
            if time.monotonic() >= next_sample:
                next_sample += .02
                tick += 20
                angle += speed//50
                status = struct.pack("<IBBH4q4i", tick, int(active), 0 if active else 2, 12400,
                                     *([angle]*4), *([speed]*4))
                os.write(master, wire(0x81, status) + imu(tick))
        stdout, stderr = proc.communicate(timeout=1)
        assert proc.returncode == 0, (stdout, stderr)
        result = json.loads((folder/"trial.json").read_text())
        assert result["disabled_feedback_board_ms"] - result["ruler_cue_board_ms"] >= 500
        assert result["left_rad"] == result["right_rad"] and result["left_rad"] > .2
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        os.close(master)
        os.close(slave)
