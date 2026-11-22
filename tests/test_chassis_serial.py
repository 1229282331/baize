"""Exercise the public examples against a v1 serial peer, including the second open."""
import os
import pty
import select
import struct
import subprocess
import sys
import time
import tty

from test_monitor import imu, wire


def exercise(executable, turn):
    master, slave = pty.openpty()
    tty.setraw(slave)
    args = [executable, os.ttyname(slave)] + (["--turn"] if turn else [])
    process = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    expected = (-500, 500, -500, 500) if turn else (500,) * 4
    incoming = bytearray()
    speeds, angles = (0,) * 4, [0] * 4
    enabled, reason, tick = False, 1, 0
    stops, enables, targets = 0, 0, 0
    next_sample = time.monotonic()
    deadline = next_sample + 4
    try:
        while process.poll() is None:
            assert time.monotonic() < deadline, (args, incoming, stops, targets)
            if select.select([master], [], [], max(0, next_sample - time.monotonic()))[0]:
                incoming.extend(os.read(master, 1024))
            while 0 in incoming:
                end = incoming.index(0) + 1
                frame = bytes(incoming[:end])
                del incoming[:end]
                if frame == wire(1, b""):
                    stops += 1
                    enabled, reason, speeds = False, 2, (0,) * 4
                elif frame == wire(2, b""):
                    assert stops and not enabled and not enables
                    enables += 1
                    enabled, reason = True, 0
                else:
                    assert enabled and frame == wire(3, struct.pack("<4i", *expected)), frame.hex()
                    speeds = expected
                    targets += 1
            if time.monotonic() >= next_sample:
                tick += 20
                angles = [a + v // 50 for a, v in zip(angles, speeds)]
                feedback = struct.pack("<IBBH4q4i", tick, int(enabled), reason, 12340, *angles, *speeds)
                os.write(master, wire(0x81, feedback) + imu(tick))
                next_sample = time.monotonic() + .02
        output, error = process.communicate(timeout=1)
        assert process.returncode == 0 and enables == 1 and targets > 1 and not enabled, (output, error)
        assert "imu " in output and "Stopped and reconnected with drive disabled." in output
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
        os.close(master)
        os.close(slave)


if __name__ == "__main__":
    for exe in sys.argv[1:]:
        exercise(exe, False)
        exercise(exe, True)
