"""Drive the timed jog CLI against a peer that can lose its wheel feedback."""
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import time

from test_monitor import imu, status, wire


def trial(mode):
    master, slave = pty.openpty()
    duration = "120" if mode == "normal" else "1800"
    args = [sys.argv[1], os.ttyname(slave), "--jog", duration, "1", "-0.7", "-0.5", "0.4"]
    proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stop, enable = wire(1, b""), wire(2, b"")
    target = wire(3, struct.pack("<4i", 1000, -700, -500, 400))
    opened = time.monotonic()
    sync_stops = 0
    incoming = bytearray()
    stop_ack = enabled = final_stop = False
    first_target = None
    frozen_ms = None
    board_ms = 0
    count = 0
    try:
        while proc.poll() is None:
            assert time.monotonic() - opened < 3, "handshake timed out"
            if select.select([master], [], [], 0.012)[0]:
                incoming.extend(os.read(master, 1024))
            now = time.monotonic()
            while 0 in incoming:
                end = incoming.index(0) + 1
                frame = bytes(incoming[:end])
                del incoming[:end]
                if frame == stop:
                    if first_target is not None:
                        if not final_stop:
                            elapsed = now - first_target
                        final_stop = True
                        enabled = False
                    else:
                        sync_stops += 1
                    # The first response is an in-flight reset status, before the STOP acknowledgement.
                    stop_ack = final_stop or sync_stops >= 2
                elif frame == enable:
                    assert stop_ack and first_target is None and not enabled
                    enabled = True
                else:
                    assert frame == target and enabled and not final_stop, frame.hex()
                    count += 1
                    if first_target is None:
                        first_target = now
                    if mode == "interrupt" and count == 2:
                        proc.send_signal(signal.SIGINT)
            board_ms += 20
            if mode == "loss" and first_target is not None and not final_stop:
                os.write(master, imu(board_ms))
            elif mode == "frozen" and first_target is not None and not final_stop:
                if frozen_ms is None:
                    frozen_ms = board_ms
                os.write(master, status(frozen_ms, 1, 0) + imu(board_ms))
            elif mode == "reset" and count >= 2 and not final_stop:
                board_ms = 20
                os.write(master, status(board_ms, 0, 1))
            else:
                os.write(master, status(board_ms, int(enabled), 0 if enabled else (2 if stop_ack else 1)))
        out, err = proc.communicate(timeout=1)
        assert final_stop and count >= 2 and "host_elapsed_ms" in out, (final_stop, count, err)
        if mode == "normal":
            assert proc.returncode == 0 and 0.095 <= elapsed < 0.23, (proc.returncode, elapsed, err)
        else:
            assert proc.returncode != 0 and elapsed < 0.6, (proc.returncode, elapsed, err)
        print(f"{mode}: {count} targets, first target to STOP {elapsed * 1000:.0f} ms")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()
        os.close(master)
        os.close(slave)


for mode in ("normal", "loss", "frozen", "reset", "interrupt"):
    trial(mode)
