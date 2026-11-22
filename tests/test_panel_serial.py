"""Exercise the desktop I/O worker, native session and logs over a POSIX serial port."""
import csv
import json
import os
import pty
import select
import struct
import sys
import tempfile
import time
import tty
from pathlib import Path
from queue import Empty

from test_monitor import imu, wire

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "host/panel"))
from panel_session import READY, Session, library


class Bench:
    def __init__(self, lib, folder):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.incoming = bytearray()
        self.commands = []
        self.enabled = False
        self.emit_wheel = self.emit_imu = True
        self.reason = 1
        self.tick = 1000
        self.speeds, self.angles = (0, 0, 0, 0), [0]*4
        self.next_sample = time.monotonic()
        self.folder, self.lib = folder, lib
        self.connect("first")

    def connect(self, name):
        self.session = Session(self.lib, os.ttyname(self.slave), self.folder/name)
        self.session.start()

    def step(self):
        if select.select([self.master], [], [], max(0, self.next_sample-time.monotonic()))[0]:
            self.incoming.extend(os.read(self.master, 1024))
        while 0 in self.incoming:
            end = self.incoming.index(0)+1
            frame = self.incoming[:end]
            del self.incoming[:end]
            if frame == wire(1, b""):
                kind = "STOP"
                self.enabled, self.reason = False, 2
                self.speeds = (0, 0, 0, 0)
            elif frame == wire(2, b""):
                assert self.reason == 2 and not self.enabled
                kind = "ENABLE"
                self.enabled, self.reason = True, 0
            else:
                kind = "TARGET"
                assert self.enabled and frame == wire(3, struct.pack("<4i", 1000, 0, 0, 0)), frame.hex()
                self.speeds = (1000, 0, 0, 0)
            self.commands.append((kind, time.monotonic()))
        if time.monotonic() >= self.next_sample:
            self.next_sample = time.monotonic()+.02
            self.tick += 20
            for i, v in enumerate(self.speeds):
                self.angles[i] += v//50
            data = b""
            if self.emit_wheel:
                data = wire(0x81, struct.pack("<IBBH4q4i", self.tick, int(self.enabled), self.reason,
                                              12400, *self.angles, *self.speeds))
            if self.emit_imu:
                data += imu(self.tick)
            if data:
                os.write(self.master, data)

    def wait(self, predicate, timeout=2):
        deadline = time.monotonic()+timeout
        while not predicate():
            assert time.monotonic() < deadline, (self.commands[-6:], self.session.serial_state, self.session.trial)
            self.step()

    def jog(self, ms):
        self.session.requests.put(("jog", "左前 · 正向", ms, (1, 0, 0, 0)))
        self.wait(lambda: self.speeds[0] != 0)

    def result(self, number):
        self.wait(lambda: self.session.trial is None)
        path = self.session.folder/f"trial-{number:03d}.json"
        return json.loads(path.read_text(encoding="utf-8"))

    def close(self):
        try:
            if self.session.is_alive():
                self.session.requests.put(("close",))
                if self.master is None:
                    self.session.join(timeout=2)
                else:
                    self.wait(lambda: not self.session.is_alive())
        finally:
            if self.master is not None:
                os.close(self.master)
            os.close(self.slave)


def read_rows(file):
    with file.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def exercise(lib, folder):
    b = Bench(lib, folder)
    try:
        b.wait(lambda: b.session.serial_state == READY and b.session.imu)
        assert b.commands and all(kind == "STOP" for kind, _ in b.commands)

        b.wait(lambda: b.tick >= 1200)
        assert all(kind == "STOP" for kind, _ in b.commands)

        b.emit_imu = False
        b.wait(lambda: b.session.elapsed()-b.session.imu[0] >= 260)
        assert b.session.serial_state == READY
        b.jog(400)
        first = next(t for kind, t in b.commands if kind == "TARGET")
        r = b.result(1)
        stop = next(t for kind, t in b.commands if kind == "STOP" and t > first)
        assert .36 <= stop-first < .60 and r["assessment"] == "反馈符合", (stop-first, r)
        assert r["mean_rad_s"] == [1, 0, 0, 0] and r["stop_ack"] and r["settled"]
        b.emit_imu = True
        b.wait(lambda: b.session.elapsed()-b.session.imu[0] < 100)

        b.jog(1800)
        b.session.requests.put(("stop",))
        assert b.result(2)["outcome"] == "stopped"

        b.jog(1800)
        b.enabled, b.reason, b.tick = False, 1, 0  # A board reset interrupts the current target.
        b.speeds = (0, 0, 0, 0)
        assert b.result(3)["outcome"] == "interrupted"
        b.wait(lambda: b.session.serial_state == READY)

        b.jog(1800)
        b.emit_wheel = False  # New IMU frames keep arriving on the same wire.
        lost_at = time.monotonic()
        b.wait(lambda: not b.enabled)
        assert .18 < time.monotonic()-lost_at < .55
        since_stop = len(b.commands)
        b.wait(lambda: not b.session.is_alive())
        assert all(k == "STOP" for k, _ in b.commands[since_stop:])
        assert b.result(4)["outcome"] == "disconnected"
        wheels = read_rows(b.session.folder/"wheels.csv")
        imus = read_rows(b.session.folder/"imu.csv")
        assert {r["trial"] for r in wheels} >= {"", "trial-001", "trial-004"}
        assert float(imus[-1]["az_m_s2"]) == 9.807
        assert int(imus[-1]["board_ms"])-int(wheels[-1]["board_ms"]) > 800
        assert any(float(r["fl_rad_s"]) == 1 and r["trial"] == "trial-001" for r in wheels)
        assert json.loads((b.session.folder/"session.json").read_text())["port"] == os.ttyname(b.slave)
        events = []
        while True:
            try:
                events.append(b.session.events.get_nowait())
            except Empty:
                break
        assert events[-1][0] == "closed" and "1 s" in events[-1][1]

        prior = len(b.commands)
        b.emit_wheel = True
        b.connect("reopened")
        b.wait(lambda: b.session.serial_state == READY and b.session.imu)
        b.session.requests.put(("pose", "车顶朝上"))
        b.wait(lambda: (b.session.folder/"poses/z-up.csv").is_file(), timeout=5)
        rows = read_rows(b.session.folder/"poses/z-up.csv")
        assert len(rows) >= 125 and all(r["trial"] == "" for r in rows)
        assert b.commands[prior:] and all(k == "STOP" for k, _ in b.commands[prior:])
        b.jog(600)
        b.session.requests.put(("close",))
        b.wait(lambda: not b.session.is_alive())
        assert not b.enabled and b.result(1)["outcome"] == "stopped"

        b.connect("unplug")
        b.wait(lambda: b.session.serial_state == READY)
        b.jog(1200)
        os.close(b.master)
        b.master = None
        b.session.join(timeout=2)
        assert not b.session.is_alive() and b.result(1)["outcome"] == "disconnected"
    finally:
        b.close()


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as directory:
        exercise(library(sys.argv[1]), Path(directory))
