import binascii
import struct
import time
from collections import deque
from dataclasses import dataclass

from cobs import cobs
from serial import Serial

from .image import Identity

INFO, ENTER, CONFIRM = 1, 2, 3


def frame(kind, payload=b""):
    raw = bytes((1, kind)) + payload
    return cobs.encode(raw + struct.pack("<H", binascii.crc_hqx(raw, 0xffff))) + b"\x00"


@dataclass(frozen=True)
class Reply:
    identity: Identity
    trial: bool


class Application:
    def __init__(self, port):
        self.serial = Serial(port, 115200, timeout=0.04, write_timeout=0.5)
        self.serial.reset_input_buffer()

    def close(self):
        self.serial.close()

    def packets(self, deadline):
        partial = bytearray()
        while time.monotonic() < deadline:
            block = self.serial.read_until(b"\x00", size=128)
            if not block:
                continue
            partial.extend(block)
            if not partial.endswith(b"\x00"):
                if len(partial) > 128:
                    partial.clear()
                continue
            try:
                raw = cobs.decode(bytes(partial[:-1]))
            except cobs.DecodeError:
                raw = b""
            partial.clear()
            if (4 <= len(raw) <= 60 and raw[0] == 1 and
                    binascii.crc_hqx(raw[:-2], 0xffff) == int.from_bytes(raw[-2:], "little")):
                yield raw[1], raw[2:-2]

    def request(self, operation, digest=bytes(32), timeout=0.55):
        # A newly opened tty may inherit a partial frame from the previous owner.
        self.serial.write(b"\x00" + frame(4, bytes([operation]) + digest))
        for kind, body in self.packets(time.monotonic() + timeout):
            if kind != 0x83 or len(body) != 43 or body[0] != operation:
                continue
            if body[1]:
                raise RuntimeError(f"chassis refused operation {operation}, result {body[1]}")
            return Reply(Identity(struct.unpack_from("<BBHI", body, 3), body[11:]), bool(body[2]))
        raise TimeoutError("no fresh maintenance reply")

    def info(self):
        for _ in range(2):
            try:
                return self.request(INFO, timeout=0.22)
            except TimeoutError:
                pass
        return self.request(INFO, timeout=0.22)

    def parked(self, require_imu=False, check_startup=False):
        self.serial.reset_input_buffer()
        if check_startup:
            for kind, body in self.packets(time.monotonic() + 0.45):
                if kind == 0x81 and len(body) == 56:
                    if body[4] != 0 or any(struct.unpack_from("<4i", body, 40)):
                        self.serial.write(b"\x00" + frame(1))
                        raise RuntimeError("candidate started with enabled drive or wheel motion")
                    break
            else:
                raise RuntimeError("candidate did not report its initial wheel state")
        self.serial.write(b"\x00" + frame(1))
        wheel_tick = imu_tick = angles = None
        imu_times = deque(maxlen=2)
        count = 0
        stationary = False
        last_wheel = 0.0
        deadline = time.monotonic() + 1.5
        for kind, body in self.packets(deadline):
            now = time.monotonic()
            if kind == 0x82 and len(body) == 28:
                tick = int.from_bytes(body[:4], "little")
                gap = 0 if imu_tick is None else (tick - imu_tick) % 2**32
                if imu_tick is None or 0 < gap < 2**31:
                    if gap >= 250:
                        imu_times.clear()
                    imu_tick = tick
                    imu_times.append(now)
                continue
            if kind != 0x81 or len(body) != 56:
                continue
            tick, state, reason, _, *wheels = struct.unpack("<IBBH4q4i", body)
            gap = 0 if wheel_tick is None else (tick - wheel_tick) % 2**32
            if wheel_tick is not None and not 0 < gap < 2**31:
                continue
            if wheels[:4] != angles or gap > 60 or now - last_wheel > 0.060:
                count = 0
            wheel_tick, last_wheel, angles = tick, now, wheels[:4]
            if state != 0 or reason != 2 or any(wheels[4:]):
                count, stationary = 0, False
                continue
            if count == 0:
                first_tick, first_rx = tick, now
            count += 1
            stationary = count >= 10 and (tick - first_tick) % 2**32 >= 300 and now - first_rx >= 0.3
            if stationary and len(imu_times) == 2 and now - imu_times[0] < 0.250:
                # Both sensors timestamp on the same MCU clock, including across wrap.
                if min((tick - imu_tick) % 2**32, (imu_tick - tick) % 2**32) < 250:
                    return True
        if not require_imu and imu_tick is None and stationary and time.monotonic() - last_wheel <= 0.060:
            return False
        raise RuntimeError("stationary wheel feedback or expected IMU stream missing")
