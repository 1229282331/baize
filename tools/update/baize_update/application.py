import binascii
import struct
import time
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
        stamps, imu_stamps = [], set()
        previous_angles = None
        start = time.monotonic()
        for kind, body in self.packets(start + 1.5):
            if kind == 0x82 and len(body) == 28:
                imu_stamps.add(int.from_bytes(body[:4], "little"))
            if kind == 0x81 and len(body) == 56:
                stamp, state, reason, _, *wheels = struct.unpack("<IBBH4q4i", body)
                angles = wheels[:4]
                consecutive = not stamps or 0 < (stamp - stamps[-1]) % (1 << 32) <= 60
                if not consecutive or angles != previous_angles:
                    stamps.clear()
                previous_angles = angles
                if state == 0 and reason == 2 and not any(wheels[4:]):
                    stamps.append(stamp)
                else:
                    stamps.clear()
            if (len(stamps) >= 10 and time.monotonic() - start >= 0.3 and
                    (not require_imu or len(imu_stamps) >= 2)):
                return len(imu_stamps) >= 2
        raise RuntimeError("stationary wheel feedback or expected IMU stream missing")
