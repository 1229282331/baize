import hashlib
import struct
from dataclasses import dataclass

CLASS_ID = bytes.fromhex("fe431dd9ebe85fa293ea0790b4a0472d")
MAX_IMAGE = 94 * 1024 - 2048


@dataclass(frozen=True)
class Identity:
    version: tuple
    digest: bytes

    def __str__(self):
        major, minor, revision, build = self.version
        return f"{major}.{minor}.{revision}+{build} {self.digest.hex()}"


def inspect_image(data):
    """Validate the complete unpadded MCUboot package before opening the port."""
    if not 520 <= len(data) <= MAX_IMAGE:
        raise ValueError("invalid update size (use the serial image, not the SWD image)")
    magic, address, header, protected, payload, flags = struct.unpack_from("<IIHHII", data)
    if (magic, address, header, flags) != (0x96F3B83D, 0, 512, 0) or payload < 8:
        raise ValueError("not a chassis MCUboot application")
    stack, entry = struct.unpack_from("<II", data, header)
    if not (0x20000000 < stack <= 0x2000C000 and stack % 8 == 0 and entry & 1 and
            0x08010200 <= entry - 1 < 0x08010200 + payload):
        raise ValueError("wrong application vectors")

    def records(start, end, expected):
        if not 0 <= start <= end - 4 <= len(data) - 4:
            raise ValueError("truncated metadata")
        magic, size = struct.unpack_from("<HH", data, start)
        if magic != expected or start + size != end:
            raise ValueError("invalid metadata length")
        pos = start + 4
        result = {}
        while pos < end:
            if end - pos < 4:
                raise ValueError("truncated metadata entry")
            tag, size = struct.unpack_from("<HH", data, pos)
            pos += 4
            if pos + size > end or tag in result:
                raise ValueError("invalid metadata entry")
            result[tag] = data[pos:pos + size]
            pos += size
        return result

    signed_end = header + payload + protected
    if records(header + payload, signed_end, 0x6908).get(0x75) != CLASS_ID:
        raise ValueError("image is for a different board")
    digest = hashlib.sha256(data[:signed_end]).digest()
    if records(signed_end, len(data), 0x6907).get(0x10) != digest:
        raise ValueError("image SHA-256 mismatch")
    return Identity(struct.unpack_from("<BBHI", data, 20), digest)
