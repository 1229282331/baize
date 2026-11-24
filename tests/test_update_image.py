import hashlib
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/update"))
from baize_update.image import CLASS_ID, inspect_image


def package(board=CLASS_ID):
    header = bytearray(512)
    struct.pack_into("<IIHHIIBBHI", header, 0, 0x96F3B83D, 0, 512, 24, 12, 0, 1, 1, 0, 7)
    app = struct.pack("<III", 0x2000c000, 0x08010209, 0)
    protected = struct.pack("<HHHH", 0x6908, 24, 0x75, 16) + board
    body = header + app + protected
    return body + struct.pack("<HHHH", 0x6907, 40, 0x10, 32) + hashlib.sha256(body).digest()


class UpdateImageTest(unittest.TestCase):
    def test_hash_covers_header_application_and_board(self):
        data = package()
        self.assertEqual(inspect_image(data).version, (1, 1, 0, 7))
        for offset in (20, 521, -1):
            changed = data.copy()
            changed[offset] ^= 1
            with self.assertRaises(ValueError):
                inspect_image(changed)

    def test_other_board_even_with_valid_hash(self):
        with self.assertRaisesRegex(ValueError, "different board"):
            inspect_image(package(bytes(16)))

    def test_rejects_short_extra_and_unbounded_metadata(self):
        data = package()
        for invalid in (data[:20], data[:-1], data + b"\xff", data[:12] + b"\xff"*4 + data[16:]):
            with self.assertRaises(ValueError):
                inspect_image(invalid)


if __name__ == "__main__":
    unittest.main()
