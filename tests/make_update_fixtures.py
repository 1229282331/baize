"""Build disposable acceptance images for tests/qualify_update.py with the release imgtool."""
import argparse
from pathlib import Path
import struct
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("app", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--version", default="1.1.0", help="major.minor.revision")
    args = parser.parse_args()
    package = Path(__file__).resolve().parents[1] / "scripts/package_firmware.py"
    args.output.mkdir(parents=True, exist_ok=True)

    def pack(path, build):
        version = f"{args.version}+{build}"
        subprocess.run([sys.executable, package, "--app", path, "--version", version,
                        "--output", args.output], check=True)
        return args.output / f"baize-{version}.bin"

    normal = pack(args.app, 1)
    raw = args.app.read_bytes()
    with tempfile.TemporaryDirectory() as directory:
        app = Path(directory) / "fixture.bin"
        stalled = bytearray(raw)
        if len(stalled) % 2:
            stalled.append(0xff)
        # Cortex-M3 Reset_Handler branches to itself before the application can feed IWDG.
        struct.pack_into("<I", stalled, 4, (0x08010200 + len(stalled)) | 1)
        app.write_bytes(stalled + bytes.fromhex("fe e7"))
        pack(app, 2)
        metadata = normal.stat().st_size - len(raw)
        app.write_bytes(raw.ljust(94208 - metadata, b"\xa5"))
        if pack(app, 3).stat().st_size != 94208:
            raise ValueError("unexpected maximum image size")


if __name__ == "__main__":
    main()
