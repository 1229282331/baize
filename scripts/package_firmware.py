"""Create the serial update and, optionally, the SWD provisioning image with MCUboot imgtool."""
import argparse
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile


def vectors(data, base):
    if len(data) < 8:
        raise ValueError("missing Cortex-M3 vector table")
    stack, entry = struct.unpack_from("<II", data)
    if not (0x20000000 < stack <= 0x2000C000 and stack % 8 == 0 and entry & 1 and
            base <= entry - 1 < base + len(data)):
        raise ValueError(f"invalid Cortex-M3 vectors for address {base:#010x}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--app", required=True, type=Path)
    parser.add_argument("--boot", type=Path)
    parser.add_argument("--version", required=True)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    app = args.app.read_bytes()
    vectors(app, 0x08010200)
    if len(app) + 512 + 128 > 94 * 1024 - 2048:
        raise ValueError("application overlaps its reserved trailer page")
    args.output.mkdir(parents=True, exist_ok=True)
    tool = shutil.which("imgtool")
    if not tool:
        parser.error("activate the MCUboot imgtool environment")
    sign = [tool, "sign", "--header-size", "512", "--pad-header", "--align", "2",
            "--slot-size", str(94 * 1024), "--max-sectors", "48", "--version", args.version,
            "--cid", "fe431dd9-ebe8-5fa2-93ea-0790b4a0472d"]
    with tempfile.TemporaryDirectory(dir=args.output) as directory:
        tmp = Path(directory)
        update = tmp / "update.bin"
        subprocess.run([*sign, args.app, update], check=True)
        if args.boot:
            boot = args.boot.read_bytes()
            vectors(boot, 0x08000000)
            if len(boot) > 65536:
                raise ValueError("bootloader exceeds 64 KiB")
            primary = tmp / "primary.bin"
            subprocess.run([*sign, "--confirm", args.app, primary], check=True)
            initial = primary.read_bytes()
            if (len(initial) != 94 * 1024 or initial[-24] != 1 or initial[-32] != 0xff or
                    initial[:update.stat().st_size] != update.read_bytes()):
                raise ValueError("invalid confirmed primary layout")
            factory = tmp / "factory.bin"
            factory.write_bytes(boot.ljust(65536, b"\xff") + initial +
                                b"\xff" * (96 * 1024))
            factory.replace(args.output / "baize-factory.bin")
        update.replace(args.output / f"baize-{args.version}.bin")


if __name__ == "__main__":
    main()
