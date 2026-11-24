"""Exercise recovery on a stationary chassis. Run explicitly on the X5 with test packages."""
import argparse
import asyncio
from contextlib import closing
import hashlib
from pathlib import Path
import time

from smpclient import error
from smpclient.requests.image_management import ImageStatesWrite, ImageUploadWrite

from baize_update.application import Application, ENTER
from baize_update.cli import locked, read_package, recovery, stage


def check(condition, message):
    if not condition:
        raise RuntimeError(message)


async def interrupted(port, data):
    async with recovery(port) as (client, _):
        async for offset in client.upload(data, slot=2):
            if offset >= 2048:
                print(f"closed transfer at {offset}/{len(data)}", flush=True)
                return
        raise RuntimeError("transfer finished before interruption")


async def rejected(port, data, identity):
    async with recovery(port) as (client, _):
        # Direct upload ID 1 addresses primary in stock MCUboot. A header probe must be refused.
        r = await client.request(ImageUploadWrite(image=1, off=0, len=len(data), data=data[:32]))
        check(getattr(r, "rc", 0) and getattr(r, "off", None) is None, "unexpected primary upload response")
        print("primary upload refused", flush=True)
        signed_end = len(data) - 40
        for name in ("board", "digest", "truncated"):
            raw = bytearray(data)
            digest = identity.digest
            if name == "board":
                raw[signed_end - 1] ^= 1  # protected class UUID of this imgtool package
                digest = hashlib.sha256(raw[:signed_end]).digest()
                raw[-32:] = digest
            elif name == "digest":
                raw[512 + 32] ^= 1
            else:
                del raw[-16:]
            async for _ in client.upload(bytes(raw), slot=2):
                pass
            reply = await client.request(ImageStatesWrite(hash=digest, confirm=False))
            check(error(reply), f"unexpected trial approval for {name}")
            print(f"{name} candidate refused by MCUboot", flush=True)
        async for _ in client.upload(data, slot=2):
            pass
        permanent = await client.request(ImageStatesWrite(hash=identity.digest, confirm=True))
        check(error(permanent), "unexpected permanent upgrade approval")
        print("permanent request refused", flush=True)


def run(mode, port, package):
    data, identity = read_package(package)
    with locked(port):
        with closing(Application(port)) as app:
            original = app.info()
            check(not original.trial and original.identity != identity, "need a different confirmed baseline")
            had_imu = app.parked()
            print(f"baseline {original.identity}; candidate {identity}", flush=True)
            app.request(ENTER)
        time.sleep(0.08)
        if mode == "interrupt":
            asyncio.run(interrupted(port, data))
        elif mode == "reject":
            asyncio.run(rejected(port, data, identity))
        else:
            asyncio.run(stage(port, data, identity))
        start = time.monotonic()
        if mode == "no-confirm":
            with closing(Application(port)) as app:
                until = start + 7
                while time.monotonic() < until:
                    try:
                        trial = app.info()
                        break
                    except TimeoutError:
                        continue
                else:
                    raise RuntimeError("trial application did not answer")
                check(trial.trial and trial.identity == identity, "wrong trial image")
                app.parked(require_imu=had_imu, check_startup=True)
                print("trial identity, wheels and IMU checked; withholding confirmation", flush=True)
        # No serial transmitter or watchdog helper is connected during recovery.
        silence = 16 if mode != "early-hang" else 10
        print(f"serial closed for {silence} s", flush=True)
        time.sleep(silence)
        with closing(Application(port)) as app:
            restored = app.info()
            check(restored == original, f"unexpected recovery image {restored}")
            app.parked(require_imu=had_imu)
            print(f"observed confirmed baseline and stopped wheels after {time.monotonic()-start:.2f} s",
                  flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=["interrupt", "reject", "no-confirm", "early-hang"])
    parser.add_argument("package", type=Path)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    run(args.mode, args.port, args.package)
