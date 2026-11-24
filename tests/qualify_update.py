"""Exercise recovery on a stationary chassis. Run explicitly on the X5 with test packages."""
import argparse
import asyncio
from contextlib import closing
import hashlib
from pathlib import Path
import time
from unittest.mock import patch

from smpclient import error
from smpclient.requests.image_management import ImageStatesWrite, ImageUploadWrite

from baize_update.application import Application, CONFIRM, ENTER
from baize_update.cli import locked, read_package, recovery, stage, verify_trial


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


def verify_missing_stream(port, identity, mode):
    counts = {0x81: 0, 0x82: 0}

    class FilteredApplication(Application):
        observing_since = None

        def parked(self, **kwargs):
            self.observing_since = time.monotonic()
            return super().parked(**kwargs)

        def request(self, operation, *args, **kwargs):
            check(operation != CONFIRM, "attempted to confirm despite missing telemetry")
            return super().request(operation, *args, **kwargs)

        def packets(self, deadline):
            for kind, body in super().packets(deadline):
                if self.observing_since is not None:
                    elapsed = time.monotonic() - self.observing_since
                    if mode == "missing-wheels":
                        drop = (kind == 0x81 and elapsed > 0.27) or (kind == 0x82 and elapsed < 0.70)
                    else:
                        drop = kind == 0x82 and elapsed > 0.05
                    if drop:
                        counts[kind] += 1
                        continue
                yield kind, body

    with patch("baize_update.cli.Application", FilteredApplication):
        try:
            verify_trial(port, identity, require_imu=True)
        except RuntimeError as exc:
            check("stationary wheel feedback or expected IMU stream missing" in str(exc), str(exc))
        else:
            raise RuntimeError("unhealthy candidate passed verification")
    check(counts[0x81 if mode == "missing-wheels" else 0x82] > 0, "no real samples filtered")
    print(f"withheld {counts[0x81]} wheels and {counts[0x82]} IMUs at X5; no confirmation sent", flush=True)


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
        if mode in ("interrupt", "interrupt-and-exit"):
            asyncio.run(interrupted(port, data))
            if mode == "interrupt-and-exit":
                return
        elif mode == "reject":
            asyncio.run(rejected(port, data, identity))
        else:
            asyncio.run(stage(port, data, identity, baseline_known=True))
        start = time.monotonic()
        if mode in ("missing-wheels", "missing-imu"):
            check(had_imu, "fixture requires a working IMU on the baseline")
            verify_missing_stream(port, identity, mode)
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
    parser.add_argument("mode", choices=["interrupt", "interrupt-and-exit", "reject", "no-confirm",
                                         "early-hang", "missing-wheels", "missing-imu"])
    parser.add_argument("package", type=Path)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    args = parser.parse_args()
    run(args.mode, args.port, args.package)
