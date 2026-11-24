import argparse
import asyncio
from contextlib import asynccontextmanager, closing, contextmanager
import fcntl
import logging
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time

from smpclient import SMPClient, success
from smpclient.exceptions import SMPClientException
from smpclient.generics import error_v1
from smpclient.requests.image_management import ImageStatesRead, ImageStatesWrite
from smpclient.requests.os_management import ResetWrite
from smpclient.transport import SMPTransportDisconnected
from smpclient.transport.serial import BufferSize, SMPSerialTransport

from .application import Application, CONFIRM, ENTER, INFO
from .image import inspect_image, MAX_IMAGE


@contextmanager
def locked(port):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"{port} is in use by another controller") from exc
        yield
    finally:
        os.close(fd)


def read_package(path):
    with path.open("rb") as file:
        data = file.read(MAX_IMAGE + 1)
    return data, inspect_image(data)


@asynccontextmanager
async def recovery(port):
    transport = SMPSerialTransport(BufferSize(256))
    # MCUboot validates flash before dispatching buffered SMP requests on this Cortex-M3.
    client = SMPClient(transport, port, timeout_s=2)
    try:
        await client.connect()
        deadline = time.monotonic() + 3
        while True:
            try:
                state = await client.request(ImageStatesRead(), timeout_s=6)
                if success(state):
                    break
            except TimeoutError:
                pass
            if time.monotonic() >= deadline:
                raise RuntimeError("MCUboot did not answer on USART2")
        yield client, state.images
    finally:
        await client.disconnect()


async def stage(port, data, identity):
    async with recovery(port) as (client, images):
        had_primary = any(image.slot == 0 and image.bootable for image in images)
        async for offset in client.upload(data, slot=2, first_timeout_s=4, subsequent_timeout_s=3):
            if offset == len(data):
                print(f"staged {offset} bytes", flush=True)
        marked = await client.request(ImageStatesWrite(hash=identity.digest, confirm=False), timeout_s=8)
        if not success(marked):
            raise RuntimeError(f"bootloader rejected the candidate: {marked}")
        if not any(image.slot == 1 and image.hash == identity.digest and image.pending and
                   not image.permanent for image in marked.images):
            raise RuntimeError("bootloader did not mark the requested candidate for trial")
        reset = await client.request(ResetWrite(), timeout_s=2)
        # MCUboot serial recovery encodes a successful reset as the legacy {"rc": 0}.
        if not (success(reset) or (error_v1(reset) and reset.rc == 0)):
            raise RuntimeError(f"bootloader rejected reset: {reset}")
        return had_primary


def verify_trial(port, identity, require_imu, had_primary=True):
    # Flash exchange precedes application startup. The MCU owns its ten-second trial deadline.
    until = time.monotonic() + 25
    with closing(Application(port)) as app:
        while time.monotonic() < until:
            try:
                running = app.request(INFO, timeout=0.22)
                break
            except TimeoutError:
                pass
        else:
            raise RuntimeError("candidate did not start; the board will recover autonomously")
        if running.identity != identity or (had_primary and not running.trial):
            raise RuntimeError(f"unexpected running image: {running}")
        imu = app.parked(require_imu=require_imu, check_startup=True)
        print(f"stopped wheel stream verified; IMU {'present' if imu else 'unavailable'}", flush=True)
        if running.trial:
            result = app.request(CONFIRM, identity.digest)
            if result.trial or result.identity != identity:
                raise RuntimeError("confirmation not acknowledged")
        current = app.request(INFO)
        if current.trial or current.identity != identity:
            raise RuntimeError("could not verify the running confirmed image")
        print(f"confirmed {current.identity}", flush=True)


def install(port, data, identity):
    with locked(port):
        had_imu = False
        with closing(Application(port)) as app:
            try:
                running = app.info()
            except TimeoutError:
                running = None   # Already in serial recovery is a valid starting state.
            if running:
                had_imu = app.parked()
                if running.identity == identity:
                    if running.trial:
                        raise RuntimeError("trial already in progress; wait for its result before retrying")
                    print(f"already installed {identity}", flush=True)
                    return
                if running.trial:
                    raise RuntimeError("wait for the existing trial to finish")
                print(f"updating {running.identity}", flush=True)
                app.request(ENTER)
        time.sleep(0.08)
        had_primary = asyncio.run(stage(port, data, identity))
        verify_trial(port, identity, had_imu, running is not None or had_primary)


def submit(port, path):
    data, identity = read_package(path)
    # Snapshot validated bytes. The worker unlinks the snapshot as soon as it reads it.
    with tempfile.NamedTemporaryFile(prefix="baize-update-", dir="/run", delete=False) as file:
        file.write(data)
        staged = Path(file.name)
    unit = staged.name
    print(f"installing {identity}\njournalctl -u {unit}", flush=True)
    try:
        result = subprocess.run(["systemd-run", "--wait", "--collect", f"--unit={unit}",
                                 "--property=RuntimeMaxSec=120", sys.executable, "-m",
                                 "baize_update.cli", "_install", str(staged), "--port", port])
        subprocess.run(["journalctl", "--no-pager", "-o", "cat", "-u", unit, "-n", "8"])
        return result.returncode
    finally:
        staged.unlink(missing_ok=True)


def main():
    # MCUboot omits the optional OS-parameter query. The transport uses its configured MTU.
    logging.getLogger("smpclient").setLevel(logging.CRITICAL)
    parser = argparse.ArgumentParser(prog="baize-update")
    sub = parser.add_subparsers(dest="action", required=True, metavar="{info,install}")
    for name in ("info", "install", "_install"):
        command = sub.add_parser(name)
        command.add_argument("--port", default="/dev/ttyUSB0")
        if name != "info":
            command.add_argument("package", type=Path)
    args = parser.parse_args()
    try:
        if args.action == "info":
            with locked(args.port), closing(Application(args.port)) as app:
                state = app.info()
                print(f"{'trial' if state.trial else 'confirmed'} {state.identity}")
        elif args.action == "install":
            return submit(args.port, args.package)
        else:
            data, identity = read_package(args.package)
            # Only remove the root-owned snapshot created by submit().
            if args.package.parent == Path("/run") and args.package.name.startswith("baize-update-"):
                args.package.unlink()
            install(args.port, data, identity)
    except TimeoutError:
        print(f"{args.port}: no timely reply; check the running image with baize-update info",
              file=sys.stderr)
        return 1
    except (OSError, ValueError, RuntimeError, SMPClientException, SMPTransportDisconnected) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
