"""Updater release regressions, run with the X5 updater's Python dependencies."""
from contextlib import asynccontextmanager, nullcontext
from pathlib import Path
import asyncio
import importlib.util
import struct
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/update"))
if "--skip-missing-deps" in sys.argv:
    sys.argv.remove("--skip-missing-deps")
    if any(importlib.util.find_spec(name) is None for name in ("cobs", "serial", "smpclient")):
        print("Install tools/update dependencies to run the updater health tests.")
        sys.exit(77)
from baize_update import application, cli


def wheel(tick):
    return 0x81, struct.pack("<IBBH4q4i", tick & 0xffffffff, 0, 2, 12000, *([0] * 8))


def imu(tick):
    return 0x82, struct.pack("<I6i", tick & 0xffffffff, *([0] * 6))


class HealthTest(unittest.TestCase):
    def observe(self, events, required=True):
        app = application.Application.__new__(application.Application)
        app.serial = MagicMock()
        now = 0

        def packets(deadline):
            nonlocal now
            for ms, packet in sorted(events, key=lambda e: e[0]):
                now = ms / 1000
                if now >= deadline:
                    break
                yield packet
            now = deadline

        app.packets = packets
        with patch.object(application.time, "monotonic", side_effect=lambda: now):
            return app.parked(require_imu=required)

    def test_live_streams_across_wrap(self):
        self.assertTrue(self.observe([(t, frame(0xffffff00 + t))
                                     for t in range(0, 600, 20) for frame in (imu, wheel)]))

    def test_late_imu_cannot_confirm_after_wheels_stop(self):
        events = [(t, wheel(t)) for t in range(0, 340, 20)]
        with self.assertRaises(RuntimeError):
            self.observe(events + [(880, imu(880)), (900, imu(900))])

    def test_stale_or_reordered_imu_cannot_confirm(self):
        wheels = [(t, wheel(t)) for t in range(0, 1500, 20)]
        for samples in ([(0, imu(0)), (20, imu(20))],
                        [(t, imu(40 if t % 40 else 20)) for t in range(0, 1500, 20)]):
            with self.subTest(samples=samples[:2]), self.assertRaises(RuntimeError):
                self.observe(wheels + samples)

    def test_optional_sensor_discovery(self):
        wheels = [(t, wheel(t)) for t in range(0, 1500, 20)]
        self.assertTrue(self.observe(wheels + [(800, imu(800)), (820, imu(820))], required=False))
        self.assertFalse(self.observe(wheels, required=False))
        with self.assertRaises(RuntimeError):
            self.observe(wheels, required=True)

    def test_queued_wheels_do_not_prove_stationary_interval(self):
        with self.assertRaises(RuntimeError):
            self.observe([(600, wheel(t)) for t in range(0, 400, 20)] +
                         [(580, imu(580)), (590, imu(590))])

    def test_retry_observes_the_restored_primary_before_install(self):
        app = MagicMock()
        app.info.side_effect = [TimeoutError(), SimpleNamespace(identity="old", trial=False)]
        app.packets.return_value = iter([wheel(20)])
        app.parked.return_value = True
        with (patch.object(cli, "locked", return_value=nullcontext()),
              patch.object(cli, "Application", return_value=app),
              patch.object(cli, "stage", return_value=True) as upload,
              patch.object(cli, "verify_trial") as verify,
              patch.object(cli.time, "sleep")):
            cli.install("port", b"image", "new")
        app.request.assert_called_once_with(application.ENTER)
        upload.assert_awaited_once()
        verify.assert_called_once_with("port", "new", True, True)

    def test_unknown_baseline_with_primary_blocks_upload(self):
        client = MagicMock()

        @asynccontextmanager
        async def slots(port):
            yield client, [SimpleNamespace(slot=0, bootable=True)]

        with patch.object(cli, "recovery", slots), self.assertRaisesRegex(RuntimeError, "primary"):
            asyncio.run(cli.stage("port", b"image", "new", baseline_known=False))
        client.upload.assert_not_called()

    def test_recovery_without_primary_still_checks_candidate(self):
        app = MagicMock()
        app.info.side_effect = TimeoutError()
        with (patch.object(cli, "locked", return_value=nullcontext()),
              patch.object(cli, "Application", return_value=app),
              patch.object(cli, "stage", return_value=False) as upload,
              patch.object(cli, "verify_trial") as verify,
              patch.object(cli.time, "sleep")):
            cli.install("port", b"image", "new")
        upload.assert_awaited_once_with("port", b"image", "new", baseline_known=False)
        verify.assert_called_once_with("port", "new", False, False)


if __name__ == "__main__":
    unittest.main()
