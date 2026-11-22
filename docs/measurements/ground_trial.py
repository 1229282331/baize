"""Run one bounded baize-monitor jog, show local ruler cues, and retain the stop and coast."""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess
import sys

from ground_geometry import wheel_travel


def run(args):
    monitor = str(args.monitor.resolve(strict=True))
    if not 1 <= args.ms <= 2000 or any(not math.isfinite(v) or abs(v) > 2 for v in args.speeds):
        raise ValueError("duration must be 1..2000 ms and each speed within +/-2 rad/s")
    args.directory.mkdir(parents=True, exist_ok=False)
    run_csv = args.directory/"wheels.csv"
    settled = args.directory/"settled.csv"
    print("车辆将限时点动。按 Enter 起步，按 Ctrl+C 取消。", flush=True)
    input()
    command = [monitor, args.port, "--jog", str(args.ms), *map(str, args.speeds),
               "--imu-csv", str(args.directory/"imu.csv")]
    enabled_ms = cue_tick = stop_tick = None
    with (args.directory/"monitor.log").open("w") as log, run_csv.open("w", newline="") as output:
        child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=log, text=True)
        try:
            rows = csv.DictReader(child.stdout)
            if not rows.fieldnames or "drive_state" not in rows.fieldnames:
                raise ValueError("monitor did not return a wheel CSV header")
            writer = csv.DictWriter(output, fieldnames=rows.fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
                state = row["drive_state"]
                host_ms, tick = int(row["host_elapsed_ms"]), int(row["board_ms"])
                if state == "1" and enabled_ms is None:
                    enabled_ms = host_ms
                    print("已使能，开始点动。", flush=True)
                if state == "1" and cue_tick is None and host_ms-enabled_ms >= max(0, args.ms-600):
                    cue_tick = tick
                    print("准备停车。做停距试验时，看车身基准点已越过哪条刻线。", flush=True)
                if state == "0" and enabled_ms is not None and stop_tick is None:
                    stop_tick = tick
            if child.wait() or stop_tick is None:
                raise ValueError("motion or stop handshake failed; see monitor.log")
        finally:
            child.stdout.close()
            if child.poll() is None:
                child.terminate()
                child.wait()
        print("收到停机反馈，正在记录车轮余动。", flush=True)
        with settled.open("wb") as rest:
            subprocess.run([monitor, args.port, "30"], stdout=rest, stderr=log, check=True, timeout=5)
    left, right = wheel_travel(run_csv, settled)
    record = {"duration_ms": args.ms, "target_rad_s": args.speeds, "ruler_cue_board_ms": cue_tick,
              "disabled_feedback_board_ms": stop_tick, "left_rad": left, "right_rad": right}
    (args.directory/"trial.json").write_text(json.dumps(record, indent=2), encoding="utf-8")
    print("编码器已停稳。现在可量地面标记和车身基准点，报出尺数。", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("monitor", type=Path)
    parser.add_argument("port")
    parser.add_argument("directory", type=Path, help="new output directory for this run")
    parser.add_argument("ms", type=int)
    parser.add_argument("speeds", nargs=4, type=float, metavar="RAD/S")
    try:
        run(parser.parse_args())
    except (EOFError, OSError, ValueError, subprocess.SubprocessError) as error:
        sys.exit(f"Ground trial: {error}")
