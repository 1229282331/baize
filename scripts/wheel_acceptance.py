"""Replay a panel recording, or exercise a raised chassis through the production host library."""
import argparse
import csv
import json
import sys
import time
from pathlib import Path
from queue import Empty

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools/panel"))
from panel_session import READY, WHEELS, Session, Wheel, library, save_json, summarize


def stream(directory, name, visit=None):
    frames = max_host_gap = max_board_gap = resets = 0
    previous = None
    with (directory/(name+".csv")).open(encoding="utf-8", newline="") as data:
        for row in csv.DictReader(data):
            host, board = int(row["host_elapsed_ms"]), int(row["board_ms"])
            if previous:
                host_gap = host-previous[0]
                if host_gap < 0:
                    raise ValueError(f"{name}: host clock went backwards")
                max_host_gap = max(max_host_gap, host_gap)
                gap = (board-previous[1]) % 2**32
                if gap > 2**31:
                    resets += 1
                else:
                    max_board_gap = max(max_board_gap, gap)
            if visit:
                visit(row, host, board)
            previous = host, board
            frames += 1
    return dict(frames=frames, max_host_gap_ms=max_host_gap, max_board_gap_ms=max_board_gap, clock_resets=resets)


def replay(directory):
    groups, previous = {}, None
    def wheel(row, host, board):
        nonlocal previous
        sample = Wheel(host, board, int(row["drive_state"]), int(row["stop_reason"]),
                       float(row["supply_v"]) if row["supply_v"] else None,
                       tuple(float(row[w+"_rad"]) for w in WHEELS), tuple(float(row[w+"_rad_s"]) for w in WHEELS))
        key = row["trial"]
        if key:
            if key not in groups:
                if previous is None:
                    raise ValueError(f"{key}: missing baseline wheel frame")
                groups[key] = [previous]
            groups[key].append(sample)
        previous = sample

    result = dict(wheels=stream(directory, "wheels", wheel), imu=stream(directory, "imu"), trials=[])
    for path in sorted(directory.glob("trial-*.json")):
        trial = json.loads(path.read_text(encoding="utf-8"))
        rows = groups.pop(path.stem, [])
        if not rows or trial["start_board_ms"] != rows[0].board or trial["outcome"] == "running":
            raise ValueError(f"{path.name}: incomplete trial or missing baseline")
        last = rows[-1]
        quiet_ms = 0
        for a, b in zip(reversed(rows[:-1]), reversed(rows[1:])):
            gap = (b.board-a.board) % 2**32
            if (not 0 < gap <= 40 or any(r.state != 0 or r.reason != 2 or any(r.speed) for r in (a, b))
                    or a.angle != last.angle or b.angle != last.angle):
                break
            quiet_ms += gap
        computed = summarize(rows, trial["target_rad_s"], trial["outcome"], quiet_ms >= 100)
        for field in ("mean_rad_s", "delta_rad", "stop_ack", "wheels_match", "end_host_ms", "continuous", "tail_samples"):
            if computed[field] != trial[field]:
                raise ValueError(f"{path.name}: {field} differs from the raw wheel samples")
        if trial["outcome"] in ("completed", "stopped") and (
                computed["assessment"] != trial["assessment"] or computed["settled"] != trial["settled"]):
            raise ValueError(f"{path.name}: inconsistent stopping result")
        result["trials"].append(dict(id=path.stem, label=trial["label"], target_rad_s=trial["target_rad_s"],
                                     quiet_ms=quiet_ms, **computed))
    if groups:
        raise ValueError(f"missing trial files: {', '.join(groups)}")
    return result


def wait_ready(session):
    until = time.monotonic()+2
    while time.monotonic() < until:
        row = session.wheel
        if (session.serial_state == READY and row and row.state == 0 and row.reason == 2
                and not any(row.speed) and session.imu
                and session.elapsed()-row.host < 250 and session.elapsed()-session.imu[0] < 250):
            return
        if not session.is_alive():
            break
        time.sleep(.01)
    raise RuntimeError(f"{session.port}: no fresh idle wheel acknowledgement and IMU")


def jog(session, label, targets, manual=False):
    session.requests.put(("jog", label, 1600 if manual else 1000, targets))
    started, cancel_at = time.monotonic(), None
    stop_sent = False
    while time.monotonic()-started < 4:
        if manual and cancel_at is None and session.wheel.state == 1 and any(session.wheel.speed):
            cancel_at = time.monotonic()+.30
        if not stop_sent and cancel_at is not None and time.monotonic() >= cancel_at:
            session.requests.put(("stop",))
            stop_sent = True
        try:
            kind, value = session.events.get(timeout=.015)
        except Empty:
            continue
        if kind == "closed":
            raise RuntimeError(value or "serial port closed during jog")
        if kind == "trial":
            record = json.loads(value.read_text(encoding="utf-8"))
            expected = "手动停机" if manual else "反馈符合"
            if (record["assessment"] != expected or manual != stop_sent or
                    not record["settled"] or not record["stop_ack"]):
                raise ValueError(f"{value.name}: {record['assessment']}, expected {expected}")
            wait_ready(session)
            return
    raise RuntimeError(f"{label}: no completed jog")


def run(port, directory, native):
    lib = library(native)
    directory.mkdir(parents=True, exist_ok=False)
    report = {}
    for opening in ("motion", "reconnected"):
        session = Session(lib, port, directory/opening)
        session.start()
        try:
            wait_ready(session)
            if opening == "motion":
                for i, wheel in enumerate(WHEELS):
                    for sign in (1, -1):
                        jog(session, f"{wheel} {sign:+}", [sign if j == i else 0 for j in range(4)])
                for speed in (.35, -.35, 2., -2.):
                    jog(session, f"four {speed:+}", [speed]*4)
                jog(session, "independent targets", [.45, -.85, 1.20, -1.55])
                jog(session, "manual stop", [.65, -.65, .65, -.65], manual=True)
            else:
                time.sleep(.35)
                wait_ready(session)
                with session.lock:
                    rows = list(session.history)
                if (len(rows) < 10 or (rows[-1].board-rows[0].board) % 2**32 < 200 or
                        any(r.state or any(r.speed) or r.angle != rows[0].angle for r in rows)):
                    raise ValueError("reconnect did not remain disabled and stationary")
        finally:
            session.requests.put(("close",))
            session.join(timeout=3)
            if session.is_alive():
                raise RuntimeError("serial session did not close")
        while not session.events.empty():
            kind, value = session.events.get_nowait()
            if kind == "closed" and value:
                raise RuntimeError(value)
        report[opening] = replay(session.folder)
    save_json(directory/"summary.json", report)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    replay_args = sub.add_parser("replay", help="recompute wheel results from an existing session; offline")
    replay_args.add_argument("directory", type=Path)
    run_args = sub.add_parser("run", help="move the raised wheels for bounded tests, then stop and reconnect")
    run_args.add_argument("port")
    run_args.add_argument("directory", type=Path, help="new directory for both connections and their raw records")
    run_args.add_argument("--library", type=Path)
    args = parser.parse_args()
    try:
        result = replay(args.directory) if args.action == "replay" else run(args.port, args.directory, args.library)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        parser.exit(1, f"{error}\n")
