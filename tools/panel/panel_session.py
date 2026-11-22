"""Local panel I/O, full-rate acquisition and per-jog measurements."""
import csv
import ctypes as C
import json
import math
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from queue import Empty, SimpleQueue
from statistics import mean
from typing import NamedTuple

WHEELS = ("fl", "fr", "rl", "rr")
WHEEL_NAMES = ("左前", "右前", "左后", "右后")
PRESETS = {"前进": (1, 1, 1, 1), "后退": (-1, -1, -1, -1),
           "左转": (-1, 1, -1, 1), "右转": (1, -1, 1, -1)}
READY, BUSY, FINISHED, INTERRUPTED = 1, 2, 3, 4
WHEEL_HEADER = ("host_elapsed_ms", "board_ms", "drive_state", "stop_reason", "supply_v",
                *(w+"_rad" for w in WHEELS), *(w+"_rad_s" for w in WHEELS), "trial")
IMU_HEADER = ("host_elapsed_ms", "board_ms", "ax_m_s2", "ay_m_s2", "az_m_s2",
              "gx_rad_s", "gy_rad_s", "gz_rad_s", "trial")
POSES = {"车顶朝上": "z-up", "车底朝上": "z-down", "车头朝上": "x-up", "车尾朝上": "x-down",
         "左侧朝上": "y-up", "右侧朝上": "y-down", "重新放平": "check-z-up"}


class Status(C.Structure):
    _fields_ = [("ms", C.c_uint32), ("valid", C.c_bool), ("state", C.c_uint8), ("reason", C.c_uint8),
                ("volts", C.c_double), ("angle", C.c_double*4), ("speed", C.c_double*4)]


class Imu(C.Structure):
    _fields_ = [("ms", C.c_uint32), ("valid", C.c_bool), ("accel", C.c_double*3), ("gyro", C.c_double*3)]


class Payload(C.Union):
    _fields_ = [("status", Status), ("imu", Imu)]


class Frame(C.Structure):
    _fields_ = [("kind", C.c_uint8), ("data", Payload)]


def library(path=None):
    local = Path(__file__).resolve().parent
    name = "baize_panel_io" + (".dll" if os.name == "nt" else ".dylib" if sys.platform == "darwin" else ".so")
    build = local.parents[1]/"build"
    places = [local, build/"native/tools", build/"native-linux/tools", build/"tools", build/"host", build/"desktop/host",
              build/"linux-host", build/"linux-tests/host"]
    paths = [Path(path)] if path else [d/sub/name for d in places for sub in ("", "Release", "Debug")]
    found = next((p for p in paths if p.is_file()), None)
    if found is None:
        raise OSError("请从仓库根目录构建 baize_panel_io 目标，或指定本机构建的库路径。")
    lib = C.CDLL(str(found.resolve()))
    link = C.c_void_p
    for name, result, args in (
        ("panel_frame_size", C.c_size_t, []), ("panel_open", link, [C.c_char_p]),
        ("panel_start", C.c_int, [link, C.c_uint32, C.POINTER(C.c_double)]),
        ("panel_stop", None, [link]), ("panel_close", None, [link]),
        ("panel_poll", C.c_int, [link, C.POINTER(Frame), C.POINTER(C.c_int)])):
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = result, args
    if lib.panel_frame_size() != C.sizeof(Frame):
        raise OSError("本机串口库和面板的数据布局不一致，请重新构建。")
    return lib


def ports():
    """Put present CH340 ports first. An editable port field also accepts other names."""
    if os.name != "nt":
        return sorted(str(p) for pattern in ("ttyUSB*", "ttyACM*") for p in Path("/dev").glob(pattern))
    import winreg
    present, ch340 = set(), set()
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            for i in range(winreg.QueryInfoKey(key)[1]):
                driver, port, _ = winreg.EnumValue(key, i)
                present.add(port)
                if "ch34" in driver.lower():
                    ch340.add(port)
    except FileNotFoundError:
        pass
    try:
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"SYSTEM\CurrentControlSet\Enum\USB\VID_1A86&PID_7523") as usb:
            for i in range(winreg.QueryInfoKey(usb)[0]):
                try:
                    with winreg.OpenKey(usb, winreg.EnumKey(usb, i) + r"\Device Parameters") as device:
                        ch340.add(winreg.QueryValueEx(device, "PortName")[0])
                except OSError:
                    continue
    except OSError:
        pass
    return sorted(present, key=lambda p: (p not in ch340, int(p[3:]) if p[3:].isdigit() else 9999, p))


def records_root():
    base = Path(os.environ.get("LOCALAPPDATA", Path.home())) if os.name == "nt" else Path(
        os.environ.get("XDG_STATE_HOME", Path.home()/".local/state"))
    return base/"Baize"/"Panel"


def save_json(path, data):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


class Wheel(NamedTuple):
    host: int
    board: int
    state: int
    reason: int
    volts: float | None
    angle: tuple
    speed: tuple


def target_values(mode, wheel, direction, motion, speed, custom, seconds):
    ms = float(seconds)*1000
    if not math.isfinite(ms) or not 1 <= ms <= 2000:
        raise ValueError("点动时长须在 0.001～2.000 s 内。")
    if mode == "四轮自定":
        values, label = tuple(map(float, custom)), mode
    else:
        v = float(speed)
        if not 0 < v <= 2:
            raise ValueError("轮速幅值须大于零，且不超过 2 rad/s。")
        if mode == "单轮":
            sign = 1 if direction == "正向" else -1
            values = tuple(sign*v if name == wheel else 0 for name in WHEEL_NAMES)
            label = wheel+" · "+direction
        else:
            values = tuple(v*s for s in PRESETS[motion])
            label = motion
    if (len(values) != 4 or not all(math.isfinite(v) and abs(v) <= 2 for v in values)
            or not any(round(v*1000) for v in values)):
        raise ValueError("四轮目标须在 ±2 rad/s 内，至少一轮须有可发送的非零目标。")
    return label, round(ms), values


def observation_result(targets, wheel, direction):
    moving = [i for i, target in enumerate(targets) if target]
    if len(moving) != 1:
        return "已记录"
    index = moving[0]
    expected = "朝车头" if targets[index] > 0 else "朝车尾"
    if (wheel in (*WHEEL_NAMES, "多轮") and wheel != WHEEL_NAMES[index]
            or direction in ("朝车头", "朝车尾") and direction != expected):
        return "不符"
    return "一致" if (wheel, direction) == (WHEEL_NAMES[index], expected) else "待核对"


def summarize(rows, targets, outcome, settled):
    """Use the final 200 ms of one uninterrupted enabled interval for tracking."""
    enabled = [i for i, row in enumerate(rows) if row.state == 1]
    active = [rows[i] for i in enabled]
    continuous = bool(active) and len(active) == enabled[-1]-enabled[0]+1 and all(
        0 < (b.board-a.board) % 2**32 < 250 and 0 <= b.host-a.host < 250 for a, b in zip(rows, rows[1:]))
    tail = [r for r in active if (active[-1].board-r.board) % 2**32 <= 200]
    enough = len(tail) >= 9 and (tail[-1].board-tail[0].board) % 2**32 >= 160
    measured = [round(mean(r.speed[i] for r in tail), 4) for i in range(4)] if enough else None
    delta = [round(b-a, 4) for a, b in zip(rows[0].angle, rows[-1].angle)] if rows and continuous else None
    matches = None
    if measured and delta and continuous:
        matches = [t*v > 0 and t*delta[i] > 0 and abs(v-t) <= max(.05, .2*abs(t)) if t else
                   max(abs(r.angle[i]-rows[0].angle[i]) for r in rows) <= .02
                   for i, (v, t) in enumerate(zip(measured, targets))]
    stop_ack = bool(active) and any(r.state == 0 and r.reason == 2 for r in rows[enabled[-1]+1:])
    if outcome == "completed" and stop_ack and settled and matches and all(matches):
        assessment = "反馈符合"
    elif outcome != "completed":
        assessment = {"stopped": "手动停机", "disconnected": "连接中断", "interrupted": "点动中断"}.get(outcome, "点动中断")
    elif not enough:
        assessment = "样本不足"
    else:
        assessment = "需复核"
    return dict(outcome=outcome, assessment=assessment, wheel_order=list(WHEELS), stop_ack=stop_ack,
                settled=settled, continuous=continuous, tail_samples=len(tail), mean_rad_s=measured,
                delta_rad=delta, wheels_match=matches, end_host_ms=rows[-1].host if rows else None)


class Session(threading.Thread):
    def __init__(self, lib, port, directory):
        super().__init__(name="Baize serial")
        self.lib, self.port, self.folder = lib, port, directory
        self.requests, self.events = SimpleQueue(), SimpleQueue()
        self.origin = time.monotonic()
        self.wheel = self.imu = None
        self.history = deque(maxlen=300)
        self.lock = threading.Lock()
        self.trial = self.pose = None
        self.rows = []
        self.quiet = None
        self.ending = None
        self.serial_state = 0
        self.sequence = 0

    def elapsed(self):
        return int((time.monotonic()-self.origin)*1000)

    def finish(self, outcome, settled=False):
        if self.trial is None:
            return
        self.trial.update(summarize(self.rows, self.trial["target_rad_s"], outcome, settled))
        path = self.folder/(self.trial["id"]+".json")
        save_json(path, self.trial)
        self.events.put(("trial", path))
        self.trial, self.ending, self.quiet = None, None, None
        self.rows = []

    def wheel_frame(self, raw, host, writer):
        volts = raw.volts if math.isfinite(raw.volts) else None
        row = Wheel(host, raw.ms, raw.state, raw.reason, volts, tuple(raw.angle), tuple(raw.speed))
        key = self.trial["id"] if self.trial else ""
        writer.writerow([host, raw.ms, raw.state, raw.reason, volts, *row.angle, *row.speed, key])
        with self.lock:
            self.wheel = row
            self.history.append(row)
        if self.trial is not None:
            self.rows.append(row)
            prev = self.rows[-2] if len(self.rows) > 1 else None
            quiet = (row.state == 0 and row.reason == 2 and not any(row.speed))
            if not quiet:
                self.quiet = None
            elif (self.quiet is None or not prev or prev.angle != row.angle
                  or not 0 < (row.board-prev.board) % 2**32 <= 40):
                self.quiet = row.board

    def start_trial(self, link, item):
        _, label, duration, targets = item
        if self.trial or self.pose or not self.wheel or not 1 <= duration <= 2000 or len(targets) != 4:
            self.events.put(("notice", "当前未就绪，请等停机握手和采集结束。"))
            return
        if not all(math.isfinite(v) and abs(v) <= 2 for v in targets):
            self.events.put(("notice", "每轮目标须在 ±2 rad/s 内。"))
            return
        targets = [round(v*1000)/1000 for v in targets]
        accepted = self.lib.panel_start(link, duration, (C.c_double*4)(*targets))
        if accepted < 0:
            raise OSError("下发使能时串口写入失败。")
        if not accepted:
            self.events.put(("notice", "点动未启动，请检查新鲜轮状态和停机回执。"))
            return
        self.sequence += 1
        self.trial = dict(id=f"trial-{self.sequence:03d}", label=label, port=self.port,
                          time=datetime.now().astimezone().isoformat(timespec="seconds"),
                          start_host_ms=self.elapsed(), start_board_ms=self.wheel.board, duration_ms=duration,
                          target_rad_s=list(targets), outcome="running")
        self.rows, self.quiet = [self.wheel], None
        save_json(self.folder/(self.trial["id"]+".json"), self.trial)
        self.events.put(("notice", f"{label} · 正在点动"))

    def start_pose(self, name):
        if (name not in POSES or self.trial or self.pose or self.serial_state != READY or not self.imu
                or self.elapsed()-self.imu[0] >= 250 or not self.wheel or any(self.wheel.speed)):
            self.events.put(("notice", "请在轮子停稳、IMU 有新样本时采集。"))
            return
        self.pose = (name, self.elapsed(), [])
        self.events.put(("notice", f"正在采集「{name}」，请保持静止。"))

    def collect_pose(self, host):
        if not self.pose:
            return
        name, start, rows = self.pose
        if (self.serial_state != READY or not self.wheel or any(self.wheel.speed) or host-start >= 4500
                or host-self.imu[0] >= 250):
            self.pose = None
            self.events.put(("notice", "姿态采集已中断，可在停稳后重试。"))
        elif len(rows) >= 125:
            self.pose = None
            if any(not 0 < (b[1]-a[1]) % 2**32 < 250 or b[0]-a[0] >= 250 for a, b in zip(rows, rows[1:])):
                self.events.put(("notice", "六轴样本出现间断，请重新采集此姿态。"))
                return
            folder = self.folder/"poses"
            folder.mkdir(exist_ok=True)
            stored = folder/f"{POSES[name]}-{start}.csv"
            with stored.open("x", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(IMU_HEADER)
                writer.writerows(rows)
            latest = folder/(POSES[name]+".csv")
            temporary = latest.with_suffix(".tmp")
            temporary.write_bytes(stored.read_bytes())
            temporary.replace(latest)
            self.events.put(("notice", f"已保存「{name}」的 {len(rows)} 帧六轴样本。"))

    def run(self):
        link = None
        failure = None
        try:
            self.folder.mkdir(parents=True, exist_ok=False)
            link = self.lib.panel_open(os.fsencode(self.port))
            if not link:
                raise OSError(f"无法打开 {self.port}，请检查接线和端口占用。")
            save_json(self.folder/"session.json", dict(port=self.port, baudrate=115200, protocol_version=1,
                      started_at=datetime.now().astimezone().isoformat(timespec="seconds"), wheel_order=list(WHEELS)))
            with (self.folder/"wheels.csv").open("x", newline="", encoding="utf-8") as w, (
                    self.folder/"imu.csv").open("x", newline="", encoding="utf-8") as a:
                wheels, imu = csv.writer(w), csv.writer(a)
                wheels.writerow(WHEEL_HEADER)
                imu.writerow(IMU_HEADER)
                self.exchange(link, wheels, imu, w, a)
        except (OSError, ValueError, OverflowError) as error:
            failure = str(error)
        finally:
            if link:
                self.lib.panel_close(link)
            try:
                self.finish("disconnected")
            except OSError as error:
                failure = str(error)
            self.events.put(("closed", failure))

    def exchange(self, link, wheels, imu, wheel_file, imu_file):
        frames, state = (Frame*8)(), C.c_int()
        close_by = None
        last_flush, stopped = 0, False
        while True:
            items = []
            try:
                while True:
                    items.append(self.requests.get_nowait())
            except Empty:
                pass
            actions = {item[0] for item in items}
            if actions & {"close", "stop"}:
                self.lib.panel_stop(link)
                stopped = self.trial is not None
                self.pose = None
                if "close" in actions:
                    close_by = self.elapsed()+900
            elif close_by is None:
                for item in items:
                    if item[0] == "jog":
                        self.start_trial(link, item)
                    elif item[0] == "pose":
                        self.start_pose(item[1])
            count = self.lib.panel_poll(link, frames, C.byref(state))
            if count < 0:
                raise OSError("连续 1 s 没有新轮反馈，连接已关闭。" if count == -2 else "串口读写中断，连接已关闭。")
            host = self.elapsed()
            for i in range(count):
                frame = frames[i]
                if frame.kind == 0x81:
                    self.wheel_frame(frame.data.status, host, wheels)
                else:
                    raw = frame.data.imu
                    self.imu = (host, raw.ms, tuple(raw.accel), tuple(raw.gyro))
                    row = (host, raw.ms, *self.imu[2], *self.imu[3], self.trial["id"] if self.trial else "")
                    imu.writerow(row)
                    if self.pose:
                        self.pose[2].append(row)
            if state.value == READY and self.serial_state != READY and not self.trial and close_by is None:
                self.events.put(("notice", f"{self.port} 已收到停机回执，可以开始点动。"))
            self.serial_state = state.value
            self.collect_pose(host)
            if self.trial and state.value in (FINISHED, INTERRUPTED):
                self.ending = ("stopped" if stopped else "completed" if state.value == FINISHED else "interrupted", host)
            if self.ending:
                outcome, end = self.ending
                settled = self.quiet is not None and (self.wheel.board-self.quiet) % 2**32 >= 100
                if settled or host-end >= 1000:
                    self.finish(outcome, settled)
                    stopped = False
            if host-last_flush >= 500:
                wheel_file.flush()
                imu_file.flush()
                last_flush = host
            if close_by is not None and (host >= close_by or (state.value == READY and not self.trial)):
                return
