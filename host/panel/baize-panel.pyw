"""Native USB control panel for the Baize chassis. Run with Python 3.10+ and Tk."""
import argparse
import json
import os
import subprocess
import sys
import tkinter as tk
from datetime import datetime
from pathlib import Path
from queue import Empty
from tkinter import font, messagebox, ttk

from panel_session import (POSES, PRESETS, READY, WHEEL_NAMES, Session, library, observation_result,
                           ports, records_root, save_json, target_values)

BG, PAPER, INK, MUTED = "#edf2f4", "#ffffff", "#182e38", "#526873"
NAVY, TEAL, PALE, LINE = "#172e38", "#006f68", "#d9f1eb", "#d7e1e4"
COLORS = ("#007c75", "#285cb7", "#b36500", "#90528d")
REASONS = ("运行", "上电 / 复位", "主动停机", "命令超时", "驱动故障")


class Panel:
    def __init__(self, root, args):
        self.root, self.args = root, args
        self.session = self.lib = None
        self.pending = self.disconnecting = self.exiting = False
        self.paths = {}
        self.last_folder = args.records
        self.latest = tk.StringVar(value="还没有点动记录")
        self.notice = tk.StringVar(value="请选择串口并连接。")
        self.mode, self.wheel = tk.StringVar(value="单轮"), tk.StringVar(value="左前")
        self.direction, self.motion = tk.StringVar(value="正向"), tk.StringVar(value="前进")
        self.speed, self.seconds = tk.StringVar(value="1.00"), tk.StringVar(value="1.60")
        self.custom = [tk.StringVar(value="0.00") for _ in range(4)]
        self.pose = tk.StringVar(value=next(iter(POSES)))
        self.port = tk.StringVar()
        self.status = tk.StringVar(value="未连接")
        self.state = tk.StringVar(value="—")
        self.volts = tk.StringVar(value="—")
        self.timing = tk.StringVar(value="—")
        self.imu_state = tk.StringVar(value="等待样本")
        available = set(font.families())
        self.face = next((f for f in ("Microsoft YaHei UI", "Noto Sans CJK SC", "Noto Sans SC", "Inter")
                          if f in available), "TkDefaultFont")
        self.mono = next((f for f in ("IBM Plex Mono", "Consolas", "DejaVu Sans Mono") if f in available), self.face)
        root.title("白泽底盘控制台")
        width, height = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{min(1100,width-24)}x{min(730,height-72)}")
        root.minsize(min(850,width-24), min(600,height-72))
        root.configure(bg=BG)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.bind("<Escape>", lambda e: self.stop())
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", font=(self.face, 10), foreground=INK)
        style.configure("TCombobox", fieldbackground=PAPER, padding=4)
        style.configure("TNotebook", background=BG, borderwidth=0)
        style.configure("TNotebook.Tab", padding=(20, 7), background=BG, foreground=MUTED)
        style.map("TNotebook.Tab", background=[("selected", PAPER)], foreground=[("selected", TEAL)])
        style.configure("Treeview", rowheight=28, background=PAPER, fieldbackground=PAPER)
        style.configure("Treeview.Heading", background=BG, foreground=MUTED)
        self.header()
        self.tabs = ttk.Notebook(root)
        self.tabs.pack(fill="both", expand=True, padx=15, pady=(10, 0))
        self.controls, self.records = tk.Frame(self.tabs, bg=BG), tk.Frame(self.tabs, bg=PAPER)
        self.tabs.add(self.controls, text="控制与反馈")
        self.tabs.add(self.records, text="试次记录")
        self.control_page()
        self.record_page()
        self.label(root, variable=self.notice, bg=BG, color=MUTED, anchor="w").pack(
            side="bottom", before=self.tabs, fill="x", padx=18, pady=6)
        choices = self.refresh_ports()
        self.port.set(args.port or (choices[0] if choices else ""))
        if args.records.is_dir():
            for path in sorted(args.records.glob("*/trial-*.json"))[-100:]:
                self.add_record(path, select=False)
            if self.log.get_children():
                self.log.selection_set(self.log.get_children()[0])
        self.tick()
        if args.port:
            root.after(60, self.connect)

    def label(self, parent, text="", *, variable=None, bg=PAPER, color=INK, size=10, bold=False,
              mono=False, anchor="center"):
        return tk.Label(parent, text=text, textvariable=variable, bg=bg, fg=color, anchor=anchor, justify="left",
                        font=(self.mono if mono else self.face, size, "bold" if bold else "normal"), bd=0)

    def button(self, parent, text, command, bg=BG, fg=INK):
        return tk.Button(parent, text=text, command=command, font=(self.face, 10, "bold"),
                         bg=bg, fg=fg, relief="flat", bd=0, highlightthickness=0, padx=12, pady=6,
                         activebackground=PALE, activeforeground=INK, cursor="hand2", disabledforeground="#71828a")

    def header(self):
        head = tk.Frame(self.root, bg=NAVY, padx=19, pady=12)
        head.pack(fill="x")
        self.label(head, "白泽底盘控制台", bg=NAVY, color="white", bold=True, size=18).pack(side="left")
        self.badge = self.label(head, variable=self.status, bg=NAVY, color="#99e5d5", bold=True)
        self.badge.pack(side="left", padx=14)
        self.stop_button = self.button(head, "■  停机   Esc", self.stop, "#ad382f", "white")
        self.stop_button.pack(side="right", padx=(12, 0))
        self.connection = self.button(head, "连接", self.connect, "#d6ede7", TEAL)
        self.connection.pack(side="right", padx=5)
        self.port_box = ttk.Combobox(head, width=12, textvariable=self.port, postcommand=self.refresh_ports)
        self.port_box.pack(side="right")
        self.label(head, "串口", bg=NAVY, color="#dae5e8").pack(side="right", padx=7)

    def refresh_ports(self):
        choices = ports()
        self.port_box.configure(values=choices)
        return choices

    def card(self, parent, title):
        frame = tk.Frame(parent, bg=PAPER, padx=14, pady=9, highlightbackground=LINE, highlightthickness=1)
        self.label(frame, title, bold=True, size=13, anchor="w").pack(fill="x", pady=(0, 6))
        return frame

    def choices(self, parent, names, variable, columns, changed=None):
        frame = tk.Frame(parent, bg=PAPER)
        frame.pack(fill="x", pady=(0, 6))
        for i in range(columns):
            frame.columnconfigure(i, weight=1, uniform="choice")
        for i, name in enumerate(names):
            tk.Radiobutton(frame, text=name, value=name, variable=variable, indicatoron=False,
                           command=changed, selectcolor=PALE, bg=BG, fg=INK, activebackground=PALE,
                           activeforeground=TEAL, relief="flat", offrelief="flat", bd=1, highlightthickness=0,
                           padx=6, pady=6, font=(self.face, 10, "bold"), cursor="hand2").grid(
                               row=i//columns, column=i%columns, sticky="ew", padx=2, pady=2)

    def control_page(self):
        base = self.controls
        base.columnconfigure(0, weight=2, uniform="panels")
        base.columnconfigure(1, weight=3, uniform="panels")
        base.rowconfigure(0, weight=1)
        left = self.card(base, "运动控制")
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 5), pady=(7, 0))
        self.choices(left, ("单轮", "整车", "四轮自定"), self.mode, 3, self.mode_changed)
        self.selection = tk.Frame(left, bg=PAPER)
        self.selection.pack(fill="x")
        self.mode_changed()
        values = tk.Frame(left, bg=PAPER)
        values.pack(fill="x", pady=7)
        for i, (title, var, low, high) in enumerate((("轮速幅值  rad/s", self.speed, .01, 2),
                                                      ("点动时长  s", self.seconds, .01, 2))):
            values.columnconfigure(i, weight=1, uniform="settings")
            self.label(values, title, color=MUTED, anchor="w").grid(row=0, column=i, sticky="ew", padx=2)
            spin = ttk.Spinbox(values, textvariable=var, from_=low, to=high, increment=.1, width=7,
                               format="%.2f", font=(self.mono, 12))
            spin.grid(row=1, column=i, sticky="ew", padx=2, pady=3)
            if i == 0:
                self.speed_box = spin
        self.start_button = self.button(left, "▶   开始点动", self.start, TEAL, "white")
        self.start_button.pack(fill="x", pady=(5, 10))
        tk.Frame(left, height=1, bg=LINE).pack(fill="x", pady=5)
        self.label(left, "最近一次", color=MUTED, anchor="w").pack(fill="x", pady=(3, 1))
        self.label(left, variable=self.latest, bold=True, anchor="w").pack(fill="x", pady=(1, 5))
        self.button(left, "查看记录 / 填写观察", lambda: self.tabs.select(self.records)).pack(anchor="w")

        right = tk.Frame(base, bg=BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(5, 0), pady=(7, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1)
        feedback = self.card(right, "轮端反馈")
        feedback.grid(row=0, column=0, sticky="nsew")
        bar = tk.Frame(feedback, bg=PAPER)
        bar.pack(fill="x", pady=(0, 6))
        for var in (self.state, self.volts, self.timing):
            self.label(bar, variable=var, color=TEAL, bold=True).pack(side="left", padx=(0, 12))
        table = tk.Frame(feedback, bg=PAPER)
        table.pack(fill="x")
        self.cells = []
        for col, title in enumerate(("轮位", "试次目标  rad/s", "实测  rad/s", "累计转角  rad")):
            table.columnconfigure(col, weight=1)
            self.label(table, title, bg=BG, color=MUTED, size=9).grid(row=0, column=col, sticky="ew", ipady=3)
        for i, name in enumerate(WHEEL_NAMES):
            self.label(table, "●  "+name, bold=True, color=COLORS[i]).grid(row=i+1, column=0, pady=2)
            row = []
            for col in (1, 2, 3):
                cell = self.label(table, "—", mono=True, bold=col == 2, size=11)
                cell.grid(row=i+1, column=col)
                row.append(cell)
            self.cells.append(row)
        self.label(feedback, "实测轮速  ·  最近 5 s", anchor="w", color=MUTED, size=9).pack(fill="x", pady=(6, 0))
        self.chart = tk.Canvas(feedback, bg="#f7f9fa", highlightthickness=0, height=87)
        self.chart.pack(fill="both", expand=True, pady=(3, 0))
        imu = self.card(right, "IMU 六轴")
        imu.grid(row=1, column=0, sticky="ew", pady=(7, 0))
        axes = tk.Frame(imu, bg=PAPER)
        axes.pack(fill="x")
        self.imu_cells = []
        for i, name in enumerate(("", "X", "Y", "Z")):
            axes.columnconfigure(i, weight=1)
            self.label(axes, name, color=MUTED, size=9).grid(row=0, column=i)
        for i, name in enumerate(("加速度  m/s²", "角速度  rad/s")):
            self.label(axes, name, anchor="w", size=9).grid(row=i+1, column=0, sticky="w")
            for j in range(3):
                cell = self.label(axes, "—", mono=True)
                cell.grid(row=i+1, column=j+1)
                self.imu_cells.append(cell)
        pose_row = tk.Frame(imu, bg=PAPER)
        pose_row.pack(fill="x", pady=(6, 0))
        self.label(pose_row, variable=self.imu_state, color=MUTED, size=9).pack(side="left")
        self.pose_btn = self.button(pose_row, "采集此姿态", self.take_pose)
        self.pose_btn.pack(side="right")
        ttk.Combobox(pose_row, state="readonly", textvariable=self.pose, values=tuple(POSES), width=9).pack(
            side="right", padx=4)

    def mode_changed(self):
        for child in self.selection.winfo_children():
            child.destroy()
        mode = self.mode.get()
        if hasattr(self, "speed_box"):
            self.speed_box.configure(state="disabled" if mode == "四轮自定" else "normal")
        if mode == "单轮":
            self.label(self.selection, "↑  车头", color=MUTED, size=9).pack(pady=(3, 2))
            self.choices(self.selection, WHEEL_NAMES, self.wheel, 2)
            self.choices(self.selection, ("正向", "反向"), self.direction, 2)
        elif mode == "整车":
            self.label(self.selection, "车身运动", color=MUTED, size=9).pack(pady=(3, 2))
            self.choices(self.selection, tuple(PRESETS), self.motion, 2)
        else:
            self.label(self.selection, "四轮有符号目标  ·  rad/s", color=MUTED, anchor="w").pack(fill="x")
            form = tk.Frame(self.selection, bg=PAPER)
            form.pack(fill="x", pady=5)
            for i, name in enumerate(WHEEL_NAMES):
                col = (i%2)*2
                form.columnconfigure(col+1, weight=1)
                self.label(form, name).grid(row=i//2, column=col, padx=(4, 3))
                ttk.Entry(form, textvariable=self.custom[i], width=6).grid(row=i//2, column=col+1,
                                                                          sticky="ew", padx=(0, 6), pady=3)

    def record_page(self):
        box = tk.Frame(self.records, bg=PAPER, padx=15, pady=12)
        box.pack(fill="both", expand=True)
        top = tk.Frame(box, bg=PAPER)
        top.pack(fill="x")
        self.label(top, "点动记录", bold=True, size=14).pack(side="left")
        self.button(top, "打开原始记录目录", self.open_folder).pack(side="right")
        table = tk.Frame(box, bg=PAPER)
        table.pack(fill="both", expand=True, pady=8)
        cols = ("time", "target", "duration", "result", "observed")
        self.log = ttk.Treeview(table, columns=cols, show="headings", selectmode="browse", height=5)
        scroll = ttk.Scrollbar(table, orient="vertical", command=self.log.yview)
        self.log.configure(yscrollcommand=scroll.set)
        for key, title, width in zip(cols, ("开始时间", "动作", "时长", "后台判断", "实物观察"),
                                     (150, 140, 72, 100, 160)):
            self.log.heading(key, text=title)
            self.log.column(key, width=width, anchor="center")
        self.log.pack(side="left", fill="both", expand=True)
        scroll.pack(side="right", fill="y")
        self.log.bind("<<TreeviewSelect>>", lambda e: self.show_record())
        self.details = tk.StringVar(value="选择一条记录查看实际轮速、转角及停机结果。")
        self.label(box, variable=self.details, anchor="w", color=MUTED).pack(fill="x", pady=4)
        inputs = tk.Frame(box, bg=PAPER)
        inputs.pack(fill="x", pady=4)
        self.observed, self.observed_dir, self.note = tk.StringVar(), tk.StringVar(), tk.StringVar()
        for name, var, vals in (("实际轮位", self.observed, ("", *WHEEL_NAMES, "多轮", "未看清")),
                                ("轮胎上缘", self.observed_dir, ("", "朝车头", "朝车尾", "未看清"))):
            self.label(inputs, name, color=MUTED).pack(side="left", padx=(0, 4))
            ttk.Combobox(inputs, values=vals, textvariable=var, state="readonly", width=7).pack(side="left", padx=(0, 10))
        self.label(inputs, "现场备注", color=MUTED).pack(side="left")
        ttk.Entry(inputs, textvariable=self.note).pack(side="left", expand=True, fill="x", padx=5)
        self.button(inputs, "保存观察", self.save_note, TEAL, "white").pack(side="right")

    def add_record(self, path, select=True):
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return
        key = str(path)
        self.paths[key] = path
        observation = data.get("observation", {})
        result = data.get("assessment", "未记录结束")
        observed = " ".join(x for x in (observation.get("wheel"), observation.get("direction"),
                                        observation.get("result")) if x)
        values = (data["time"][:19].replace("T", " "), data["label"], f'{data["duration_ms"]/1000:g} s',
                  result, observed)
        if self.log.exists(key):
            self.log.item(key, values=values)
        else:
            self.log.insert("", 0, iid=key, values=values)
        if select:
            self.log.selection_set(key)
            self.log.see(key)
        if self.log.get_children()[0] == key:
            self.latest.set(f'{data["label"]}  ·  {result}')

    def show_record(self):
        selected = self.log.selection()
        if not selected:
            return
        try:
            data = json.loads(self.paths[selected[0]].read_text(encoding="utf-8"))
        except (OSError, ValueError) as error:
            self.details.set(f"无法读取此试次记录。{error}")
            return
        def vector(key):
            return " / ".join(f"{v:+.2f}" for v in data[key]) if data.get(key) is not None else "—"
        failed = [name for name, match in zip(WHEEL_NAMES, data.get("wheels_match") or ()) if not match]
        self.details.set(f"左前 / 右前 / 左后 / 右后      目标  {vector('target_rad_s')} rad/s\n"
                         f"末段均速  {vector('mean_rad_s')} rad/s      轮角增量  {vector('delta_rad')} rad\n"
                         f"停机回执  {'有' if data.get('stop_ack') else '未确认'}"
                         f"      编码器停稳  {'是' if data.get('settled') else '未确认'}"
                         + ("      跟踪需复核  "+"、".join(failed) if failed else ""))
        note = data.get("observation", {})
        self.observed.set(note.get("wheel", ""))
        self.observed_dir.set(note.get("direction", ""))
        self.note.set(note.get("note", ""))

    def save_note(self):
        selected = self.log.selection()
        if not selected:
            self.notice.set("请先选择要记录观察的那一次点动。")
            return
        path = self.paths[selected[0]]
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            wheel, direction = self.observed.get(), self.observed_dir.get()
            result = observation_result(data["target_rad_s"], wheel, direction)
            data["observation"] = dict(wheel=wheel, direction=direction, result=result, note=self.note.get())
            save_json(path, data)
            self.add_record(path)
            self.notice.set(f"{path.name} 已保存，实物观察{result}。")
        except (OSError, ValueError) as error:
            self.notice.set(str(error))

    def open_folder(self):
        selection = self.log.selection()
        path = self.paths[selection[0]].parent if selection else self.last_folder
        path.mkdir(parents=True, exist_ok=True)
        if os.name == "nt":
            os.startfile(path)
        else:
            subprocess.Popen(["open" if sys.platform == "darwin" else "xdg-open", str(path)])

    def connect(self):
        if self.session:
            self.disconnecting = True
            self.session.requests.put(("close",))
            self.notice.set("正在停机并关闭串口。")
            return
        name = self.port.get().strip()
        if not name:
            self.notice.set("请选择或输入串口名。")
            return
        try:
            self.lib = self.lib or library(self.args.library)
        except OSError as error:
            messagebox.showerror("串口库", str(error), parent=self.root)
            return
        self.last_folder = self.args.records/datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        self.session = Session(self.lib, name, self.last_folder)
        self.pending = self.disconnecting = False
        self.notice.set(f"正在与 {name} 握手，115200 bit/s · 8N1")
        self.session.start()

    def start(self):
        if not self.can_start():
            return
        try:
            label, duration, targets = target_values(self.mode.get(), self.wheel.get(), self.direction.get(),
                                                    self.motion.get(), self.speed.get(),
                                                    [v.get() for v in self.custom], self.seconds.get())
        except ValueError as error:
            self.notice.set(str(error) if "could not convert" not in str(error) else "请输入有效的轮速和时长。")
            return
        self.pending = True
        self.start_button.configure(state="disabled")
        self.session.requests.put(("jog", label, duration, targets))

    def can_start(self):
        s = self.session
        return bool(s and not self.pending and not self.disconnecting and not s.trial and not s.pose and s.wheel
                    and s.serial_state == READY and s.elapsed()-s.wheel.host < 250)

    def stop(self):
        if self.session:
            self.session.requests.put(("stop",))
            self.pending = False
            self.notice.set("已请求停机。")

    def take_pose(self):
        if self.can_start():
            self.pending = True
            self.session.requests.put(("pose", self.pose.get()))

    def close(self):
        self.exiting = True
        if self.session:
            self.disconnecting = True
            self.session.requests.put(("close",))
        else:
            self.root.destroy()

    def plot(self, rows, fresh):
        c = self.chart
        c.delete("all")
        w, h = max(2, c.winfo_width()), max(2, c.winfo_height())
        left, right, top, bottom = 30, w-7, 9, h-8
        limit = max(2, max((abs(v) for r in rows for v in r.speed), default=0))
        def y(v): return (top+bottom)/2 - v/limit*(bottom-top)/2
        for v in (limit, 0, -limit):
            yy = y(v)
            c.create_line(left, yy, right, yy, fill=LINE, dash=(3, 3))
            c.create_text(left-5, yy, text=f"{v:g}", anchor="e", fill=MUTED, font=(self.mono, 8))
        if not fresh or len(rows) < 2:
            return
        end = rows[-1].host
        for i, color in enumerate(COLORS):
            points = []
            prev = None
            for r in rows:
                if end-r.host > 5000:
                    continue
                if prev and (r.host-prev.host >= 250 or (r.board-prev.board) % 2**32 >= 250):
                    if len(points) >= 4: c.create_line(*points, fill=color, width=2)
                    points = []
                points.extend([right-(end-r.host)/5000*(right-left), y(r.speed[i])])
                prev = r
            if len(points) >= 4: c.create_line(*points, fill=color, width=2)

    def tick(self):
        s = self.session
        if s:
            try:
                while True:
                    kind, value = s.events.get_nowait()
                    self.pending = False
                    if kind == "trial":
                        self.add_record(value, select=self.tabs.index("current") == 0 or not self.log.selection())
                        self.notice.set(self.latest.get())
                    elif kind == "closed":
                        self.session = None
                        self.notice.set(value or "串口已关闭。")
                        self.disconnecting = False
                    else:
                        self.notice.set(value)
            except Empty:
                pass
        if self.exiting and not self.session:
            self.root.destroy()
            return
        s = self.session
        history = []
        if s:
            with s.lock:
                history = list(s.history)
        row = history[-1] if history else None
        now = s.elapsed() if s else 0
        fresh = bool(row and now-row.host < 250)
        self.status.set("断开中" if self.disconnecting else "● 已连接" if fresh else "轮反馈过期" if row else
                        "等待轮状态" if s else "未连接")
        self.badge.configure(fg="#99e5d5" if fresh else "#f1bc83" if s else "#cad6da")
        self.connection.configure(text="断开" if s else "连接", state="disabled" if self.disconnecting else "normal")
        self.port_box.configure(state="disabled" if s else "normal")
        self.stop_button.configure(state="normal" if s else "disabled")
        ready = self.can_start()
        self.start_button.configure(state="normal" if ready else "disabled")
        self.state.set(("已使能" if row.state == 1 else REASONS[row.reason] if row.reason < len(REASONS)
                        else "未知状态") if fresh else "—")
        self.volts.set(f"{row.volts:.2f} V" if fresh and row.volts is not None else "— V")
        span = (row.board-history[-20].board) % 2**32 if fresh and len(history) >= 20 else 0
        self.timing.set(f"{19000/span:.1f} Hz" if 0 < span < 5000 else "— Hz")
        trial = s.trial if s else None
        for i, (target, velocity, angle) in enumerate(self.cells):
            target.configure(text=f"{trial['target_rad_s'][i] if trial else 0:.2f}" if fresh else "—")
            velocity.configure(text=f"{row.speed[i]:+.3f}" if fresh else "—")
            angle.configure(text=f"{row.angle[i]:.3f}" if fresh else "—")
        imu = s.imu if s else None
        imu_fresh = bool(imu and now-imu[0] < 250)
        self.pose_btn.configure(state="normal" if ready and imu_fresh else "disabled")
        self.imu_state.set("正在采集" if s and s.pose else "六轴在线" if imu_fresh else "样本过期" if imu else "等待样本")
        for i, cell in enumerate(self.imu_cells):
            cell.configure(text=f"{imu[2+i//3][i%3]:+.3f}" if imu_fresh else "—")
        if self.tabs.index("current") == 0:
            self.plot(history, fresh)
        self.root.after(100, self.tick)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="open this port on launch with the drive disabled")
    parser.add_argument("--library", type=Path, help="path to the native baize_panel_io library")
    parser.add_argument("--records", type=Path, default=records_root())
    root = tk.Tk()
    Panel(root, parser.parse_args())
    root.mainloop()
