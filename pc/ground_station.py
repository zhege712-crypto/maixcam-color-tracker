"""
智能搬运 · 上位机（图形界面）
=============================

给 MaixCAM 相机端配的调试/演示界面。一根 USB 转串口（CH340）接相机的
A19(TX)，就能看到相机现在盯着谁、对准了没有、离动作点还差多少像素。

它能干的事：
    · 选串口、连接、实时显示帧率 / 丢帧 / 坏帧
    · 把归一化坐标还原成画面框，实时画出来（按颜色上色，带轨迹）
    · 显示状态机当前阶段、目标颜色、四个坐标字段的原始值
    · 下发全部下行命令：NEXT / ABORT / PING / RUN / STANDBY /
      SET_COLOR / SET_FPS / CALIBRATE / SAVE
    · 收到相机的回执就在日志里标 OK / FAIL —— 命令到底成功没有，一眼可见
    · 录制 CSV（离线复盘用）
    · 演示模式：没有相机也没有串口也能把界面跑起来

用法：
    python ground_station.py                 打开界面，手动选串口连接
    python ground_station.py --port COM5     启动即连接
    python ground_station.py --demo          演示模式（不需要任何硬件）
    python ground_station.py --port loop://  自检串口通路（pyserial 的回环假口）

依赖：
    pip install pyserial        # 只有真实串口需要；演示模式不需要

接线（和主控接线方式不同，注意别冲突）：
    相机 A19(TX) -> CH340 RX
    GND 共地
    如果主控也在线上，把 CH340 的 RX 并在 A19 上监听即可（不要并 TX，
    否则上位机和主控会同时往相机的 A18 上发，冲突）。
"""

import argparse
import math
import os
import queue
import sys
import threading
import time
from collections import deque

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P  # noqa: E402

try:
    import tkinter as tk
    from tkinter import ttk
except Exception:                                            # noqa: BLE001
    print("这个工具需要 tkinter（Python 自带）。如果你用的是精简版 Python，"
          "装完整版即可。")
    raise

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except Exception:                                            # noqa: BLE001
    serial = None
    list_ports = None
    HAVE_SERIAL = False


# ================================================================ 主题

BG = "#14161b"
PANEL = "#1c1f26"
PANEL_HI = "#242833"
BORDER = "#2e3440"
FG = "#d8dee9"
FG_DIM = "#7c8698"
CANVAS_BG = "#0b0d11"

C_RED = "#ff5a5f"
C_GREEN = "#3ddc84"
C_BLUE = "#4da3ff"
C_YELLOW = "#f5c451"
C_CYAN = "#56d4dd"

COLOR_HEX = {"red": C_RED, "green": C_GREEN, "blue": C_BLUE, "?": FG_DIM}

# 状态对应的强调色
STATE_HEX = {
    P.STATE_SEARCH: FG_DIM,
    P.STATE_MATERIAL: C_YELLOW,
    P.STATE_GRAB: C_GREEN,
    P.STATE_DEST: C_CYAN,
    P.STATE_DROP: C_BLUE,
    P.STATE_DONE: C_GREEN,
}

FONT_UI = ("Microsoft YaHei UI", 9)
FONT_UI_B = ("Microsoft YaHei UI", 9, "bold")
FONT_BIG = ("Microsoft YaHei UI", 13, "bold")
FONT_MONO = ("Consolas", 9)
FONT_MONO_B = ("Consolas", 10, "bold")


def hexs(data):
    return " ".join("%02X" % b for b in data)


# ================================================================ 画面换算

def norm_box(x_n, y_n, w_n, h_n, cw, ch):
    """归一化字段 -> 画布上的 (x, y, w, h)。

    注意：协议里 w_n 和 h_n **都**是按画面宽度归一化的（见协议文档），
    所以换算高度时用的也是 cw 而不是 ch。这一点搞错的话，框会被拉高。
    """
    cx = x_n / 255.0 * cw
    cy = y_n / 255.0 * ch
    bw = w_n / 255.0 * cw
    bh = h_n / 255.0 * cw
    return cx - bw / 2.0, cy - bh / 2.0, bw, bh


# ================================================================ 帧统计

class FrameStats:
    """上行帧统计（纯逻辑，不碰界面 —— 可以直接单测）。

    三件很容易做错、而且做错了会误导现场判断的事，集中在这里处理：

    1. **断线不记丢帧。** 拔插 USB 或者相机重启之后，帧序号会跳一大截。
       按序号差直接累加的话，一次重启会被记成几十上百帧丢失，
       真正的丢帧（接线松动、波特率不对）就淹没在噪声里了。
       所以：距上一帧超过 `resync_after` 秒才收到的第一帧算"重新同步"，
       不计入丢帧，单独记 resyncs。
    2. **帧率用真时间窗**（最近 1 秒实际收到多少帧），
       而不是"总帧数 / 总时长" —— 后者在断流后会一直虚高，骗人。
    3. **断流判定**：超过 `stale_after` 秒没有上行帧就置 stale。
       界面据此显示"数据过期"，而不是一直画着最后一帧装作还活着。
    """

    def __init__(self, window=1.0, stale_after=1.0, resync_after=0.8):
        self.window = window
        self.stale_after = stale_after
        self.resync_after = resync_after
        self.reset()

    def reset(self):
        self.total = 0
        self.no_target = 0
        self.lost = 0            # 真正的丢帧（序号断了，但链路一直通着）
        self.resyncs = 0         # 断线/重启后的重新同步次数（不计入丢帧）
        self.fps = 0.0
        self.idle = 0.0          # 距上一帧多久
        self.stale = False
        self.last_recv = None
        self.prev_seq = None
        self.last_state = None
        self.last_color = None
        self._marks = deque()

    def on_frame(self, now, f):
        """喂一帧上行数据。返回 True 表示状态或颜色变了（值得记一行日志）。"""
        if self.last_recv is not None:
            if now - self.last_recv > self.resync_after:
                self.resyncs += 1          # 断过线，序号不可比，别算丢帧
            else:
                self.lost += P.seq_gap(self.prev_seq, f["seq"])
        self.prev_seq = f["seq"]
        self.last_recv = now
        self.total += 1
        if not f["exists"]:
            self.no_target += 1

        self._marks.append(now)
        while self._marks and now - self._marks[0] > self.window:
            self._marks.popleft()

        changed = (f["state"] != self.last_state
                   or f["color_name"] != self.last_color)
        self.last_state = f["state"]
        self.last_color = f["color_name"]
        return changed

    def tick(self, now):
        """推进时间（每帧都可以调，代价很小）。"""
        if self.last_recv is None:
            self.fps = 0.0
            self.idle = 0.0
            self.stale = True
            return
        while self._marks and now - self._marks[0] > self.window:
            self._marks.popleft()
        self.idle = max(0.0, now - self.last_recv)
        self.stale = self.idle > self.stale_after
        # 断流时不要在界面上显示一个虚高的帧率
        self.fps = 0.0 if self.stale else float(len(self._marks)) / self.window

    @property
    def loss_pct(self):
        denom = self.total + self.lost
        return 100.0 * self.lost / denom if denom else 0.0


# ================================================================ 数据源

class SerialLink:
    """真实串口：读线程负责收字节、解帧，主线程只管取。"""

    is_demo = False

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.parser = P.MixedParser()
        self._q = queue.Queue()
        self._alive = False
        self._ser = None
        self._thread = None
        self.error = None

    def open(self):
        if not HAVE_SERIAL:
            raise RuntimeError("没装 pyserial（pip install pyserial）")
        # 支持 pyserial 的 URL 形式（比如 loop://），方便没有硬件时自检
        if "://" in self.port:
            self._ser = serial.serial_for_url(self.port, self.baud, timeout=0.05)
        else:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.05)
        time.sleep(0.2)
        self._ser.reset_input_buffer()
        self._alive = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while self._alive:
            try:
                n = self._ser.in_waiting or 1
                data = self._ser.read(n)
            except Exception as e:                           # noqa: BLE001
                self.error = str(e)
                self._alive = False
                break
            if data:
                # 解析只在这个线程里做，不需要加锁
                for f in self.parser.feed(data):
                    self._q.put(f)

    def close(self):
        self._alive = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:                                # noqa: BLE001
                pass
            self._ser = None

    def send(self, data):
        if self._ser is None:
            return False
        self._ser.write(data)
        return True

    def drain(self):
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                break
        return out

    @property
    def bad_frames(self):
        return self.parser.bad_frames


# ---- 演示数据 ----------------------------------------------------------

DEMO_PERIOD = 10.0

# 演示用的关键点（归一化坐标）。数值照着现场日志的真实范围取的：
# 物料 w_n 40~55、色环 w_n 34~40、中心 128。
DEMO_KEYS = {
    "far": (72, 108, 30, 62),
    "grab": (128, 128, 52, 108),
    "ring": (196, 168, 40, 78),
}


def _lerp(a, b, p):
    return a + (b - a) * p


def _ease(p):
    p = 0.0 if p < 0 else (1.0 if p > 1 else p)
    return p * p * (3.0 - 2.0 * p)


def demo_pose(t):
    """按时间返回 (exists, color, state, xn, yn, wn, hn)。"""
    colors = ("red", "green", "blue")
    color = colors[int(t / DEMO_PERIOD) % 3]
    tt = t % DEMO_PERIOD
    far, grab, ring = DEMO_KEYS["far"], DEMO_KEYS["grab"], DEMO_KEYS["ring"]

    def mix(k1, k2, p):
        p = _ease(p)
        return tuple(_lerp(k1[i], k2[i], p) for i in range(4))

    if tt < 1.6:                       # 搜索：目标在左前方晃
        p = tt / 1.6
        xn = far[0] + 14 * math.sin(p * 7.0)
        yn = far[1] + 8 * math.cos(p * 5.0)
        return 1, color, P.STATE_SEARCH, xn, yn, far[2], far[3]
    if tt < 4.0:                       # 对准物料：往中心挪
        return (1, color, P.STATE_MATERIAL) + mix(far, grab, (tt - 1.6) / 2.4)
    if tt < 4.8:                       # 可抓取
        return (1, color, P.STATE_GRAB) + grab
    if tt < 7.0:                       # 对准目标区
        return (1, color, P.STATE_DEST) + mix(grab, ring, (tt - 4.8) / 2.2)
    if tt < 7.8:                       # 可放置
        return (1, color, P.STATE_DROP) + ring
    if tt < 9.0:                       # 完成，画面里暂时没目标
        return 0, color, P.STATE_DONE, 0, 0, 0, 0
    return 1, color, P.STATE_SEARCH, far[0], far[1], far[2], far[3]


class DemoLink:
    """演示数据源：没有相机也能把界面跑起来，接口和 SerialLink 一样。"""

    is_demo = True

    def __init__(self):
        self._t0 = 0.0
        self._seq = 0
        self._next = 0.0
        self._acks = queue.Queue()
        self._frames = 0
        self.error = None

    def open(self):
        self._t0 = time.time()
        self._next = time.time()

    def close(self):
        pass

    def send(self, data):
        d = P.parse_downlink(data)
        if d is None:
            return False
        # 搬运模式下相机没有"单一追踪色"可标定，CALIBRATE 会回 FAIL —— 照实模拟
        ok = P.STATUS_FAIL if d["cmd"] == P.CMD_CALIBRATE else P.STATUS_OK
        ack = P.parse_downlink(P.build_ack(d["cmd"], ok))
        self._acks.put(dict(ack, kind="downlink", demo_ok=(ok == P.STATUS_OK)))
        return True

    def drain(self):
        out = []
        while True:
            try:
                out.append(self._acks.get_nowait())
            except queue.Empty:
                break

        now = time.time()
        n = 0
        while now >= self._next and n < 40:
            exists, color, state, xn, yn, wn, hn = demo_pose(now - self._t0)
            code = P.COLOR_VALUES.get(color, 1) if exists else 0
            raw = P.build_uplink(
                self._seq, 1 if exists else 0, code, state,
                int(round(xn)), int(round(yn)),
                int(round(wn)), int(round(hn)))
            out.append(dict(P.parse_uplink(raw), kind="uplink"))
            self._seq = (self._seq + 1) & 0xFF
            self._frames += 1
            if self._frames % 97 == 0:      # 偶尔丢一帧，让"丢帧"统计不是死的
                self._seq = (self._seq + 1) & 0xFF
            self._next += 1.0 / 14.0
            n += 1
        if n >= 40:
            self._next = now
        return out

    @property
    def bad_frames(self):
        return 0


# ================================================================ 界面

class GroundStation:

    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.link = None

        # 运行时状态
        self.last = None                     # 最近一帧上行数据
        self.stats = FrameStats()
        self.ack_ok = 0
        self.ack_fail = 0
        self.last_ack = None
        self.stamp_bad = 0                   # 扫描坏帧用的上一次快照
        self._trail = deque(maxlen=24)
        self._t_last_stat = time.time()
        self._last_stat_total = -1
        self._rec = None
        self._rec_path = None
        self._csv_rows = 0
        self._was_stale = None
        self.standby = False

        root.title("智能搬运 · 上位机  |  MaixCAM 相机调试")
        root.configure(bg=BG)
        root.geometry("1000x820")
        root.minsize(940, 780)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._build_toolbar()
        self._build_body()
        self._build_log()

        self._dirty_canvas = True
        self.root.after(40, self._tick)

        if args.demo:
            self.start_link(DemoLink())
        elif args.port:
            self.connect(args.port, args.baud)

    # ------------------------------------------------------------ 布局

    def _build_toolbar(self):
        bar = tk.Frame(self.root, bg=PANEL, height=54)
        bar.pack(side="top", fill="x")
        bar.pack_propagate(False)

        def lab(text, fg=FG_DIM):
            return tk.Label(bar, text=text, bg=PANEL, fg=fg, font=FONT_UI)

        lab("串口").pack(side="left", padx=(12, 4), pady=14)

        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=14,
                                    state="readonly", font=FONT_MONO)
        self.port_cb.pack(side="left", pady=14)

        self._refresh_ports()

        self.btn_refresh = self._btn(bar, "刷新", self._refresh_ports, wide=5)
        self.btn_refresh.pack(side="left", padx=(6, 12), pady=14)

        lab("波特率").pack(side="left", padx=(0, 4), pady=14)
        self.baud_var = tk.StringVar(value=str(self.args.baud))
        ttk.Combobox(bar, textvariable=self.baud_var, width=8, state="readonly",
                     values=("115200", "921600", "460800", "230400", "57600"),
                     font=FONT_MONO).pack(side="left", pady=14)

        self.btn_conn = self._btn(bar, "连接", self.toggle_connect, wide=7,
                                  accent=C_GREEN)
        self.btn_conn.pack(side="left", padx=10, pady=14)

        # 右侧：运行指示
        self.lbl_led = tk.Label(bar, text="●", bg=PANEL, fg="#4a4f5a",
                                font=("Arial", 15))
        self.lbl_led.pack(side="right", padx=(0, 8), pady=14)
        self.lbl_conn = tk.Label(bar, text="未连接", bg=PANEL, fg=FG_DIM,
                                 font=FONT_UI)
        self.lbl_conn.pack(side="right", padx=(0, 2), pady=14)

        self.lbl_fps_top = tk.Label(bar, text="", bg=PANEL, fg=C_CYAN,
                                    font=FONT_MONO_B)
        self.lbl_fps_top.pack(side="right", padx=(0, 22), pady=14)

        tk.Frame(self.root, bg=BORDER, height=1).pack(side="top", fill="x")

    def _btn(self, parent, text, cmd, wide=6, accent=None, bg=None):
        fg = "#0b0d11" if accent else FG
        b = tk.Button(parent, text=text, command=cmd, font=FONT_UI,
                      bg=accent or bg or PANEL_HI, fg=fg,
                      activebackground=BORDER, activeforeground=FG,
                      relief="flat", bd=0, width=wide, cursor="hand2",
                      highlightthickness=0)
        return b

    def _build_body(self):
        body = tk.Frame(self.root, bg=BG)
        body.pack(side="top", fill="both", expand=True, padx=10, pady=10)

        # ---- 左：画面
        left = tk.Frame(body, bg=PANEL, highlightthickness=1,
                        highlightbackground=BORDER)
        left.pack(side="left", fill="both", expand=True)

        head = tk.Frame(left, bg=PANEL, height=28)
        head.pack(side="top", fill="x")
        head.pack_propagate(False)
        tk.Label(head, text="画面（640×480 归一化还原）", bg=PANEL, fg=FG_DIM,
                 font=FONT_UI).pack(side="left", padx=10)
        self.lbl_hud = tk.Label(head, text="", bg=PANEL, fg=FG_DIM,
                                font=FONT_MONO)
        self.lbl_hud.pack(side="right", padx=10)

        self.cw, self.ch = 560, 420
        self.canvas = tk.Canvas(left, width=self.cw, height=self.ch,
                                bg=CANVAS_BG, highlightthickness=0)
        # 跟着窗口长：画面是归一化坐标，铺满才好看，也不会留下空白
        self.canvas.pack(side="top", fill="both", expand=True,
                         padx=10, pady=(0, 10))
        self.canvas.bind("<Configure>", self._on_canvas_resize)
        self._draw_idle()

        # ---- 右：信息 + 控制
        right = tk.Frame(body, bg=BG, width=372)
        right.pack(side="left", fill="y", padx=(10, 0))
        right.pack_propagate(False)

        self._build_info(right)
        self._build_controls(right)

    def _card(self, parent, title):
        wrap = tk.Frame(parent, bg=PANEL, highlightthickness=1,
                        highlightbackground=BORDER)
        wrap.pack(side="top", fill="x", pady=(0, 8))
        tk.Label(wrap, text=title, bg=PANEL, fg=FG_DIM, font=FONT_UI,
                 anchor="w").pack(side="top", fill="x", padx=10, pady=(6, 2))
        return wrap

    def _build_info(self, parent):
        """状态 + 坐标压成两块紧凑的卡片 —— 右栏必须给控制面板留够高度。"""
        card = self._card(parent, "状态")

        self.v_state = tk.StringVar(value="—")
        self.v_color = tk.StringVar(value="—")
        self.v_seq = tk.StringVar(value="—")
        self.v_fps = tk.StringVar(value="—")
        self.v_loss = tk.StringVar(value="—")
        self.v_bad = tk.StringVar(value="—")
        self.v_ack = tk.StringVar(value="—")

        grid = tk.Frame(card, bg=PANEL)
        grid.pack(side="top", fill="x", padx=10, pady=(0, 8))
        grid.columnconfigure(1, weight=1)
        grid.columnconfigure(3, weight=1)

        def cell(r, c, name, var, font):
            tk.Label(grid, text=name, bg=PANEL, fg=FG_DIM, font=FONT_UI,
                     anchor="w", width=5).grid(row=r, column=c, sticky="w")
            lbl = tk.Label(grid, textvariable=var, bg=PANEL, fg=FG, font=font,
                           anchor="w")
            lbl.grid(row=r, column=c + 1, sticky="w", padx=(0, 12))
            return lbl

        self.lbl_state = cell(0, 0, "状态", self.v_state, FONT_BIG)
        self.lbl_state.grid_configure(columnspan=3)
        self.lbl_color = cell(1, 0, "颜色", self.v_color, FONT_UI_B)
        self.lbl_color.grid_configure(columnspan=3)
        cell(2, 0, "序号", self.v_seq, FONT_MONO)
        cell(2, 2, "帧率", self.v_fps, FONT_MONO)
        cell(3, 0, "丢帧", self.v_loss, FONT_MONO)
        cell(3, 2, "坏帧", self.v_bad, FONT_MONO)
        cell(4, 0, "回执", self.v_ack, FONT_MONO).grid_configure(columnspan=3)

        # ---- 坐标
        card2 = self._card(parent, "坐标")
        self.v_coord = {}
        grid2 = tk.Frame(card2, bg=PANEL)
        grid2.pack(side="top", fill="x", padx=10, pady=(0, 6))

        for idx, (short, key) in enumerate((("x", "x_n"), ("y", "y_n"),
                                            ("w", "w_n"), ("h", "h_n"))):
            r, c = divmod(idx, 2)
            cellf = tk.Frame(grid2, bg=PANEL)
            cellf.grid(row=r, column=c, sticky="w", padx=(0, 26), pady=1)
            tk.Label(cellf, text=short, bg=PANEL, fg=FG_DIM, font=FONT_MONO,
                     width=2, anchor="w").pack(side="left")
            self.v_coord[key] = (tk.StringVar(value="—"),
                                 tk.StringVar(value="—"))
            tk.Label(cellf, textvariable=self.v_coord[key][0], bg=PANEL, fg=FG,
                     font=FONT_MONO, width=4, anchor="e").pack(side="left")
            tk.Label(cellf, text="→", bg=PANEL, fg="#3d4a5c",
                     font=FONT_MONO).pack(side="left", padx=2)
            tk.Label(cellf, textvariable=self.v_coord[key][1], bg=PANEL,
                     fg=FG_DIM, font=FONT_MONO, width=4, anchor="w").pack(
                         side="left")

        self.v_delta = tk.StringVar(value="Δ —")
        tk.Label(grid2, textvariable=self.v_delta, bg=PANEL, fg=C_YELLOW,
                 font=FONT_MONO, anchor="w").grid(row=2, column=0, columnspan=2,
                                                  sticky="w", pady=(3, 0))

    def _build_controls(self, parent):
        card = self._card(parent, "控制")

        row1 = tk.Frame(card, bg=PANEL)
        row1.pack(side="top", fill="x", padx=10)
        self._btn(row1, "PING", lambda: self.cmd(P.CMD_PING, 0, "PING"),
                  wide=6).pack(side="left")
        self._btn(row1, "NEXT", lambda: self.cmd(P.CMD_NEXT, 0, "NEXT"),
                  wide=6, accent=C_GREEN).pack(side="left", padx=6)
        self._btn(row1, "ABORT", lambda: self.cmd(P.CMD_ABORT, 0, "ABORT"),
                  wide=6).pack(side="left")

        row2 = tk.Frame(card, bg=PANEL)
        row2.pack(side="top", fill="x", padx=10, pady=(6, 0))
        self.btn_run = self._btn(row2, "RUN", self.toggle_run, wide=6)
        self.btn_run.pack(side="left")
        self._btn(row2, "SAVE", lambda: self.cmd(P.CMD_SAVE, 0, "SAVE"),
                  wide=6).pack(side="left", padx=6)
        self._btn(row2, "CAL", lambda: self.cmd(P.CMD_CALIBRATE, 0, "CALIBRATE"),
                  wide=6).pack(side="left")

        row3 = tk.Frame(card, bg=PANEL)
        row3.pack(side="top", fill="x", padx=10, pady=(6, 0))
        tk.Label(row3, text="切换颜色", bg=PANEL, fg=FG_DIM,
                 font=FONT_UI).pack(side="left")
        for name, code, cname in (("红", P.COLOR_RED, "red"),
                                  ("绿", P.COLOR_GREEN, "green"),
                                  ("蓝", P.COLOR_BLUE, "blue")):
            b = self._btn(row3, name,
                          lambda c=code, n=cname: self.cmd(P.CMD_SET_COLOR, c,
                                                           "SET_COLOR " + n),
                          wide=3)
            b.configure(fg=COLOR_HEX[cname])
            b.pack(side="left", padx=(6, 0))

        row4 = tk.Frame(card, bg=PANEL)
        row4.pack(side="top", fill="x", padx=10, pady=(6, 0))
        tk.Label(row4, text="帧率", bg=PANEL, fg=FG_DIM,
                 font=FONT_UI).pack(side="left")
        self.fps_var = tk.StringVar(value="14")
        ttk.Combobox(row4, textvariable=self.fps_var, width=5, state="readonly",
                     values=("5", "10", "14", "20", "25", "30", "60"),
                     font=FONT_MONO).pack(side="left", padx=(6, 6))
        self._btn(row4, "下发", self.send_fps, wide=5).pack(side="left")

        row5 = tk.Frame(card, bg=PANEL)
        row5.pack(side="top", fill="x", padx=10, pady=(6, 4))
        self.btn_rec = self._btn(row5, "录制 CSV", self.toggle_record, wide=10)
        self.btn_rec.pack(side="left")
        self._btn(row5, "清空日志", self.clear_log, wide=9).pack(side="left",
                                                                padx=6)

        tip = "NEXT / ABORT 平时由主控发，这里手动发是为了联调。"
        tk.Label(card, text=tip, bg=PANEL, fg=FG_DIM,
                 font=("Microsoft YaHei UI", 8), wraplength=340,
                 justify="left", anchor="w").pack(
                     side="top", fill="x", padx=10, pady=(0, 8))

    def _build_log(self):
        wrap = tk.Frame(self.root, bg=PANEL, highlightthickness=1,
                        highlightbackground=BORDER)
        wrap.pack(side="top", fill="both", padx=10, pady=(0, 10))

        head = tk.Frame(wrap, bg=PANEL, height=24)
        head.pack(side="top", fill="x")
        head.pack_propagate(False)
        tk.Label(head, text="日志", bg=PANEL, fg=FG_DIM,
                 font=FONT_UI).pack(side="left", padx=10)

        box = tk.Frame(wrap, bg=PANEL)
        box.pack(side="top", fill="both", expand=True, padx=6, pady=(0, 6))
        self.log_text = tk.Text(box, height=9, bg=CANVAS_BG, fg=FG,
                                font=FONT_MONO, relief="flat", bd=0,
                                insertbackground=FG, wrap="none")
        sb = tk.Scrollbar(box, command=self.log_text.yview, width=12)
        self.log_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_text.pack(side="left", fill="both", expand=True)

        self.log_text.tag_configure("info", foreground=FG_DIM)
        self.log_text.tag_configure("tx", foreground=C_YELLOW)
        self.log_text.tag_configure("ok", foreground=C_GREEN)
        self.log_text.tag_configure("err", foreground=C_RED)
        self.log_text.tag_configure("rx", foreground=C_CYAN)
        self.log_text.tag_configure("hint", foreground=C_BLUE)
        self.log_text.configure(state="disabled")

    # ------------------------------------------------------------ 日志

    def log(self, text, tag="info"):
        # 界面还没建完就被调用（比如启动时扫串口失败），直接忽略
        if not hasattr(self, "log_text"):
            return
        # 用户往上滚着看历史时不要把他拽回底部 —— 这是最烦人的一种"优化"
        follow = self.log_text.yview()[1] >= 0.999
        t = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", "%s  %s\n" % (t, text), tag)
        lines = int(self.log_text.index("end-1c").split(".")[0])
        if lines > 600:
            self.log_text.delete("1.0", "%d.0" % (lines - 600))
        if follow:
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    # ------------------------------------------------------------ 连接

    def _refresh_ports(self):
        if not HAVE_SERIAL:
            self.port_cb["values"] = ("(未安装 pyserial)",)
            self.port_var.set("(未安装 pyserial)")
            self.log("没找到 pyserial，串口模式不可用。安装：pip install pyserial",
                     "err")
            return
        ports = list(list_ports.comports())
        vals = ["%s  %s" % (p.device, p.description) for p in ports]
        self._port_map = {v: p.device for v, p in zip(vals, ports)}
        self.port_cb["values"] = vals if vals else ("(没有发现串口)",)
        if vals:
            self.port_var.set(vals[0])
        else:
            self.port_var.set("(没有发现串口)")
            self.log("没有发现串口设备。检查 CH340 驱动、USB 线，以及相机是否上电。",
                     "err")

    def toggle_connect(self):
        if self.link is not None:
            self.disconnect()
            return
        if self.args.demo:
            self.start_link(DemoLink())
            return
        label = self.port_var.get()
        port = getattr(self, "_port_map", {}).get(label)
        if not port:
            self.log("请先选择一个真实串口（点「刷新」重新扫描）", "err")
            return
        self.connect(port, int(self.baud_var.get()))

    def connect(self, port, baud):
        # 开串口交给 start_link 统一处理（它会 open 并登记状态）
        self.start_link(SerialLink(port, baud))

    def start_link(self, link):
        self.link = link
        try:
            link.open()
        except Exception as e:                               # noqa: BLE001
            self.link = None
            self.log("打开 %s 失败：%s" % (getattr(link, "port", "数据源"), e),
                     "err")
            return
        self.stats.reset()
        self.ack_ok = self.ack_fail = 0
        self.last_ack = None
        self.stamp_bad = 0
        self._trail.clear()
        self._was_stale = None
        self._last_stat_total = -1
        self.last = None
        self.lbl_led.configure(fg=C_GREEN)
        if link.is_demo:
            self.port_var.set("(演示模式)")
            self.lbl_conn.configure(text="演示模式", fg=C_YELLOW)
            self.btn_conn.configure(text="断开")
            self.log("演示模式：数据是程序自己造的，没接任何硬件。", "hint")
            self.log("界面上的东西都是真的在跑 —— 只是目标不是真相机看的。",
                     "hint")
        else:
            self.lbl_conn.configure(text=getattr(link, "port", "已连接"),
                                    fg=C_GREEN)
            self.btn_conn.configure(text="断开")
            self.log("开始接收。若一直没有数据：检查 A19 是否接到 RX、"
                     "GND 是否共地、波特率是否 115200。", "hint")

    def disconnect(self):
        if self.link is not None:
            try:
                self.link.close()
            except Exception:                                # noqa: BLE001
                pass
            self.link = None
        # 断开后不要把最后一帧留在画布上 —— 那看起来和"正在接收"一模一样
        self.last = None
        self._trail.clear()
        self.stats.reset()
        self._was_stale = None
        self._dirty_canvas = True
        self.v_fps.set("—")
        self.lbl_fps_top.configure(text="")
        self.lbl_hud.configure(text="")
        self.lbl_led.configure(fg="#4a4f5a")
        self.lbl_conn.configure(text="未连接", fg=FG_DIM)
        self.btn_conn.configure(text="连接")
        self.log("已断开", "info")

    def on_close(self):
        self.disconnect()
        if self._rec:
            try:
                self._rec.close()
            except Exception:                                # noqa: BLE001
                pass
        self.root.destroy()

    # ------------------------------------------------------------ 下行命令

    def cmd(self, code, arg, name):
        if self.link is None:
            self.log("还没连接，命令没发出去", "err")
            return
        frame = P.build_downlink(code, arg)
        try:
            self.link.send(frame)
        except Exception as e:                               # noqa: BLE001
            self.log("发送失败：%s" % e, "err")
            return
        self.log("-> %-14s %s" % (name, hexs(frame)), "tx")

    def send_fps(self):
        try:
            v = int(self.fps_var.get())
        except ValueError:
            return
        if not 1 <= v <= 60:
            self.log("帧率只能是 1~60", "err")
            return
        self.cmd(P.CMD_SET_FPS, v, "SET_FPS %d" % v)

    def toggle_run(self):
        self.standby = not self.standby
        arg = 0 if self.standby else 1
        self.cmd(P.CMD_RUN_STANDBY, arg, "STANDBY" if self.standby else "RUN")
        self.btn_run.configure(text="STBY" if self.standby else "RUN")

    # ------------------------------------------------------------ CSV

    def toggle_record(self):
        if self._rec:
            try:
                self._rec.close()
            except Exception:                                # noqa: BLE001
                pass
            self._rec = None
            self.btn_rec.configure(text="录制 CSV", fg=FG)
            self.log("已停止录制：%s（%d 行）" % (self._rec_path, self._csv_rows),
                     "ok")
            return
        os.makedirs("captures", exist_ok=True)
        path = os.path.join("captures",
                            "gs-%s.csv" % time.strftime("%Y%m%d-%H%M%S"))
        try:
            self._rec = open(path, "w", encoding="utf-8-sig", newline="")
            self._rec.write("t,seq,exists,color,state,x_n,y_n,w_n,h_n,dx,dy\n")
        except OSError as e:
            self.log("建不了 CSV：%s" % e, "err")
            self._rec = None
            return
        self._rec_path = path
        self._csv_rows = 0
        self.btn_rec.configure(text="停止录制", fg=C_RED)
        self.log("开始录制：%s" % path, "ok")

    # ------------------------------------------------------------ 主循环

    def _tick(self):
        try:
            self._poll()
            # 断流时"数据过期 N.Ns"的读数在走，得持续重画
            if self.link is not None and self.stats.stale:
                self._dirty_canvas = True
            if self._dirty_canvas:
                self._redraw()
                self._draw_overlay()
                self._dirty_canvas = False
        except Exception as e:                               # noqa: BLE001
            self.log("界面异常：%s" % e, "err")
        self.root.after(40, self._tick)

    def _poll(self):
        if self.link is not None and getattr(self.link, "error", None):
            self.log("串口出错：%s" % self.link.error, "err")
            self.link.error = None
            self.disconnect()

        now = time.time()
        if self.link is None:
            self.stats.tick(now)
            return

        frames = self.link.drain()
        dropped = 0
        if len(frames) > self.MAX_DRAIN:
            # 界面卡住过（拖窗口、最小化恢复），一次涌进来太多帧。
            # 只处理最后一批，否则会把界面再卡一次。
            dropped = len(frames) - self.MAX_DRAIN
            frames = frames[-self.MAX_DRAIN:]

        changed = False
        for f in frames:
            if f["kind"] != "uplink":
                self._on_ack(f)
                continue

            # 状态/颜色变了才记日志，否则日志会被 30 帧/秒刷爆
            if self.stats.on_frame(now, f):
                sname = P.STATE_NAMES.get(f["state"], "?")
                if f["exists"]:
                    self.log("目标 %s · %s" % (sname, f["color_name"]), "rx")
                else:
                    self.log("状态 %s · 画面里没有目标" % sname, "info")

            if f["exists"]:
                self._trail.append((f["x_n"], f["y_n"]))
            self.last = f
            changed = True
            self._write_csv(now, f)

        self.stats.tick(now)

        bad = self.link.bad_frames
        if bad != self.stamp_bad:
            self.log("坏帧 +%d（校验和/帧尾不对，通常是接线或波特率）"
                     % (bad - self.stamp_bad), "err")
            self.stamp_bad = bad

        if dropped:
            self.log("界面卡顿，跳过 %d 帧（统计已计入）" % dropped, "info")

        if changed and self.last is not None:
            self._dirty_canvas = True
            self._render(self.last)

        self._update_link_status(now, bad)

        if now - self._t_last_stat >= 1.0:
            self._t_last_stat = now
            s = self.stats
            # 没有新帧就别重复打同一行 —— 断流时日志会被同一组数字刷满
            if s.total and s.total != self._last_stat_total:
                self._last_stat_total = s.total
                self.log("统计：累计 %d 帧，无目标 %d，丢帧 %d，重连 %d，坏帧 %d"
                         % (s.total, s.no_target, s.lost, s.resyncs, bad))
            self._flush_csv()

    # 一次最多处理多少帧 —— 界面跟不上时宁可跳过，也不要雪崩
    MAX_DRAIN = 400

    def _update_link_status(self, now, bad):
        """帧率 / 坏帧 / 断流指示（每 tick 都跑，所以断流也能被发现）。"""
        s = self.stats
        self.v_fps.set("%.1f fps" % s.fps)
        self.v_bad.set(str(bad))
        self.lbl_fps_top.configure(text="%.1f fps" % s.fps)

        if s.stale == self._was_stale:
            return
        was = self._was_stale
        self._was_stale = s.stale
        # 断流状态一翻转就要重画画布（上面盖着"数据过期"）
        self._dirty_canvas = True
        if s.stale:
            if s.last_recv is not None:
                self.log("断流：%.1f 秒没收到上行帧（相机待机？掉电？线松了？）"
                         % s.idle, "err")
            self.lbl_led.configure(fg=C_YELLOW)
        elif was is True:
            # 只有"确实是断过再恢复"才说恢复接收；刚连上不算
            self.log("恢复接收", "ok")
            self.lbl_led.configure(fg=C_GREEN)

    def _render(self, f):
        """把"最近一帧"刷到右栏。

        注意：这里每 tick 调一次（约 25Hz），**不是**每帧调一次 ——
        30 帧/秒时每帧都刷 10 个 Tk 变量纯属浪费，而且只有最后一帧的值留下来。
        """
        s = self.stats
        state = f["state"]
        exists = bool(f["exists"])
        # 没有目标时 color 字段是 0，显示成 "?" 只会让人以为识别出问题了
        color = f["color_name"] if exists else "—"

        self.v_state.set(P.STATE_NAMES.get(state, "?"))
        self.lbl_state.configure(fg=STATE_HEX.get(state, FG))
        self.v_color.set(color)
        self.lbl_color.configure(fg=COLOR_HEX.get(color, FG_DIM))
        self.v_seq.set(str(f["seq"]))
        self.v_loss.set("%d (%.2f%%)" % (s.lost, s.loss_pct))
        self.v_ack.set(self.last_ack or "—")

        if exists:
            cw, ch = self.cw, self.ch
            for k, n, p in (("x_n", f["x_n"], round(f["x_n"] / 255.0 * cw)),
                            ("y_n", f["y_n"], round(f["y_n"] / 255.0 * ch)),
                            ("w_n", f["w_n"], round(f["w_n"] / 255.0 * cw)),
                            ("h_n", f["h_n"], round(f["h_n"] / 255.0 * cw))):
                a, b = self.v_coord[k]
                a.set(str(n))
                b.set(str(p))
            self.v_delta.set("Δx %+d   Δy %+d   （中心 128,128）"
                             % (f["dx"], f["dy"]))
        else:
            # exists=0 时协议规定坐标字段全为 0。照原样显示会变成
            # "x 0 → 0，Δx -128" —— 看起来像"目标跑到左上角去了"，纯属误导。
            for k in ("x_n", "y_n", "w_n", "h_n"):
                a, b = self.v_coord[k]
                a.set("—")
                b.set("—")
            self.v_delta.set("—   （画面里没有目标）")

        self.lbl_hud.configure(
            text="seq %d   %s   %s" % (f["seq"], color,
                                       P.STATE_NAMES.get(state, "?")))

    # ------------------------------------------------------------ CSV

    def _write_csv(self, now, f):
        if self._rec is None:
            return
        try:
            self._rec.write("%.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n" % (
                now, f["seq"], f["exists"], f["color"], f["state"],
                f["x_n"], f["y_n"], f["w_n"], f["h_n"], f["dx"], f["dy"]))
            self._csv_rows += 1
        except OSError as e:
            self.log("写 CSV 失败：%s（已停止录制）" % e, "err")
            self.toggle_record()

    def _flush_csv(self):
        if self._rec is None:
            return
        try:
            self._rec.flush()      # 每秒刷一次：程序崩了也不至于一行都没有
        except OSError:
            pass

    def _on_ack(self, d):
        # 回执帧：cmd 最高位为 1，ack_of 是原命令
        ok = bool(d.get("demo_ok", d["arg"] == P.STATUS_OK))
        name = P.CMD_NAMES.get(d["ack_of"], "0x%02X" % d["ack_of"])
        text = "%s %s" % (name, "OK" if ok else "FAIL")
        if ok:
            self.ack_ok += 1
        else:
            self.ack_fail += 1
        # 带上累计计数：否则下一条命令一来就把上一条的结果盖掉了
        self.last_ack = "%s   [OK %d / FAIL %d]" % (
            text, self.ack_ok, self.ack_fail)
        self.v_ack.set(self.last_ack)
        if ok:
            self.log("<- 回执 %s" % text, "ok")
        else:
            self.log("<- 回执 %s（相机拒绝了这条命令）" % text, "err")

    # ------------------------------------------------------------ 绘制

    def _on_canvas_resize(self, ev):
        """画布跟着窗口变 —— 它是归一化坐标的显示区，铺满即可。"""
        if ev.width < 80 or ev.height < 80:
            return
        if (ev.width, ev.height) == (self.cw, self.ch):
            return
        self.cw, self.ch = ev.width, ev.height
        self._dirty_canvas = True

    def _draw_idle(self):
        c = self.canvas
        c.delete("all")
        c.create_text(self.cw / 2, self.ch / 2,
                      text="未连接\n选好串口后点「连接」，或加 --demo 看演示",
                      fill=FG_DIM, font=FONT_UI, justify="center")

    def _draw_overlay(self):
        """画在最上层的状态条：断流警告。

        为什么要有它：相机待机或掉电时，界面上最后一帧会一直停在那儿，
        看起来和"活着"一模一样 —— 现场最容易因此误判成"相机不识别了"。
        """
        if self.link is None:
            return
        s = self.stats
        if not s.stale:
            return
        c = self.canvas
        cw = self.cw
        if s.last_recv is None:
            text = "等待第一帧…"
        else:
            text = "数据过期  %.1f s" % s.idle
        w = 210
        x0 = (cw - w) / 2.0
        c.create_rectangle(x0, 10, x0 + w, 42, fill="#2a2415", outline=C_YELLOW)
        c.create_text(cw / 2, 26, text=text, fill=C_YELLOW,
                      font=("Consolas", 10, "bold"))

    def _redraw(self):
        c = self.canvas
        cw, ch = self.cw, self.ch
        c.delete("all")

        # 背景网格
        for i in range(1, 4):
            x = cw * i / 4.0
            y = ch * i / 4.0
            c.create_line(x, 0, x, ch, fill="#151a22")
            c.create_line(0, y, cw, y, fill="#151a22")

        # 中心十字
        cx, cy = cw / 2.0, ch / 2.0
        c.create_line(cx, 0, cx, ch, fill="#252b36", dash=(4, 6))
        c.create_line(0, cy, cw, cy, fill="#252b36", dash=(4, 6))
        c.create_text(cw - 8, ch - 10, text="中心 128,128", fill="#39414f",
                      font=("Consolas", 8), anchor="e")

        f = self.last
        if f is None:
            c.create_text(cw / 2, ch / 2, text="等待数据…", fill=FG_DIM,
                          font=FONT_UI)
            return

        # 轨迹
        if len(self._trail) > 1:
            pts = []
            for x_n, y_n in self._trail:
                pts.extend((x_n / 255.0 * cw, y_n / 255.0 * ch))
            c.create_line(*pts, fill="#2b3a4d", width=1, smooth=True)

        if not f["exists"]:
            # 这里不能写死"搜索中"：状态可能是"完成"，也可能是"对准目标区"时目标丢了
            c.create_text(cw / 2, ch / 2 - 8, text="画面里没有目标",
                          fill=FG_DIM, font=("Microsoft YaHei UI", 12))
            c.create_text(cw / 2, ch / 2 + 18,
                          text="状态：%s" % P.STATE_NAMES.get(f["state"], "?"),
                          fill="#4a5464", font=FONT_UI)
            return

        x, y, w, h = norm_box(f["x_n"], f["y_n"], f["w_n"], f["h_n"], cw, ch)
        col = COLOR_HEX.get(f["color_name"], FG)
        state = P.STATE_NAMES.get(f["state"], "?")

        # 框
        c.create_rectangle(x, y, x + w, y + h, outline=col, width=2)
        # 四角加粗，看得清
        arm = max(6, min(w, h) * 0.22)
        for (ax, ay, dx, dy) in ((x, y, 1, 1), (x + w, y, -1, 1),
                                 (x, y + h, 1, -1), (x + w, y + h, -1, -1)):
            c.create_line(ax, ay, ax + arm * dx, ay, fill=col, width=3)
            c.create_line(ax, ay, ax, ay + arm * dy, fill=col, width=3)

        # 中心点
        pcx = x + w / 2.0
        pcy = y + h / 2.0
        c.create_oval(pcx - 3, pcy - 3, pcx + 3, pcy + 3, fill=col, outline="")

        # 标签
        label = "%s · %s" % (f["color_name"].upper(), state)
        ty = y - 8 if y > 18 else y + h + 8
        c.create_text(x, ty, text=label, fill=col, anchor="w",
                      font=("Consolas", 10, "bold"))

        # 到中心的偏差箭头
        if abs(f["dx"]) > 6 or abs(f["dy"]) > 6:
            c.create_line(pcx, pcy, cx, cy, fill="#3d4a5c", arrow="last",
                          dash=(2, 4))

        # 右上角坐标
        c.create_text(cw - 8, 10,
                      text="x %d  y %d  w %d  h %d   Δ %+d %+d"
                      % (f["x_n"], f["y_n"], f["w_n"], f["h_n"],
                         f["dx"], f["dy"]),
                      fill="#5a6678", font=("Consolas", 8), anchor="ne")


# ================================================================ 入口

def main(argv=None):
    ap = argparse.ArgumentParser(description="智能搬运 · 上位机（图形界面）")
    ap.add_argument("--port", help="启动即连接的串口，如 COM5")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--demo", action="store_true",
                    help="演示模式：自己造数据，不需要相机和串口")
    args = ap.parse_args(argv)

    # 让高 DPI 屏上的字不糊
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except Exception:                                        # noqa: BLE001
        pass

    root = tk.Tk()
    style = ttk.Style(root)
    try:
        style.theme_use("clam")
    except Exception:                                        # noqa: BLE001
        pass
    # clam 主题下 Combobox 的只读态不吃 foreground，得用 map 单独指定，
    # 否则就是"白底浅字"——看起来像一个空框。
    style.configure("TCombobox",
                    fieldbackground=PANEL_HI, background=PANEL_HI,
                    foreground=FG, arrowcolor=FG, bordercolor=BORDER,
                    lightcolor=PANEL_HI, darkcolor=PANEL_HI)
    style.map("TCombobox",
              fieldbackground=[("readonly", PANEL_HI), ("disabled", PANEL)],
              foreground=[("readonly", FG)],
              selectbackground=[("readonly", PANEL_HI)],
              selectforeground=[("readonly", FG)],
              arrowcolor=[("readonly", FG)])
    root.option_add("*TCombobox*Listbox.background", PANEL_HI)
    root.option_add("*TCombobox*Listbox.foreground", FG)
    root.option_add("*TCombobox*Listbox.selectBackground", C_BLUE)
    root.option_add("*TCombobox*Listbox.selectForeground", "#0b0d11")
    app = GroundStation(root, args)
    app.log("上位机已启动。协议 v3.0：上行 %d 字节，下行 %d 字节。"
            % (P.UP_FRAME_LEN, P.DOWN_FRAME_LEN), "hint")
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
