"""
智能搬运 · 上位机（图形界面，v4 协议）
=====================================

给 MaixCAM 相机端配的调试/演示界面。一根 USB 转串口（CH340）接相机的
A19(TX)，就能看到相机现在盯着谁、目标在画面里的哪个位置、多大。

v4 与旧版（v3）的差别决定了这个界面长什么样：
    · 上行 20 字节：x/y 是目标**中心**像素，坐标系固定 640x480，
      所以画布按 640x480 等比缩放画框，而不再有"归一化坐标"这回事；
    · 第 6 字节 bit7 = 色环标志，低 7 位 = 角度，用 decode_rotation() 拆；
    · **没有帧序号** —— 丢帧统计从原理上就不成立，界面里不再显示；
    · **没有任务阶段**（可抓取/可放置这些是 v3 主控状态机的概念，v4 相机
      不负责，所以整块删掉）；
    · **没有回执帧** —— 命令发出去之后相机不会回 OK/FAIL，回执面板删掉；
    · 下行只有 4 字节：三个颜色按钮 + PING，其余命令（NEXT/ABORT/RUN/
      SAVE/CAL/SET_FPS）都不存在了。

它还能干的事：
    · 实时显示帧率 / 坏帧 / 有无目标 / 颜色 / 环还是块 / 中心 / 尺寸
    · 把中心坐标还原成画面框，实时画出来（按颜色上色，带轨迹）
    · 录制 CSV（离线复盘用）
    · 演示模式：没有相机也没有串口也能把界面跑起来
    · 断流检测：超过 1 秒没帧就在画布上盖"数据过期"横幅

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
LETTERBOX = "#07080b"

C_RED = "#ff5a5f"
C_GREEN = "#3ddc84"
C_BLUE = "#4da3ff"
C_YELLOW = "#f5c451"
C_CYAN = "#56d4dd"

COLOR_HEX = {"red": C_RED, "green": C_GREEN, "blue": C_BLUE, "?": FG_DIM}

# 形状的强调色：环用青色、块用蓝色，和颜色本身区分开（否则红环红块
# 只看颜色分不出来）
SHAPE_RING_HEX = C_CYAN
SHAPE_BLOCK_HEX = C_BLUE

FONT_UI = ("Microsoft YaHei UI", 9)
FONT_UI_B = ("Microsoft YaHei UI", 9, "bold")
FONT_BIG = ("Microsoft YaHei UI", 13, "bold")
FONT_MONO = ("Consolas", 9)
FONT_MONO_B = ("Consolas", 10, "bold")


def hexs(data):
    return " ".join("%02X" % b for b in data)


def shape_name(is_ring):
    return "色环" if is_ring else "色块"


# ================================================================ 帧统计

class FrameStats:
    """v4 上行帧统计（纯逻辑，不碰界面 —— 可以直接单测）。

    v4 没有帧序号，所以这里**没有**丢帧、丢帧率、重连次数：
    协议里根本推不出"应该收到多少帧"，任何 lost 数字都是编的。
    能可靠测量的只有三件事：

    1. **帧率用真时间窗**（最近 1 秒实际收到多少帧），而不是
       "总帧数 / 总时长" —— 后者在断流恢复后会虚高，骗人。
    2. **断流判定**：超过 stale_after 秒没有上行帧就置 stale。
       界面据此显示"数据过期"，而不是一直画着最后一帧装作还活着。
    3. **分类计数**：无目标 / 色环 / 色块，方便判断相机到底认到了什么。
    """

    def __init__(self, window=1.0, stale_after=1.0):
        self.window = window
        self.stale_after = stale_after
        self.reset()

    def reset(self):
        self.total = 0
        self.no_target = 0
        self.rings = 0
        self.blocks = 0
        self.fps = 0.0
        self.idle = 0.0          # 距上一帧多久
        self.stale = False
        self.last_recv = None
        self.last_exists = None
        self.last_color = None
        self.last_ring = None
        self._marks = deque()

    def on_frame(self, now, f):
        """喂一帧上行数据。返回 True 表示"值得记一行日志"的状态变了。"""
        self.last_recv = now
        self.total += 1
        if f["exists"]:
            if f["is_ring"]:
                self.rings += 1
            else:
                self.blocks += 1
        else:
            self.no_target += 1

        self._marks.append(now)
        while self._marks and now - self._marks[0] > self.window:
            self._marks.popleft()

        # 颜色每帧都相同，只有换了目标类型/颜色/有无才算"变化"，
        # 否则日志会被 30 帧/秒刷爆。
        changed = (bool(f["exists"]) != self.last_exists
                   or f["color_name"] != self.last_color
                   or bool(f["is_ring"]) != self.last_ring)
        self.last_exists = bool(f["exists"])
        self.last_color = f["color_name"]
        self.last_ring = bool(f["is_ring"])
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


# ================================================================ 数据源

class SerialLink:
    """真实串口：读线程负责收字节、解帧，主线程只管取。"""

    is_demo = False

    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.parser = P.UplinkParser()
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

DEMO_PERIOD = 12.0       # 一个完整演示循环的秒数
DEMO_FPS = 25.0          # 和界面刷新节奏（40ms）差不多，队列不会越积越多


def demo_pose(t):
    """按时间返回一帧要发的 v4 字段。

    坐标直接是 640x480 参考系的中心像素（v4 不再归一化），场景里目标绕
    画面中心走椭圆、尺寸周期性变化；开头 0.8 秒和中段各有一段没有目标，
    用来演示 exists=0 的显示；后半程当成色环并带旋转角度。
    """
    colors = ("red", "green", "blue")
    color = colors[int(t / 6.0) % 3]
    tt = t % DEMO_PERIOD

    # 两段"没有目标"：开头还没认到，中段目标离开视野
    if tt < 0.8 or 5.4 <= tt < 6.2:
        return 0, color, 0.0, 0, 0, 0, 0, False

    p = (tt - 0.8) / (DEMO_PERIOD - 0.8)
    ang = p * 2.0 * math.pi
    cx = P.REF_W / 2.0 + math.cos(ang) * 190.0
    cy = P.REF_H / 2.0 + math.sin(ang) * 130.0
    w = 95.0 + 35.0 * math.sin(p * 6.0 * math.pi)
    h = w * 0.78
    is_ring = tt >= 6.2
    angle = (t * 120.0) % 360.0 if is_ring else 0.0
    return (1, color, angle, int(round(cx)), int(round(cy)),
            int(round(w)), int(round(h)), is_ring)


class DemoLink:
    """演示数据源：没有相机也能把界面跑起来，接口和 SerialLink 一样。

    这里刻意走 P.build_uplink -> P.parse_uplink 的完整路径，而不是直接
    拼字典 —— 演示模式顺便就在验证组帧/解帧这条真实链路。
    """

    is_demo = True

    def __init__(self):
        self._t0 = 0.0
        self._next = 0.0
        self.error = None

    def open(self):
        self._t0 = time.time()
        self._next = self._t0

    def close(self):
        pass

    def send(self, data):
        # v4 没有回执：能按 4 字节下行解析出来就说明命令合法。
        # 返回 True 只是让界面记录"发出了"，不代表相机执行成功。
        return P.parse_downlink(data) is not None

    def drain(self):
        now = time.time()
        out = []
        # 界面卡住时不要一次补几百帧，补一小批就够了
        while now >= self._next and len(out) < 40:
            exists, color, angle, cx, cy, w, h, is_ring = demo_pose(
                now - self._t0)
            raw = P.build_uplink(
                exists, P.COLOR_VALUES[color],
                P.encode_rotation(angle, is_ring), cx, cy, w, h)
            out.append(P.parse_uplink(raw))
            self._next += 1.0 / DEMO_FPS
        if len(out) >= 40:
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
        self.stamp_bad = 0                   # 上一次坏帧快照（日志去重用）
        self._trail = deque(maxlen=24)       # 目标中心轨迹（参考系像素）
        self._t_last_stat = time.time()
        self._last_stat_total = -1
        self._rec = None
        self._rec_path = None
        self._csv_rows = 0
        self._was_stale = None
        self._port_map = {}

        root.title("智能搬运 · 上位机  |  MaixCAM 相机调试 (v4)")
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
        tk.Label(head, text="画面（固定 640×480 参考系，x/y 是目标中心）",
                 bg=PANEL, fg=FG_DIM, font=FONT_UI).pack(side="left", padx=10)
        self.lbl_hud = tk.Label(head, text="", bg=PANEL, fg=FG_DIM,
                                font=FONT_MONO)
        self.lbl_hud.pack(side="right", padx=10)

        self.cw, self.ch = 560, 420
        self.canvas = tk.Canvas(left, width=self.cw, height=self.ch,
                                bg=LETTERBOX, highlightthickness=0)
        # 画布跟着窗口长；里面按 640x480 等比缩放，留黑边而不是拉伸
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
        """目标信息卡片 —— 字段和 v4 帧一一对应，不多也不少。"""
        card = self._card(parent, "目标")

        self.v_exists = tk.StringVar(value="—")
        self.v_color = tk.StringVar(value="—")
        self.v_shape = tk.StringVar(value="—")
        self.v_fps = tk.StringVar(value="—")
        self.v_bad = tk.StringVar(value="—")
        self.v_center = tk.StringVar(value="—")
        self.v_size = tk.StringVar(value="—")
        self.v_delta = tk.StringVar(value="—")

        grid = tk.Frame(card, bg=PANEL)
        grid.pack(side="top", fill="x", padx=10, pady=(0, 8))
        grid.columnconfigure(1, weight=1)
        grid.columnconfigure(3, weight=1)

        def cell(r, c, name, var, font, width=5):
            tk.Label(grid, text=name, bg=PANEL, fg=FG_DIM, font=FONT_UI,
                     anchor="w", width=width).grid(row=r, column=c, sticky="w")
            lbl = tk.Label(grid, textvariable=var, bg=PANEL, fg=FG, font=font,
                           anchor="w")
            lbl.grid(row=r, column=c + 1, sticky="w", padx=(0, 12))
            return lbl

        self.lbl_exists = cell(0, 0, "目标", self.v_exists, FONT_BIG)
        self.lbl_exists.grid_configure(columnspan=3)
        self.lbl_color = cell(1, 0, "颜色", self.v_color, FONT_UI_B)
        self.lbl_color.grid_configure(columnspan=3)
        self.lbl_shape = cell(2, 0, "形状", self.v_shape, FONT_UI_B)
        self.lbl_shape.grid_configure(columnspan=3)
        cell(3, 0, "帧率", self.v_fps, FONT_MONO)
        cell(3, 2, "坏帧", self.v_bad, FONT_MONO)
        self.lbl_center = cell(4, 0, "中心", self.v_center, FONT_MONO)
        self.lbl_center.grid_configure(columnspan=3)
        self.lbl_size = cell(5, 0, "尺寸", self.v_size, FONT_MONO)
        self.lbl_size.grid_configure(columnspan=3)
        cell(6, 0, "偏差", self.v_delta, FONT_MONO).grid_configure(columnspan=3)

    def _build_controls(self, parent):
        card = self._card(parent, "控制")

        row1 = tk.Frame(card, bg=PANEL)
        row1.pack(side="top", fill="x", padx=10)
        tk.Label(row1, text="选色", bg=PANEL, fg=FG_DIM,
                 font=FONT_UI).pack(side="left")
        for name, code, cname in (("红", P.CMD_RED, "red"),
                                  ("绿", P.CMD_GREEN, "green"),
                                  ("蓝", P.CMD_BLUE, "blue")):
            b = self._btn(row1, name,
                          lambda c=code, n=cname: self.cmd(
                              c, "TRACK_" + n.upper()),
                          wide=3)
            b.configure(fg=COLOR_HEX[cname])
            b.pack(side="left", padx=(6, 0))
        self._btn(row1, "PING", lambda: self.cmd(P.CMD_PING, "PING"),
                  wide=6).pack(side="right")

        row2 = tk.Frame(card, bg=PANEL)
        row2.pack(side="top", fill="x", padx=10, pady=(8, 4))
        self.btn_rec = self._btn(row2, "录制 CSV", self.toggle_record, wide=10)
        self.btn_rec.pack(side="left")
        self._btn(row2, "清空日志", self.clear_log, wide=9).pack(side="left",
                                                                padx=6)

        tip = ("v4 下行只有 4 字节：选色 / PING。相机记住最后一次选色，"
               "一直按这个颜色识别；没有回执，所以日志里只记录本机发出的命令。")
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
        port = self._port_map.get(label)
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
            self.log("演示模式：数据是程序用 P.build_uplink 造的，没接任何硬件。",
                     "hint")
            self.log("目标会绕画面移动、换颜色、换环/块，偶尔消失。", "hint")
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
        for var in (self.v_exists, self.v_color, self.v_shape, self.v_center,
                    self.v_size, self.v_delta, self.v_fps, self.v_bad):
            var.set("—")
        self.lbl_exists.configure(fg=FG_DIM)
        self.lbl_color.configure(fg=FG_DIM)
        self.lbl_shape.configure(fg=FG_DIM)
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

    def cmd(self, code, name):
        """发一条 v4 下行（4 字节）。没有回执，发出去就结束。"""
        if self.link is None:
            self.log("还没连接，命令没发出去", "err")
            return
        frame = P.build_downlink(code)
        try:
            ok = self.link.send(frame)
        except Exception as e:                               # noqa: BLE001
            self.log("发送失败：%s" % e, "err")
            return
        if not ok:
            self.log("发送失败：链路未就绪", "err")
            return
        self.log("-> %-11s %s" % (name, hexs(frame)), "tx")

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
            # 字段按 v4 帧来：没有序号、没有 state，多了环/块标志和角度
            self._rec.write("t,exists,color,shape,angle_deg,cx,cy,w,h\n")
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
            # 有无目标 / 颜色 / 环块 变了才记日志，否则日志会被 30 帧/秒刷爆
            if self.stats.on_frame(now, f):
                if f["exists"]:
                    self.log("目标 %s · %s" % (f["color_name"],
                                              shape_name(f["is_ring"])), "rx")
                else:
                    self.log("画面里没有目标（当前追踪色 %s）"
                             % f["color_name"], "info")

            if f["exists"]:
                self._trail.append((f["cx"], f["cy"]))
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
            self.log("界面卡顿，跳过 %d 帧" % dropped, "info")

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
                self.log("统计：累计 %d 帧，无目标 %d，色环 %d，色块 %d，坏帧 %d"
                         % (s.total, s.no_target, s.rings, s.blocks, bad))
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
                self.log("断流：%.1f 秒没收到上行帧（相机掉电？线松了？）"
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
        exists = bool(f["exists"])
        color = f["color_name"]

        if exists:
            self.v_exists.set("有目标")
            self.lbl_exists.configure(fg=C_GREEN)
            self.lbl_color.configure(fg=COLOR_HEX.get(color, FG))
            self.v_color.set(color)
            if f["is_ring"]:
                self.v_shape.set("色环  %.1f°" % f["angle_deg"])
                self.lbl_shape.configure(fg=SHAPE_RING_HEX)
            else:
                self.v_shape.set("色块")
                self.lbl_shape.configure(fg=SHAPE_BLOCK_HEX)
            self.v_center.set("(%d, %d) px" % (f["cx"], f["cy"]))
            self.v_size.set("%d × %d px" % (f["w"], f["h"]))
            self.v_delta.set("Δx %+d   Δy %+d   （中心 320,240）"
                             % (f["cx"] - P.REF_W // 2,
                                f["cy"] - P.REF_H // 2))
            self.lbl_hud.configure(
                text="%s · %s · (%d, %d)"
                % (color, shape_name(f["is_ring"]), f["cx"], f["cy"]))
        else:
            # exists=0 时协议规定坐标字段全为 0。照原样显示会变成
            # "中心 (0,0)，Δx -320" —— 看起来像目标跑到左上角去了，纯属误导。
            # 颜色字段仍然是有效的（相机记得当前选色），所以保留但压暗。
            self.v_exists.set("无目标")
            self.lbl_exists.configure(fg=FG_DIM)
            self.lbl_color.configure(fg=FG_DIM)
            self.v_color.set(color)
            self.v_shape.set("—")
            self.lbl_shape.configure(fg=FG_DIM)
            self.v_center.set("—")
            self.v_size.set("—")
            self.v_delta.set("—   （画面里没有目标）")
            self.lbl_hud.configure(text="无目标 · 追踪色 %s" % color)

    # ------------------------------------------------------------ CSV

    def _write_csv(self, now, f):
        if self._rec is None:
            return
        # 无目标时 is_ring 没有意义（协议里坐标/形状字段都是 0），
        # 照实写 none，别让离线复盘把"没目标"误当成"色块"。
        if not f["exists"]:
            shape = "none"
        else:
            shape = "ring" if f["is_ring"] else "block"
        try:
            self._rec.write("%.4f,%d,%s,%s,%.1f,%d,%d,%d,%d\n" % (
                now, f["exists"], f["color_name"], shape,
                f["angle_deg"], f["cx"], f["cy"], f["w"], f["h"]))
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

    # ------------------------------------------------------------ 绘制

    def _on_canvas_resize(self, ev):
        """画布跟着窗口变 —— 640x480 参考系等比缩放到里面。"""
        if ev.width < 80 or ev.height < 80:
            return
        if (ev.width, ev.height) == (self.cw, self.ch):
            return
        self.cw, self.ch = ev.width, ev.height
        self._dirty_canvas = True

    def _view(self):
        """把固定的 640x480 参考系等比铺到画布中央，返回缩放和偏移。

        为什么不直接按画布宽高拉伸：画布不是 4:3 时目标框会变形，
        测出来的 w/h 比例就不可信了。留黑边比画错强。
        """
        cw, ch = self.cw, self.ch
        scale = min(cw / float(P.REF_W), ch / float(P.REF_H))
        vw, vh = P.REF_W * scale, P.REF_H * scale
        return scale, (cw - vw) / 2.0, (ch - vh) / 2.0, vw, vh

    def _draw_idle(self):
        c = self.canvas
        c.delete("all")
        c.create_text(self.cw / 2, self.ch / 2,
                      text="未连接\n选好串口后点「连接」，或加 --demo 看演示",
                      fill=FG_DIM, font=FONT_UI, justify="center")

    def _draw_overlay(self):
        """画在最上层的状态条：断流警告。

        为什么要有它：相机掉电或线松了时，界面上最后一帧会一直停在那儿，
        看起来和"活着"一模一样 —— 现场最容易因此误判成"相机不识别了"。
        """
        if self.link is None:
            return
        s = self.stats
        if not s.stale:
            return
        c = self.canvas
        scale, ox, oy, vw, vh = self._view()
        if s.last_recv is None:
            text = "等待第一帧…"
        else:
            text = "数据过期  %.1f s" % s.idle
        w = 210
        x0 = ox + vw / 2.0 - w / 2.0
        c.create_rectangle(x0, oy + 10, x0 + w, oy + 42,
                           fill="#2a2415", outline=C_YELLOW)
        c.create_text(ox + vw / 2.0, oy + 26, text=text, fill=C_YELLOW,
                      font=("Consolas", 10, "bold"))

    def _redraw(self):
        c = self.canvas
        c.delete("all")

        scale, ox, oy, vw, vh = self._view()
        # 有效画面（640x480）之外是黑边，先画出来，让人知道哪块是真的
        c.create_rectangle(ox, oy, ox + vw, oy + vh, fill=CANVAS_BG,
                           outline="#232b38")

        # 背景网格：按参考系的 1/4 等分，不是按画布像素
        for i in range(1, 4):
            x = ox + vw * i / 4.0
            y = oy + vh * i / 4.0
            c.create_line(x, oy, x, oy + vh, fill="#151a22")
            c.create_line(ox, y, ox + vw, y, fill="#151a22")

        # 参考系中心十字（320,240），不是画布中心
        cx = ox + vw / 2.0
        cy = oy + vh / 2.0
        c.create_line(cx, oy, cx, oy + vh, fill="#252b36", dash=(4, 6))
        c.create_line(ox, cy, ox + vw, cy, fill="#252b36", dash=(4, 6))
        c.create_text(ox + 6, oy + 6, text="640×480 参考系", fill="#39414f",
                      font=("Consolas", 8), anchor="nw")
        c.create_text(ox + vw - 6, oy + vh - 6, text="中心 320,240",
                      fill="#39414f", font=("Consolas", 8), anchor="se")

        f = self.last
        if f is None:
            if self.link is None:
                c.create_text(cx, cy,
                              text="未连接\n选好串口后点「连接」，或加 --demo 看演示",
                              fill=FG_DIM, font=FONT_UI, justify="center")
            else:
                c.create_text(cx, cy, text="等待数据…", fill=FG_DIM,
                              font=FONT_UI)
            return

        # 轨迹（存的是参考系像素，画的时候才乘 scale）
        if len(self._trail) > 1:
            pts = []
            for tx, ty in self._trail:
                pts.extend((ox + tx * scale, oy + ty * scale))
            c.create_line(*pts, fill="#2b3a4d", width=1, smooth=True)

        if not f["exists"]:
            c.create_text(cx, cy - 8, text="画面里没有目标", fill=FG_DIM,
                          font=("Microsoft YaHei UI", 12))
            c.create_text(cx, cy + 18,
                          text="当前追踪色：%s" % f["color_name"],
                          fill="#4a5464", font=FONT_UI)
            return

        # v4 的 x/y 是**中心**：左上角要自己减半个宽高
        bx = ox + (f["cx"] - f["w"] / 2.0) * scale
        by = oy + (f["cy"] - f["h"] / 2.0) * scale
        bw = f["w"] * scale
        bh = f["h"] * scale
        col = COLOR_HEX.get(f["color_name"], FG)

        # 框
        c.create_rectangle(bx, by, bx + bw, by + bh, outline=col, width=2)
        # 四角加粗，看得清
        arm = min(18.0, max(5.0, min(bw, bh) * 0.22))
        for (ax, ay, dx, dy) in ((bx, by, 1, 1), (bx + bw, by, -1, 1),
                                 (bx, by + bh, 1, -1),
                                 (bx + bw, by + bh, -1, -1)):
            c.create_line(ax, ay, ax + arm * dx, ay, fill=col, width=3)
            c.create_line(ax, ay, ax, ay + arm * dy, fill=col, width=3)

        # 中心点（就是协议里的 cx,cy）
        pcx = ox + f["cx"] * scale
        pcy = oy + f["cy"] * scale
        c.create_oval(pcx - 3, pcy - 3, pcx + 3, pcy + 3, fill=col, outline="")

        # 标签：颜色 + 环/块（环再带角度，块的角度无意义就不显示）
        label = "%s · %s" % (f["color_name"].upper(), shape_name(f["is_ring"]))
        if f["is_ring"]:
            label += " %.0f°" % f["angle_deg"]
        ty = by - 8 if by > oy + 18 else by + bh + 8
        c.create_text(bx, ty, text=label, fill=col, anchor="w",
                      font=("Consolas", 10, "bold"))

        # 到画面中心的偏差箭头
        if abs(f["cx"] - P.REF_W / 2) > 6 or abs(f["cy"] - P.REF_H / 2) > 6:
            c.create_line(pcx, pcy, cx, cy, fill="#3d4a5c", arrow="last",
                          dash=(2, 4))

        # 右上角原始坐标（都是参考系像素）
        c.create_text(ox + vw - 8, oy + 10,
                      text="中心 %d,%d  尺寸 %d×%d   Δ %+d %+d"
                      % (f["cx"], f["cy"], f["w"], f["h"],
                         f["cx"] - P.REF_W // 2, f["cy"] - P.REF_H // 2),
                      fill="#5a6678", font=("Consolas", 8), anchor="ne")


# ================================================================ 入口

def main(argv=None):
    ap = argparse.ArgumentParser(description="智能搬运 · 上位机（图形界面，v4）")
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
    app.log("上位机已启动。协议 v4：上行 %d 字节（640×480 中心坐标），"
            "下行 %d 字节。" % (P.UP_FRAME_LEN, P.DOWN_FRAME_LEN), "hint")
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
