"""
智能搬运 · 上位机（Material You / Android 17 风格，v4 协议）
=========================================================

界面按 Material Design 3（Material You）的深色方案重画：
    · 圆角卡片（28dp 大圆角、12dp 小圆角），靠"容器色阶"分层，不画边框
    · 药丸按钮：填充式（filled）、色调式（tonal）、图标式
    · 分段按钮（segmented）：识别颜色 红/绿/蓝 一组
    · 排版按 M3 type scale 放大：标题 32px、正文 16px、标签 14px

跨屏幕自适应是这次重写的重点：
    · 所有尺寸都不是写死的像素，而是 `K(基准像素)` —— K 由 **DPI × 屏幕尺寸**
      共同算出，换一台电脑自动适配，不会出现"字小得看不见"或"窗口大到装不下"
    · 字体一律用**负数字号**（Tk 里负号 = 像素），这样字体大小和布局像素
      走同一个 K，不会被 Tk 自己的点→像素换算再放大一次（那是最容易
      在高分屏上炸掉的地方）
    · 窗口初始大小也按屏幕比例取，最小尺寸同理，最大化/缩放时布局跟着重排

数据层（串口收发、帧统计、演示数据）从旧版原样保留 —— 那部分已经验证过，
这次只动界面。

用法：
    python ground_station.py                 打开界面，左下角点串口名切换
    python ground_station.py --port COM5     启动即连接
    python ground_station.py --demo          演示模式（不需要任何硬件）

依赖：
    pip install pyserial        # 只有真实串口需要；演示模式不需要

接线：
    相机 A19(TX) -> CH340 RX，GND 共地。
    主控也在线上时只并 RX（不要并 TX，否则会和主控同时往相机发命令）。
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
except Exception:                                            # noqa: BLE001
    print("这个工具需要 tkinter（Python 自带）。如果是精简版 Python，装完整版即可。")
    raise

try:
    import serial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except Exception:                                            # noqa: BLE001
    serial = None
    list_ports = None
    HAVE_SERIAL = False


# ================================================================ 主题（Material 3 Dark）

SURFACE = "#131316"            # 窗口背景
SURFACE_1 = "#1D1B20"          # 卡片（container-low）
SURFACE_2 = "#232028"          # 卡片（container）
SURFACE_3 = "#2B2930"          # 按钮容器（container-high）
SURFACE_4 = "#36343B"          # 悬停
SURFACE_5 = "#45414A"          # 按下

ON_SURFACE = "#E7E0E8"         # 主文字
ON_SURFACE_VAR = "#CAC4D0"     # 次文字
ON_SURFACE_DIM = "#938F99"     # 弱文字

PRIMARY = "#D0BCFF"            # 主色（Material You 紫）
ON_PRIMARY = "#381E72"
PRIMARY_CONT = "#4F378B"

SECONDARY = "#CCC2DC"
TERTIARY = "#EFB8C8"
ERROR = "#F2B8B5"
ERROR_CONT = "#8C1D18"

CANVAS_BG = "#0C0C0F"          # 画面区底色
LETTERBOX = "#050507"          # 画面留黑边

# 目标颜色 —— 用 M3 的柔和色阶，别用刺眼的纯色
C_RED = "#FF8A8F"
C_GREEN = "#7EDCA2"
C_BLUE = "#9CCAFF"
C_YELLOW = "#EFC06B"
C_CYAN = "#7EDCE0"

COLOR_HEX = {"red": C_RED, "green": C_GREEN, "blue": C_BLUE, "?": ON_SURFACE_DIM}
COLOR_NAME_CN = {"red": "红", "green": "绿", "blue": "蓝", "?": "—"}

# 识别颜色按钮用"低饱和填充 + 高饱和文字"，避免整块荧光色
COLOR_CONT = {"red": "#4A2226", "green": "#1F3A2A", "blue": "#1E3350"}


def hexs(data):
    return " ".join("%02X" % b for b in data)


def shape_name(is_ring):
    return "色环" if is_ring else "色块"


# ================================================================ 自适应缩放

class Metrics:
    """把"设计稿像素"换算成当前屏幕上的实际像素。

    为什么不用 Tk 自带的 tk scaling：那套是按 DPI 调"点"，而我们的布局是按
    像素写的，两套混用会在 125%/150% 缩放的机器上出现"卡片放大了但字没放大"
    或者反过来。这里统一成一个 K：
        K = (DPI / 96) * clamp(min(屏宽/1920, 屏高/1080), 0.78, 1.15)
    字体用负数字号（Tk 里负号表示"像素"），所以字体和布局走同一个 K。
    """

    def __init__(self, root):
        try:
            dpi = float(root.winfo_fpixels("1i"))
        except Exception:                                    # noqa: BLE001
            dpi = 96.0
        self.dpi = dpi
        self.sw = root.winfo_screenwidth()
        self.sh = root.winfo_screenheight()
        by_dpi = dpi / 96.0
        by_screen = min(self.sw / 1920.0, self.sh / 1080.0)
        by_screen = max(0.78, min(1.15, by_screen))
        self.k = max(0.80, min(2.20, by_dpi * by_screen))

    def __call__(self, v):
        """设计稿像素 -> 实际像素。"""
        return int(round(v * self.k))

    def font(self, px, bold=False, mono=False):
        fam = "Consolas" if mono else "Microsoft YaHei UI"
        size = max(9, int(round(px * self.k)))
        # 负号 = 像素单位，避开 Tk 的点→像素二次换算
        return (fam, -size, "bold" if bold else "normal")


# ================================================================ 画布小工具

def rrect(cv, x, y, w, h, r, fill):
    """圆角矩形。tkinter 没有现成的，用四个角弧 + 两块矩形拼。"""
    if w <= 0 or h <= 0:
        return
    r = int(min(r, w / 2, h / 2))
    if r <= 1:
        cv.create_rectangle(x, y, x + w, y + h, fill=fill, outline=fill)
        return
    x2, y2 = x + w, y + h
    cv.create_rectangle(x + r, y, x2 - r, y2, fill=fill, outline=fill)
    cv.create_rectangle(x, y + r, x2, y2 - r, fill=fill, outline=fill)
    for cx, cy, start in ((x + r, y + r, 90), (x2 - r, y + r, 0),
                          (x2 - r, y2 - r, 270), (x + r, y2 - r, 180)):
        cv.create_arc(cx - r, cy - r, cx + r, cy + r, start=start,
                      extent=90, fill=fill, outline=fill)


def pill(cv, x, y, w, h, fill):
    rrect(cv, x, y, w, h, h // 2, fill)


# ================================================================ 帧统计

class FrameStats:
    """v4 上行帧统计（纯逻辑，不碰界面）。

    v4 没有帧序号，所以：
        · 算不了丢帧，只能按"最近 1 秒收到多少帧"算帧率；
        · 超过 1 秒没有帧就判"断流"，界面据此显示数据过期，
          而不是一直画着最后一帧装作还活着。
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
        self.idle = 0.0
        self.stale = False
        self.last_recv = None
        self._marks = deque()

    def on_frame(self, now, f):
        self.total += 1
        if f["exists"]:
            if f.get("is_ring"):
                self.rings += 1
            else:
                self.blocks += 1
        else:
            self.no_target += 1
        self.last_recv = now
        self._marks.append(now)
        while self._marks and now - self._marks[0] > self.window:
            self._marks.popleft()

    def tick(self, now):
        if self.last_recv is None:
            self.fps = 0.0
            self.idle = 0.0
            self.stale = True
            return
        while self._marks and now - self._marks[0] > self.window:
            self._marks.popleft()
        self.idle = max(0.0, now - self.last_recv)
        self.stale = self.idle > self.stale_after
        self.fps = 0.0 if self.stale else float(len(self._marks)) / self.window


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
        # 原始字节环形缓冲：线上到底来了什么，原样留着。
        # 波特率不对时这里就是一堆乱码，正好用来定位问题。
        self._raw = bytearray()
        self._raw_lock = threading.Lock()

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
                with self._raw_lock:
                    self._raw.extend(data)
                    if len(self._raw) > 256:
                        del self._raw[: len(self._raw) - 256]
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

    def raw_snapshot(self):
        """最近收到的原始字节（最多 256 个），给「原始帧」视图用。"""
        with self._raw_lock:
            return bytes(self._raw)

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

    def raw_snapshot(self):
        return b""

    @property
    def bad_frames(self):
        return 0


# ================================================================ 界面




# ================================================================ 界面

class GroundStation:

    def __init__(self, root, args):
        self.root = root
        self.args = args
        self.link = None
        self.last = None
        self.stats = FrameStats()
        self._trail = deque(maxlen=30)
        self._t_last_stat = time.time()
        self._last_stat_total = -1
        self._rec = None
        self._rec_path = None
        self._csv_rows = 0
        self._was_stale = None
        self.baud = int(getattr(args, "baud", 115200) or 115200)
        self.raw_mode = False
        self._raw_t = 0.0
        self._ports = []
        self._port_idx = 0
        self._hover = None
        self._pressed = None
        self._buttons = []          # [(x, y, w, h, action)]
        self._log_lines = deque(maxlen=400)
        self._dirty = True

        # ---- 自适应度量
        self.m = Metrics(root)
        K = self.m
        # 窗口别无限放大：高分屏上按 K 算出来的尺寸可能超过屏幕，
        # 所以再压一道"物理像素上限"，保证 4K 屏上也不会占满整屏。
        # 最小高度不能拍脑袋写死：右侧三张卡 + 顶部栏 + 至少几行日志
        # 加起来才是"不溢出"的下限。屏幕小的时候 K 本身就小，所以这个下限
        # 也会跟着变小，1366x768 的笔记本同样放得下。
        need_h = self.sidebar_need(K) + K(14) * 2 + K(58) + K(14) + K(96)
        min_w = K(900)
        min_h = min(need_h, int(K.sh * 0.92))
        w = min(int(K.sw * 0.92), K(1380), 1900)
        h = min(int(K.sh * 0.90), K(860), 1200)
        h = max(h, min_h)
        w = max(w, min_w)
        x = max(0, (K.sw - w) // 2)
        y = max(0, (K.sh - h) // 3)

        root.title("智能搬运 · 上位机  |  MaixCAM 视觉 (v4)")
        root.configure(bg=SURFACE)
        root.geometry("%dx%d+%d+%d" % (w, h, x, y))
        root.minsize(min(min_w, int(K.sw * 0.9)), min(min_h, int(K.sh * 0.92)))
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ---- 画面区（自绘）
        self.cv = tk.Canvas(root, bg=SURFACE, highlightthickness=0, bd=0)
        self.cv.pack(side="top", fill="both", expand=True)
        self.cv.bind("<Configure>", self._on_resize)
        self.cv.bind("<Button-1>", self._on_press)
        self.cv.bind("<ButtonRelease-1>", self._on_release)
        self.cv.bind("<Motion>", self._on_motion)
        self.cv.bind("<Leave>", lambda e: self._set_hover(None))

        # ---- 日志（用 Text，但做成"卡片里的等宽文本"）
        self._build_log()
        self._refresh_ports()

        self.cw, self.ch = w, h - K(150)
        self.root.after(40, self._tick)

        if args.demo:
            self.start_link(DemoLink())
        elif args.port:
            self.connect(args.port, self.baud)

    # ------------------------------------------------------------ 日志

    def _build_log(self):
        K = self.m
        outer = tk.Frame(self.root, bg=SURFACE, height=K(150))
        outer.pack(side="top", fill="x", padx=K(16), pady=(0, K(14)))
        outer.pack_propagate(False)
        self.log_outer = outer
        card = tk.Frame(outer, bg=SURFACE_1)
        card.pack(fill="x")
        inner = tk.Frame(card, bg=SURFACE_1)
        inner.pack(fill="both", expand=True, padx=K(14), pady=K(10))
        self.log_text = tk.Text(
            inner, height=6, bg=SURFACE_1, fg=ON_SURFACE_VAR,
            font=self.m.font(13, mono=True), relief="flat", bd=0,
            insertbackground=ON_SURFACE, wrap="none", highlightthickness=0,
            spacing1=K(1), spacing3=K(1))
        sb = tk.Scrollbar(inner, command=self.log_text.yview, width=K(12),
                          bg=SURFACE_2, troughcolor=SURFACE_1,
                          activebackground=SURFACE_4, relief="flat", bd=0)
        self.log_text.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log_text.pack(side="left", fill="both", expand=True)
        for tag, col in (("info", ON_SURFACE_DIM), ("tx", C_YELLOW),
                         ("ok", C_GREEN), ("err", C_RED), ("rx", C_CYAN),
                         ("hint", SECONDARY), ("raw", C_GREEN),
                         ("rawtail", "#6E6A75")):
            self.log_text.tag_configure(tag, foreground=col)
        self.log_text.configure(state="disabled")

    def log(self, text, tag="info"):
        if not hasattr(self, "log_text"):
            return
        follow = self.log_text.yview()[1] >= 0.999
        self.log_text.configure(state="normal")
        self.log_text.insert("end", "%s  %s\n" % (time.strftime("%H:%M:%S"), text), tag)
        n = int(self.log_text.index("end-1c").split(".")[0])
        if n > 600:
            self.log_text.delete("1.0", "%d.0" % (n - 600))
        if follow:
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    # ------------------------------------------------------------ 端口 / 连接

    def _refresh_ports(self):
        if not HAVE_SERIAL:
            self._ports = []
            self.log("没找到 pyserial，串口模式不可用。安装：pip install pyserial", "err")
            return
        try:
            ports = list(list_ports.comports())
        except Exception:                                    # noqa: BLE001
            ports = []
        self._ports = [(p.device, p.description or "") for p in ports]
        if self._ports:
            self._port_idx = 0
            self.log("发现串口：" + ", ".join(p[0] for p in self._ports))
        else:
            self.log("没有发现串口。检查 CH340 驱动 / USB 线 / 相机是否上电。", "err")

    @property
    def _cur_port(self):
        if not self._ports:
            return None
        return self._ports[self._port_idx % len(self._ports)][0]

    def _cycle_port(self):
        if not self._ports:
            self._refresh_ports()
            return
        self._port_idx = (self._port_idx + 1) % len(self._ports)
        self.log("切换串口 -> %s" % self._cur_port)

    def toggle_connect(self):
        if self.link is not None:
            self.disconnect()
            return
        if self.args.demo:
            self.start_link(DemoLink())
            return
        port = self._cur_port
        if not port:
            self.log("没有可用串口（点一下串口名旁边的 ↻ 重新扫描）", "err")
            self._refresh_ports()
            return
        self.connect(port, self.baud)

    def connect(self, port, baud):
        self.start_link(SerialLink(port, baud))

    def start_link(self, link):
        self.link = link
        try:
            link.open()
        except Exception as e:                               # noqa: BLE001
            self.link = None
            self.log("打开 %s 失败：%s" % (getattr(link, "port", "数据源"), e), "err")
            return
        self.stats.reset()
        self._trail.clear()
        self._was_stale = None
        self._last_stat_total = -1
        self.last = None
        if link.is_demo:
            self.log("演示模式：数据是程序自己造的，没接任何硬件。", "hint")
        else:
            self.log("已连接 %s @ %d 8N1" % (getattr(link, "port", "?"), self.baud), "ok")
            self.log("一直没数据的话：查 A19 是否接到 RX、GND 是否共地、波特率 115200。", "hint")
        self._dirty = True

    def disconnect(self):
        if self.link is not None:
            try:
                self.link.close()
            except Exception:                                # noqa: BLE001
                pass
            self.link = None
        self.last = None
        self._trail.clear()
        self.stats.reset()
        self._was_stale = None
        self.log("已断开")
        self._dirty = True

    def on_close(self):
        self.disconnect()
        if self._rec:
            try:
                self._rec.close()
            except Exception:                                # noqa: BLE001
                pass
        self.root.destroy()

    # ------------------------------------------------------------ 命令 / CSV

    def send_color(self, color_name):
        cmd = {"red": P.CMD_RED, "green": P.CMD_GREEN, "blue": P.CMD_BLUE}[color_name]
        self.cmd(cmd, "识别%s" % COLOR_NAME_CN[color_name])

    def cmd(self, code, name):
        if self.link is None:
            self.log("还没连接，命令没发出去", "err")
            return
        frame = P.build_downlink(code)
        try:
            self.link.send(frame)
        except Exception as e:                               # noqa: BLE001
            self.log("发送失败：%s" % e, "err")
            return
        self.log("-> %s  %s" % (name, hexs(frame)), "tx")

    def _ask_baud(self):
        """弹窗输入任意波特率；正连着就按新速率重连。"""
        from tkinter import simpledialog
        try:
            v = simpledialog.askinteger(
                "设置波特率",
                "输入波特率（常用 115200 / 921600 / 9600，范围 300~4000000）",
                initialvalue=self.baud, minvalue=300, maxvalue=4000000,
                parent=self.root)
        except Exception:                                    # noqa: BLE001
            return
        if not v or int(v) == self.baud:
            return
        self.baud = int(v)
        if self.link is not None and not getattr(self.link, "is_demo", False):
            port = getattr(self.link, "port", None)
            self.disconnect()
            if port:
                self.connect(port, self.baud)
            self.log("波特率 -> %d，已按新速率重连" % self.baud, "ok")
        else:
            self.log("波特率 -> %d（下次连接生效）" % self.baud)

    def toggle_record(self):
        if self._rec:
            try:
                self._rec.close()
            except Exception:                                # noqa: BLE001
                pass
            self._rec = None
            self.log("已停止录制：%s（%d 行）" % (self._rec_path, self._csv_rows), "ok")
            return
        os.makedirs("captures", exist_ok=True)
        path = os.path.join("captures", "gs-%s.csv" % time.strftime("%Y%m%d-%H%M%S"))
        try:
            self._rec = open(path, "w", encoding="utf-8-sig", newline="")
            self._rec.write("t,exists,color,shape,angle_deg,cx,cy,w,h\n")
        except OSError as e:
            self.log("建不了 CSV：%s" % e, "err")
            self._rec = None
            return
        self._rec_path = path
        self._csv_rows = 0
        self.log("开始录制：%s" % path, "ok")

    # ------------------------------------------------------------ 主循环

    def _tick(self):
        try:
            self._poll()
            if self._dirty:
                self._draw()
                self._dirty = False
        except Exception as e:                               # noqa: BLE001
            self.log("界面异常：%s" % e, "err")
        self.root.after(60, self._tick)

    def _poll(self):
        now = time.time()
        if self.link is not None and getattr(self.link, "error", None):
            self.log("串口出错：%s" % self.link.error, "err")
            self.link.error = None
            self.disconnect()

        if self.link is not None:
            frames = self.link.drain()
            if len(frames) > 400:
                frames = frames[-400:]
            for f in frames:
                self.stats.on_frame(now, f)
                self.last = f
                if f["exists"]:
                    self._trail.append((f["cx"], f["cy"]))
                self._write_csv(now, f)
                self._dirty = True
                if self.raw_mode:
                    raw = f.get("raw") or b""
                    self.log("帧 %3d B  %s" % (len(raw), hexs(raw)), "raw")
            if self.raw_mode and self.link is not None and now - self._raw_t > 0.5:
                # 每 0.5 秒把「线上原始字节」也吐一次：坏帧、噪声、波特率不对
                # 造成的乱码都在这里，而不是被解析器丢掉。
                self._raw_t = now
                tail = self.link.raw_snapshot()
                if tail:
                    self.log("原始 %3d B  ...%s" % (len(tail), hexs(tail[-32:])),
                             "rawtail")

            bad = self.link.bad_frames
            if bad and not getattr(self, "_bad_logged", False):
                self.log("出现坏帧（校验和/帧尾不对），检查接线和波特率", "err")
                self._bad_logged = bad
            if not bad:
                self._bad_logged = 0

        self.stats.tick(now)

        # 断流状态翻转
        if self.stats.stale != self._was_stale:
            self._was_stale = self.stats.stale
            if self.stats.stale and self.link is not None and self.stats.last_recv:
                self.log("断流：%.1f 秒没收到帧" % self.stats.idle, "err")
            self._dirty = True

        if now - self._t_last_stat >= 1.0:
            self._t_last_stat = now
            s = self.stats
            if not self.raw_mode and s.total != self._last_stat_total:
                self._last_stat_total = s.total
                self.log("累计 %d 帧｜无目标 %d｜环 %d｜块 %d｜%.1f fps"
                         % (s.total, s.no_target, s.rings, s.blocks, s.fps))
            self._flush_csv()
            self._dirty = True

    def _write_csv(self, now, f):
        if self._rec is None:
            return
        try:
            self._rec.write("%.4f,%d,%s,%s,%.1f,%d,%d,%d,%d\n" % (
                now, f["exists"], f["color_name"],
                "ring" if f.get("is_ring") else "block",
                f.get("angle_deg", 0.0), f["cx"], f["cy"], f["w"], f["h"]))
            self._csv_rows += 1
        except OSError as e:
            self.log("写 CSV 失败：%s（已停止录制）" % e, "err")
            self.toggle_record()

    def _flush_csv(self):
        if self._rec is not None:
            try:
                self._rec.flush()
            except OSError:
                pass

    # ------------------------------------------------------------ 交互

    def _on_resize(self, ev):
        if ev.width < 100 or ev.height < 100:
            return
        self._fit_log()
        self.cw, self.ch = ev.width, ev.height
        self._dirty = True

    def _fit_log(self):
        """窗口变矮时让日志区让位给右侧卡片 —— 日志是次要信息，卡片是操作。

        没有这一步，用户在 768 高的笔记本上把窗口拉小，控制按钮会跑到窗口外面。
        """
        K = self.m
        if not hasattr(self, "log_outer"):
            return
        total = self.root.winfo_height()
        need = (self.sidebar_need(K) + K(14) * 2 + K(58) + K(14))
        want = total - need - K(16)
        h = int(max(K(64), min(K(150), want)))
        try:
            cur = int(self.log_outer.cget("height"))
        except Exception:                                    # noqa: BLE001
            cur = h
        if abs(h - cur) > 2:
            self.log_outer.configure(height=h)

    def _hit(self, x, y):
        for bx, by, bw, bh, act in self._buttons:
            if bx <= x < bx + bw and by <= y < by + bh:
                return act
        return None

    def _on_motion(self, ev):
        h = self._hit(ev.x, ev.y)
        if h != self._hover:
            self._hover = h
            self._dirty = True

    def _set_hover(self, h):
        if h != self._hover:
            self._hover = h
            self._dirty = True

    def _on_press(self, ev):
        self._pressed = self._hit(ev.x, ev.y)
        self._dirty = True

    def _on_release(self, ev):
        act = self._hit(ev.x, ev.y)
        pressed, self._pressed = self._pressed, None
        self._dirty = True
        if act is None or act != pressed:
            return
        if act == "connect":
            self.toggle_connect()
        elif act == "port":
            self._cycle_port()
        elif act == "rescan":
            self._refresh_ports()
        elif act == "ping":
            self.cmd(P.CMD_PING, "PING")
        elif act.startswith("color:"):
            self.send_color(act.split(":", 1)[1])
        elif act == "csv":
            self.toggle_record()
        elif act == "clear":
            self.clear_log()
        elif act == "raw":
            self.raw_mode = not self.raw_mode
            if self.raw_mode:
                self.log("--- 原始帧模式：只显示收到的字节，不刷统计 ---", "hint")
            else:
                self.log("--- 回到日志模式 ---", "hint")
        elif act == "baud":
            self._ask_baud()

    # ------------------------------------------------------------ 绘制

    def _btn(self, x, y, w, h, act):
        self._buttons.append((x, y, w, h, act))

    def _draw(self):
        cv, K = self.cv, self.m
        cw, ch = self.cw, self.ch
        cv.delete("all")
        self._buttons = []

        pad = K(14)
        bar_h = K(58)
        gap = K(12)

        # ---------------- 顶部 app bar
        rrect(cv, pad, pad, cw - pad * 2, bar_h, K(20), SURFACE_2)
        cx = pad + K(20)
        cy = pad + bar_h / 2
        # 状态点
        dot = C_GREEN if (self.link and not self.stats.stale) else (
            C_YELLOW if self.link else SURFACE_5)
        cv.create_oval(cx - K(5), cy - K(5), cx + K(5), cy + K(5),
                       fill=dot, outline=dot)
        cx += K(18)
        if self.link is None:
            conn_txt = "未连接"
        elif self.link.is_demo:
            conn_txt = "演示模式"
        else:
            conn_txt = "%s 已连接" % getattr(self.link, "port", "")
        conn_item = cv.create_text(cx, cy, text=conn_txt, anchor="w",
                                   fill=ON_SURFACE, font=K.font(15, bold=True))
        # 用画布自己量出来的边界去排下一个元素 —— 按字符数估算宽度在不同
        # 字体/缩放下必然重叠（第一版就是这么翻车的）。
        bb = cv.bbox(conn_item)
        cx = (bb[2] if bb else cx + K(120)) + K(18)

        cv.create_text(cx, cy, text="MaixCAM 视觉上位机",
                       anchor="w", fill=ON_SURFACE_DIM, font=K.font(14))

        # 右侧帧率药丸
        fps_txt = "%.1f fps" % self.stats.fps
        fw = K(104)
        fx = cw - pad - K(16) - fw
        pill(cv, fx, cy - K(16), fw, K(32), PRIMARY_CONT if self.link else SURFACE_3)
        cv.create_text(fx + fw / 2, cy, text=fps_txt,
                       fill=PRIMARY if self.link else ON_SURFACE_DIM,
                       font=K.font(14, bold=True, mono=True))
        self._btn(fx, cy - K(16), fw, K(32), "none")

        # ---------------- 内容区
        # 画面卡按 4:3 定价（视频就是 4:3 的，给宽了只会两边压黑边），
        # 剩下的宽度全给右侧栏 —— 这样换任何屏幕都不会留怪异的空白。
        top = pad * 2 + bar_h
        bottom = ch - pad
        avail_w = cw - pad * 2
        avail_h = max(K(200), bottom - top)
        video_w = min(avail_w * 0.62, avail_h * 4.0 / 3.0 + K(24))
        video_w = max(K(320), video_w)
        side_w = max(K(300), avail_w - video_w - gap)
        canvas_w = avail_w - side_w - gap
        canvas_h = avail_h

        self._draw_video(pad, top, canvas_w, canvas_h)

        sx = pad * 2 + canvas_w
        self._draw_sidebar(sx, top, cw - sx - pad, canvas_h, gap)

    def _draw_video(self, x, y, w, h):
        cv, K = self.cv, self.m
        rrect(cv, x, y, w, h, K(24), SURFACE_1)

        pad = K(12)
        x0, y0 = x + pad, y + pad
        w0, h0 = w - pad * 2, h - pad * 2

        # 4:3 等比内框（上下或左右留黑边）
        if w0 / h0 > 4 / 3:
            vh = h0
            vw = vh * 4 / 3
        else:
            vw = w0
            vh = vw * 3 / 4
        vx = x0 + (w0 - vw) / 2
        vy = y0 + (h0 - vh) / 2
        cv.create_rectangle(vx, vy, vx + vw, vy + vh, fill=CANVAS_BG, outline=CANVAS_BG)

        # 网格
        for i in range(1, 4):
            gx = vx + vw * i / 4.0
            gy = vy + vh * i / 4.0
            cv.create_line(gx, vy, gx, vy + vh, fill="#17171C")
            cv.create_line(vx, gy, vx + vw, gy, fill="#17171C")
        # 中心十字（640x480 的中心是 320,240）
        cxq, cyq = vx + vw / 2, vy + vh / 2
        cv.create_line(cxq, vy, cxq, vy + vh, fill="#26262E", dash=(K(3), K(6)))
        cv.create_line(vx, cyq, vx + vw, cyq, fill="#26262E", dash=(K(3), K(6)))
        cv.create_text(vx + vw - K(8), vy + vh - K(10), anchor="e",
                       text="640×480 参考系 · 中心 320,240",
                       fill="#3A3A44", font=K.font(11, mono=True))

        f = self.last
        if f is None or not f["exists"]:
            tip = "等待数据…" if self.link is None else "画面里没有目标"
            cv.create_text(cxq, cyq, text=tip, fill=ON_SURFACE_DIM, font=K.font(18))
            return

        # 轨迹
        if len(self._trail) > 1:
            pts = []
            for tx, ty in self._trail:
                pts.extend((vx + tx / 640.0 * vw, vy + ty / 480.0 * vh))
            cv.create_line(*pts, fill=SURFACE_4, width=K(2), smooth=True)

        # 目标框（f["cx"]/f["cy"] 是中心，单位是 640×480 参考系）
        col = COLOR_HEX.get(f["color_name"], ON_SURFACE_DIM)
        bw = f["w"] / 640.0 * vw
        # 协议里 w 和 h 都按**画面宽度**归一化，所以换算到屏幕高度时要走 480 这一侧
        bh = f["h"] / 480.0 * vh
        bx = vx + f["cx"] / 640.0 * vw - bw / 2
        by = vy + f["cy"] / 480.0 * vh - bh / 2

        cv.create_rectangle(bx, by, bx + bw, by + bh, outline=col, width=K(3))
        pcx, pcy = bx + bw / 2, by + bh / 2
        cv.create_oval(pcx - K(4), pcy - K(4), pcx + K(4), pcy + K(4), fill=col, outline="")

        # 标签
        label = "%s · %s" % (f["color_name"].upper(), shape_name(f["is_ring"]))
        ly = by - K(14) if by - K(14) > vy + K(10) else by + K(6)
        cv.create_text(bx, ly, text=label, anchor="w", fill=col, font=K.font(15, bold=True))

        # 到中心的连线
        if abs(f["cx"] - 320) > 4 or abs(f["cy"] - 240) > 4:
            cv.create_line(pcx, pcy, cxq, cyq, fill=SURFACE_5, dash=(K(2), K(4)))

        # 左上角坐标读数
        cv.create_text(vx + K(8), vy + K(10), anchor="w",
                       text="中心 %d,%d   尺寸 %d×%d" % (f["cx"], f["cy"], f["w"], f["h"]),
                       fill=ON_SURFACE_DIM, font=K.font(12, mono=True))

        # 断流横幅
        if self.stats.stale:
            msg = "数据过期 %.1f s" % self.stats.idle
            tw = K(200)
            rrect(cv, cxq - tw / 2, vy + K(14), tw, K(34), K(17), ERROR_CONT)
            cv.create_text(cxq, vy + K(31), text=msg, fill=ERROR,
                           font=K.font(14, bold=True))

    # 三张卡在"标准模式"下各需要多少设计像素（含间隙）
    GOAL_H = 150
    COORD_H = 150
    CTRL_H = 300

    @classmethod
    def sidebar_need(cls, K):
        return K(cls.GOAL_H + cls.COORD_H + cls.CTRL_H) + K(12) * 2

    def _draw_sidebar(self, x, y, w, h, gap):
        K = self.m
        # 窗口被拉矮时切"紧凑模式"：卡片和行高一起压，保证按钮不跑到窗口外面
        compact = h < self.sidebar_need(K)
        goal_h = K(126 if compact else self.GOAL_H)
        coord_h = K(126 if compact else self.COORD_H)
        ctrl_h = max(K(180), h - goal_h - coord_h - gap * 2)
        cy = y
        self._card_goal(x, cy, w, goal_h)
        cy += goal_h + gap
        self._card_coord(x, cy, w, coord_h)
        cy += coord_h + gap
        self._card_ctrl(x, cy, w, ctrl_h)

    def _card_head(self, x, y, w, title):
        K = self.m
        cv = self.cv
        rrect(cv, x, y, w, K(38), K(14), SURFACE_2)
        cv.create_text(x + K(16), y + K(19), text=title, anchor="w",
                       fill=ON_SURFACE_VAR, font=K.font(14, bold=True))

    def _card_goal(self, x, y, w, h):
        K, cv = self.m, self.cv
        rrect(cv, x, y, w, h, K(22), SURFACE_1)
        self._card_head(x, y, w, "目标")

        f = self.last
        has = bool(f and f["exists"])
        # 大标题：有目标 / 无目标
        title = "有目标" if has else "无目标"
        col = ON_SURFACE if has else ON_SURFACE_DIM
        cv.create_text(x + K(18), y + K(38) + K(22), text=title, anchor="w",
                       fill=col, font=K.font(30, bold=True))

        if has:
            cname = f["color_name"]
            yy = y + K(38) + K(62)
            cv.create_oval(x + K(20), yy - K(6), x + K(32), yy + K(6),
                           fill=COLOR_HEX.get(cname, ON_SURFACE_DIM),
                           outline=COLOR_HEX.get(cname, ON_SURFACE_DIM))
            cv.create_text(x + K(42), yy, text="%s  %s" % (COLOR_NAME_CN.get(cname, "?"),
                                                         cname.upper()),
                           anchor="w", fill=ON_SURFACE, font=K.font(17, bold=True))
            cv.create_text(x + w - K(18), yy,
                           text="%s · %.0f°" % (shape_name(f["is_ring"]), f.get("angle_deg", 0)),
                           anchor="e", fill=ON_SURFACE_VAR, font=K.font(15))
        else:
            cv.create_text(x + w - K(18), y + K(38) + K(22),
                           text="%.1f fps" % self.stats.fps, anchor="e",
                           fill=ON_SURFACE_DIM, font=K.font(15, mono=True))

    def _card_coord(self, x, y, w, h):
        K, cv = self.m, self.cv
        rrect(cv, x, y, w, h, K(22), SURFACE_1)
        self._card_head(x, y, w, "位置与尺寸")

        f = self.last
        rows = [("中心 X", "—"), ("中心 Y", "—"), ("宽 W", "—"), ("高 H", "—")]
        if f and f["exists"]:
            rows = [("中心 X", "%d px" % f["cx"]), ("中心 Y", "%d px" % f["cy"]),
                    ("宽 W", "%d px" % f["w"]), ("高 H", "%d px" % f["h"])]
        top = y + K(38) + K(10)
        rh = K(26)
        for i, (k, v) in enumerate(rows):
            ry = top + i * rh
            cv.create_text(x + K(18), ry, text=k, anchor="w",
                           fill=ON_SURFACE_DIM, font=K.font(14))
            cv.create_text(x + w - K(18), ry, text=v, anchor="e",
                           fill=ON_SURFACE, font=K.font(15, bold=True, mono=True))

    def _card_ctrl(self, x, y, w, h):
        K, cv = self.m, self.cv
        rrect(cv, x, y, w, h, K(22), SURFACE_1)
        self._card_head(x, y, w, "控制")

        # 高度不够就整体压：行高、间隙、字号一起缩，保证所有按钮都留在卡片里
        full = K(38) + K(12) + K(20) + K(44) + K(12) + K(42) + K(10) + K(42) + K(10) + K(38) + K(14)
        f = min(1.0, max(0.62, (h - K(8)) / float(full)))
        gapv = K(12) * f
        by = y + K(38) * f + gapv
        pad = K(14)

        # --- 分段按钮：识别颜色
        cv.create_text(x + K(18), by, text="识别颜色", anchor="w",
                       fill=ON_SURFACE_DIM, font=K.font(13 * f))
        by += K(20) * f
        seg_h = K(44) * f
        seg_w = (w - pad * 2 - K(8)) / 3.0
        rrect(cv, x + pad, by, w - pad * 2, seg_h, seg_h / 2, SURFACE_3)
        for i, cname in enumerate(("red", "green", "blue")):
            bx = x + pad + i * (seg_w + K(4))
            bw = seg_w - K(4)
            act = "color:" + cname
            if self._hover == act:
                pill(cv, bx, by, bw, seg_h, SURFACE_4)
            cv.create_text(bx + bw / 2, by + seg_h / 2,
                           text=COLOR_NAME_CN[cname], fill=COLOR_HEX[cname],
                           font=K.font(18 * f, bold=True))
            self._btn(bx, by, bw, seg_h, act)
        by += seg_h + gapv

        # --- PING + 录制
        bh = K(42) * f
        half = (w - pad * 2 - K(10)) / 2.0
        self._draw_tonal(x + pad, by, half, bh, "PING", "ping", font_size=15 * f)
        rec_txt = "停止录制" if self._rec else "录制 CSV"
        self._draw_tonal(x + pad + half + K(10), by, half, bh, rec_txt, "csv",
                         active=bool(self._rec), font_size=15 * f)
        by += bh + gapv * 0.83

        # --- 串口 + 连接
        ph = K(42) * f
        port_txt = ("演示数据源" if getattr(self.link, "is_demo", False)
                    else (self._cur_port or "无串口")) + " ▾"
        baud_w = K(104) * f
        conn_w = K(84) * f
        pw = w - pad * 2 - baud_w - conn_w - K(10) * 2
        self._draw_tonal(x + pad, by, pw, ph, port_txt, "port", align="w",
                         font_size=14 * f)
        self._draw_tonal(x + pad + pw + K(10), by, baud_w, ph, str(self.baud),
                         "baud", font_size=13 * f)
        self._draw_filled(x + pad + pw + baud_w + K(20), by, conn_w, ph,
                          "断开" if self.link else "连接", "connect",
                          font_size=15 * f)
        by += ph + gapv * 0.83

        bw3 = (w - pad * 2 - K(10) * 2) / 3.0
        self._draw_tonal(x + pad, by, bw3, K(38) * f, "重扫", "rescan",
                         font_size=12 * f)
        self._draw_tonal(x + pad + bw3 + K(10), by, bw3, K(38) * f, "原始帧",
                         "raw", active=self.raw_mode, font_size=12 * f)
        self._draw_tonal(x + pad + (bw3 + K(10)) * 2, by, bw3, K(38) * f,
                         "清空", "clear", font_size=12 * f)

    def _draw_tonal(self, x, y, w, h, text, act, active=False, align="c", font_size=15):
        K, cv = self.m, self.cv
        base = PRIMARY_CONT if active else SURFACE_3
        hovered = self._hover == act
        if hovered:
            base = SURFACE_4 if not active else "#5E47A0"
        if self._pressed == act:
            base = SURFACE_5
        pill(cv, x, y, w, h, base)
        fg = PRIMARY if active else ON_SURFACE
        if align == "w":
            cv.create_text(x + K(16), y + h / 2, text=text, anchor="w", fill=fg,
                           font=K.font(font_size, bold=True, mono=True))
        else:
            cv.create_text(x + w / 2, y + h / 2, text=text, fill=fg,
                           font=K.font(font_size, bold=True))
        self._btn(x, y, w, h, act)

    def _draw_filled(self, x, y, w, h, text, act, font_size=16):
        K, cv = self.m, self.cv
        base = PRIMARY
        if self._hover == act:
            base = "#E2D3FF"
        if self._pressed == act:
            base = "#B9A5E8"
        pill(cv, x, y, w, h, base)
        cv.create_text(x + w / 2, y + h / 2, text=text, fill=ON_PRIMARY,
                       font=K.font(font_size, bold=True))
        self._btn(x, y, w, h, act)


# ================================================================ 入口

def main(argv=None):
    ap = argparse.ArgumentParser(description="智能搬运 · 上位机（Material You 界面）")
    ap.add_argument("--port", help="启动即连接的串口，如 COM5")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--demo", action="store_true",
                    help="演示模式：自己造数据，不需要相机和串口")
    args = ap.parse_args(argv)

    # 让 Windows 按真实 DPI 渲染，否则高分屏上整窗会被系统缩放拉糊
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:                                    # noqa: BLE001
            ctypes.windll.user32.SetProcessDPIAware()
    except Exception:                                        # noqa: BLE001
        pass

    root = tk.Tk()
    app = GroundStation(root, args)
    app.log("上位机已启动。协议 v4：上行 20 字节 / 下行 4 字节。", "hint")
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
