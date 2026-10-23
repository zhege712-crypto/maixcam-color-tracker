"""
上位机调试工具：读 CH340 串口，按协议解析相机上报的数据
=========================================================

用途：在接 STM32 之前，先单独验证「相机 -> 串口 -> 电脑」整条链路是否正常。
      也可以用它给相机下发命令（切换颜色 / 帧率 / 待机 / PING / SAVE）。

用法：
    python serial_monitor.py --list                     列出可用串口
    python serial_monitor.py --port COM5                开始监听（Ctrl+C 结束）
    python serial_monitor.py --port COM5 --color green  启动时切换追踪颜色
    python serial_monitor.py --port COM5 --color green --repeat
                                                        重复下发直到收到相机的第一帧
    python serial_monitor.py --simulate                 不开串口，用模拟数据自检本工具

运行中可交互输入命令（回车执行）：
    r / g / b     切换追踪 红 / 绿 / 蓝
    run / stby    运行 / 待机
    fps 30        设置帧率
    ping          链路自检
    save          让相机保存配置
    q             退出

依赖：pyserial（只有真实串口模式需要）
    pip install pyserial
"""

import argparse
import math
import sys
import threading
import time

# 让中文在 Windows 控制台也能正常打印
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

import protocol as P

try:
    import serial  # pyserial
    from serial.tools import list_ports
    HAVE_SERIAL = True
except Exception:
    serial = None
    list_ports = None
    HAVE_SERIAL = False


# ---------------------------------------------------------------- 打印

def pct(v):
    return v * 100.0 / 255.0


def fmt_frame(f):
    tag = "TARGET" if f["exists"] else "no-tgt"
    st = P.STATE_NAMES.get(f.get("state", 0), "?")
    return (
        "seq=%3d  %-6s %-5s [%-4s]  xn=%3d(%5.1f%%) yn=%3d(%5.1f%%)  "
        "wn=%3d hn=%3d  dx=%+4d dy=%+4d"
        % (f["seq"], tag, f["color_name"], st,
           f["x_n"], pct(f["x_n"]), f["y_n"], pct(f["y_n"]),
           f["w_n"], f["h_n"], f["dx"], f["dy"])
    )


# ---------------------------------------------------------------- 统计

class Stats:
    def __init__(self):
        self.total = 0
        self.no_target = 0
        self.lost = 0          # 根据帧序号推算的丢帧
        self.bad = 0           # 校验失败的帧
        self._prev_seq = None
        self._t0 = time.time()
        self._last_report = time.time()
        self._window = 0

    def on_frame(self, f):
        self.total += 1
        self._window += 1
        if not f["exists"]:
            self.no_target += 1
        self.lost += P.seq_gap(self._prev_seq, f["seq"])
        self._prev_seq = f["seq"]

    def tick(self, parser):
        """每秒打印一次状态行，返回是否打印了。"""
        now = time.time()
        if now - self._last_report < 1.0:
            return False
        win = now - self._last_report
        self.bad = parser.bad_frames
        print(
            "  ── 速率 %.1f 帧/秒   累计 %d 帧   无目标 %d   丢帧 %d   坏帧 %d"
            % (self._window / win, self.total, self.no_target, self.lost, self.bad)
        )
        self._window = 0
        self._last_report = now
        return True

    def summary(self, parser):
        dt = max(1e-6, time.time() - self._t0)
        print("\n" + "=" * 60)
        print("累计 %.1f 秒，收到 %d 帧（平均 %.1f 帧/秒）" % (dt, self.total, self.total / dt))
        print("  无目标 %d 帧   丢帧 %d   校验失败 %d"
              % (self.no_target, self.lost, parser.bad_frames))
        if self.total:
            print("  丢帧率 %.2f%%" % (100.0 * self.lost / (self.total + self.lost)))
        print("=" * 60)


# ---------------------------------------------------------------- 下发命令

def make_cmd(text):
    """把一行命令文本转成下行帧；无法识别返回 None。"""
    t = text.strip().lower()
    if t in ("r", "red"):
        return P.build_downlink(P.CMD_SET_COLOR, P.COLOR_RED), "SET_COLOR red"
    if t in ("g", "green"):
        return P.build_downlink(P.CMD_SET_COLOR, P.COLOR_GREEN), "SET_COLOR green"
    if t in ("b", "blue"):
        return P.build_downlink(P.CMD_SET_COLOR, P.COLOR_BLUE), "SET_COLOR blue"
    if t == "run":
        return P.build_downlink(P.CMD_RUN_STANDBY, 0x01), "RUN"
    if t in ("stby", "standby"):
        return P.build_downlink(P.CMD_RUN_STANDBY, 0x00), "STANDBY"
    if t == "ping":
        return P.build_downlink(P.CMD_PING, 0x00), "PING"
    if t == "save":
        return P.build_downlink(P.CMD_SAVE, 0x00), "SAVE"
    if t in ("next", "n"):
        return P.build_downlink(P.CMD_NEXT, 0x00), "NEXT"
    if t in ("abort", "ab"):
        return P.build_downlink(P.CMD_ABORT, 0x00), "ABORT"
    if t in ("cal", "calib"):
        return P.build_downlink(P.CMD_CALIBRATE, 0x00), "CALIBRATE"
    if t.startswith("fps"):
        parts = t.split()
        if len(parts) == 2 and parts[1].isdigit() and 1 <= int(parts[1]) <= 60:
            return P.build_downlink(P.CMD_SET_FPS, int(parts[1])), "SET_FPS %s" % parts[1]
    return None


def send_cmd(ser, text, quiet=False):
    r = make_cmd(text)
    if r is None:
        if not quiet:
            print("  无法识别的命令：%s（可用：r/g/b、run/stby、fps N、ping、save、next、abort）" % text)
        return False
    frame, name = r
    ser.write(frame)
    if not quiet:
        print("  -> 已下发 %-16s %s" % (name, " ".join("%02X" % b for b in frame)))
    return True


# ---------------------------------------------------------------- 交互线程

def start_stdin_thread(ser, stop_flag):
    def worker():
        while not stop_flag["stop"]:
            try:
                line = input()
            except (EOFError, KeyboardInterrupt):
                return
            if line.strip().lower() in ("q", "quit", "exit"):
                stop_flag["stop"] = True
                return
            if line.strip():
                send_cmd(ser, line)
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    return t


# ---------------------------------------------------------------- 模拟模式

def simulate(args):
    """不开串口：自己造帧、偶尔插噪声和坏帧，验证解析与统计是否正确。"""
    print("模拟模式：生成假的相机数据，用于自检本工具（不需要相机和串口）")
    print("每隔 1 秒打印一次速率，Ctrl+C 结束\n")
    parser = P.UplinkParser()
    stats = Stats()
    seq = 0
    t0 = time.time()
    noise_at = 1.0
    try:
        while True:
            t = time.time() - t0
            if not args.duration or t < args.duration:
                # 造一个绕圈移动的"物块"，尺寸缓慢变大
                ang = t * 1.2
                cx = args.width / 2.0 + math.cos(ang) * args.width * 0.30
                cy = args.height / 2.0 + math.sin(ang) * args.height * 0.30
                size = 60 + 90 * (0.5 + 0.5 * math.sin(t * 0.4))
                blob = [cx - size / 2, cy - size / 2, size, size * 0.85]
                if int(t) % 10 == 9:      # 每 10 秒里模拟 1 秒丢失目标
                    blob = None
                # 复用与相机端一致的换算逻辑
                frame = _sim_frame(seq, args.color, blob, args.width, args.height)
                data = frame
                if abs(t - noise_at) < 0.02:      # 插入噪声 + 一个坏帧，验证重新同步
                    bad = bytearray(P.build_uplink(seq, 1, 5, 0, 10, 10, 10, 10))
                    bad[12] ^= 0xFF               # 破坏校验和
                    data = b"\x01\x02\xAA\xAA" + bytes(bad) + frame
                    print("  [自检] 已插入 3 字节噪声 + 1 个校验和错误帧")
                    noise_at += 3.0
                for f in parser.feed(data):
                    print(fmt_frame(f))
                    stats.on_frame(f)
                seq = (seq + 1) & 0xFF
            stats.tick(parser)
            if args.duration and t >= args.duration:
                break
            time.sleep(1.0 / args.fps)
    except KeyboardInterrupt:
        pass
    stats.summary(parser)


def _sim_frame(seq, color, blob, w, h):
    """与相机端 frame_from_blob 等价的换算（模拟模式用）。"""
    code = P.COLOR_VALUES[color]
    if blob is None:
        return P.build_uplink(seq, 0, code, 0, 0, 0, 0, 0)
    bx, by, bw, bh = blob
    cx, cy = bx + bw / 2.0, by + bh / 2.0
    return P.build_uplink(seq, 1, code, 0,
                          P.norm255(cx, w), P.norm255(cy, h),
                          P.norm255(bw, w), P.norm255(bh, w))


# ---------------------------------------------------------------- 真实串口

def run_serial(args):
    if not HAVE_SERIAL:
        print("!! 没有找到 pyserial，无法打开串口。")
        print("   安装：pip install pyserial      （或用 uv：uv pip install pyserial）")
        print("   想先验证本工具本身，可以跑：python serial_monitor.py --simulate")
        return 2

    if args.list:
        ports = list(list_ports.comports())
        if not ports:
            print("没有发现任何串口设备。检查 CH340 驱动是否装好、USB 是否插好。")
        for p in ports:
            print("%-8s %s" % (p.device, p.description))
        return 0

    if not args.port:
        print("请用 --port 指定串口，例如 --port COM5；不知道名字就先跑 --list")
        return 2

    print("打开 %s @ %d 8N1 ……" % (args.port, args.baud))
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    time.sleep(0.2)
    ser.reset_input_buffer()

    parser = P.UplinkParser()
    stats = Stats()
    stop_flag = {"stop": False}

    # 启动时的命令（可重复下发，直到收到相机第一帧 —— 对应"相机开机慢"的场景）
    pending = []
    if args.color:
        pending.append(args.color)
    if args.fps:
        pending.append("fps %d" % args.fps)
    if args.standby:
        pending.append("stby")
    if args.run:
        pending.append("run")
    if args.ping:
        pending.append("ping")
    if args.save:
        pending.append("save")

    for c in pending:
        send_cmd(ser, c)

    if pending and args.repeat:
        print("--repeat 已启用：每 200ms 重发一次，直到收到相机第一帧（相机开机约十几秒）")

    if sys.stdin.isatty():
        start_stdin_thread(ser, stop_flag)
        print("运行中可直接输入命令：r/g/b、run/stby、fps N、ping、save、q")
    print("-" * 60)

    last_repeat = time.time()
    first_frame_seen = False
    try:
        while not stop_flag["stop"]:
            data = ser.read(256)
            if data:
                for f in parser.feed(data):
                    if not first_frame_seen:
                        first_frame_seen = True
                        print("收到相机第一帧，链路正常。")
                    print(fmt_frame(f))
                    stats.on_frame(f)
            if pending and args.repeat and not first_frame_seen:
                if time.time() - last_repeat >= 0.2:
                    for c in pending:
                        send_cmd(ser, c, quiet=True)
                    last_repeat = time.time()
            stats.tick(parser)
    except KeyboardInterrupt:
        pass
    finally:
        stats.summary(parser)
        ser.close()
    return 0


# ---------------------------------------------------------------- 抓包文件解码

def decode_file(path):
    """解析 pc\\serial_capture.ps1 抓下来的原始字节。

    这条路不需要 pyserial —— 采集交给 PowerShell 的 .NET 串口，
    解码复用这里已经测过的解析器。
    """
    import struct  # noqa: F401  (保持与其它分支一致的导入风格)
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        print("打不开文件 %s：%s" % (path, e))
        return 2

    print("文件 %s，共 %d 字节" % (path, len(data)))
    parser = P.UplinkParser()
    stats = Stats()
    frames = parser.feed(data)
    for f in frames:
        print(fmt_frame(f))
        stats.on_frame(f)

    if not frames:
        print("\n!! 一个合法帧都没解出来。下面是前 64 字节的原始数据，"
              "用来判断是没数据、波特率不对还是接线问题：")
        print("   " + " ".join("%02X" % b for b in data[:64]))
        if not data:
            print("   （文件是空的 —— 线上没有数据。检查 A19 -> CH340 RX，"
                  "以及 GND 是否共地）")
        else:
            print("   （有数据但不是我们的协议帧 —— 通常是波特率不是 115200，"
                  "或接到了别的信号）")
    else:
        print("\n参考：协议里 30fps × 17 字节 ≈ 每秒 510 字节")

    # 文件模式没有"速率"的概念（解析是瞬间完成的），所以单独给个摘要
    print("\n" + "=" * 60)
    print("共解析出 %d 帧；无目标 %d 帧；丢帧 %d；校验/帧尾失败 %d"
          % (stats.total, stats.no_target, stats.lost, parser.bad_frames))
    if stats.total > 1:
        span = stats.total + stats.lost
        print("按帧序号推算，应有 %d 帧，实际收到 %d 帧，丢帧率 %.2f%%"
              % (span, stats.total, 100.0 * stats.lost / span))
    print("=" * 60)
    return 0 if frames else 1


# ---------------------------------------------------------------- 入口

def main(argv=None):
    ap = argparse.ArgumentParser(description="相机串口协议调试上位机")
    ap.add_argument("--list", action="store_true", help="列出可用串口")
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--color", choices=["red", "green", "blue"], help="启动时切换追踪颜色")
    ap.add_argument("--fps", type=int, help="启动时设置相机发送帧率(1-60)")
    ap.add_argument("--run", action="store_true", help="启动时让相机进入运行状态")
    ap.add_argument("--standby", action="store_true", help="启动时让相机进入待机状态")
    ap.add_argument("--ping", action="store_true", help="启动时发一次 PING")
    ap.add_argument("--save", action="store_true", help="启动时让相机保存配置")
    ap.add_argument("--repeat", action="store_true",
                    help="重复下发启动命令，直到收到相机第一帧")
    ap.add_argument("--simulate", action="store_true", help="模拟模式：不需要串口")
    ap.add_argument("--file", help="解析一个抓包文件（配合 pc\\serial_capture.ps1 使用，不需要 pyserial）")
    ap.add_argument("--duration", type=float, default=0, help="模拟模式运行秒数，0=一直跑")
    ap.add_argument("--width", type=int, default=640, help="模拟模式的画面宽度")
    ap.add_argument("--height", type=int, default=480, help="模拟模式的画面高度")
    args = ap.parse_args(argv)

    if args.file:
        return decode_file(args.file)

    if args.simulate:
        print("协议帧长：上行 %d 字节，下行 %d 字节" % (P.UP_FRAME_LEN, P.DOWN_FRAME_LEN))
        simulate(args)
        return 0
    return run_serial(args)


if __name__ == "__main__":
    sys.exit(main())
