"""
上位机调试工具：读 CH340 串口，按 v4 协议解析相机上报的数据
=============================================================

v4 与旧版（v3）的差别，直接决定了这个工具"能做什么、不能做什么"：
    · 上行 20 字节定长，x/y 是目标**中心**像素（不是左上角），
      坐标系固定 640x480；
    · 第 6 字节 bit7 = 是否色环，低 7 位 = 角度，必须用 decode_rotation() 拆；
    · **没有帧序号**，所以没有"丢帧"可算 —— 只能看每秒实际到了多少帧；
    · 下行只有 4 字节 AA BB cmd FF：0x01/02/03 选红/绿/蓝，0x7F 是 PING；
    · 没有回执帧、没有任务阶段：命令发出去就完了，收不到 OK/FAIL。

用途：接 STM32 之前，先单独验证「相机 -> 串口 -> 电脑」整条链路是否正常，
      也用来给相机下发选色 / PING。

用法：
    python serial_monitor.py --list                     列出可用串口
    python serial_monitor.py --port COM5                开始监听（Ctrl+C 结束）
    python serial_monitor.py --port COM5 --color green  启动时先下发"选绿色"
    python serial_monitor.py --simulate --duration 3    不开串口，自检本工具
    python serial_monitor.py --file capture.bin         解码抓包文件（不需要 pyserial）

运行中可交互输入命令（回车执行）：
    r / g / b     让相机改认 红 / 绿 / 蓝
    ping          链路自检（相机收到会立刻回一帧）
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

# 模拟模式固定的发帧节奏。v4 已经删掉 SET_FPS 命令，相机帧率不能再从
# 电脑端改，所以这里也不再暴露 --fps，免得让人以为还能远程调速。
SIM_FPS = 30.0


# ---------------------------------------------------------------- 打印

def fmt_frame(f):
    """一行打印一帧。

    exists=0 时协议规定坐标全 0，如果照原样打印会变成"中心 (0,0)"，
    看起来像目标跑到左上角去了 —— 很容易误判。所以无目标时坐标显示 "—"，
    只保留颜色字段（相机仍然记得当前追踪的是哪种颜色）。
    """
    if f["exists"]:
        shape = "色环" if f["is_ring"] else "色块"
        line = "存在 · %-5s · %s · 中心 (%4d,%4d) · 尺寸 (%3d×%3d)" % (
            f["color_name"], shape, f["cx"], f["cy"], f["w"], f["h"])
        if f["is_ring"]:
            # 只有环才有"角度"的意义；色块的角度字段是无效值
            line += " · 角度 %5.1f°" % f["angle_deg"]
        return line
    return "无   · %-5s · —    · 中心 (   —,   —) · 尺寸 (  —×  —)" % (
        f["color_name"])


# ---------------------------------------------------------------- 统计

class Stats:
    """只统计 v4 能可靠测量的东西。

    v4 没有帧序号，收到多少帧就是多少帧，"应该收到多少"无从得知，
    所以这里故意不提供 lost / loss_pct：算出来的任何"丢帧率"都是编的。
    """

    def __init__(self):
        self.total = 0
        self.no_target = 0
        self.rings = 0
        self.blocks = 0
        self._t0 = time.time()
        self._last_report = self._t0
        self._window = 0           # 本统计窗口内收到的帧数

    def on_frame(self, f):
        self.total += 1
        self._window += 1
        if f["exists"]:
            if f["is_ring"]:
                self.rings += 1
            else:
                self.blocks += 1
        else:
            self.no_target += 1

    def tick(self, parser):
        """每秒打印一次实际到达速率，返回是否打印了。"""
        now = time.time()
        if now - self._last_report < 1.0:
            return False
        win = now - self._last_report
        print(
            "  ── 速率 %.1f 帧/秒   累计 %d 帧   无目标 %d   坏帧 %d"
            % (self._window / win, self.total, self.no_target, parser.bad_frames)
        )
        self._window = 0
        self._last_report = now
        return True

    def summary(self, parser):
        dt = max(1e-6, time.time() - self._t0)
        print("\n" + "=" * 68)
        print("累计 %.1f 秒，收到 %d 帧（平均 %.1f 帧/秒）"
              % (dt, self.total, self.total / dt))
        print("  无目标 %d 帧   色环 %d   色块 %d   校验/帧尾失败 %d"
              % (self.no_target, self.rings, self.blocks, parser.bad_frames))
        print("  注：v4 没有帧序号，无法推算丢帧；上面的帧率就是实际到达速率。")
        print("=" * 68)


# ---------------------------------------------------------------- 下发命令

def make_cmd(text):
    """把一行命令文本转成 4 字节下行帧；无法识别返回 None。

    v4 的 cmd 值与颜色号相同（1/2/3），所以选色不用再带一个参数 ——
    旧版的 build_downlink(cmd, arg) 在这里已经不存在了。
    """
    t = text.strip().lower()
    if t in ("r", "red"):
        return P.build_downlink(P.CMD_RED), "TRACK_RED"
    if t in ("g", "green"):
        return P.build_downlink(P.CMD_GREEN), "TRACK_GREEN"
    if t in ("b", "blue"):
        return P.build_downlink(P.CMD_BLUE), "TRACK_BLUE"
    if t == "ping":
        return P.build_downlink(P.CMD_PING), "PING"
    return None


def send_cmd(ser, text, quiet=False):
    r = make_cmd(text)
    if r is None:
        if not quiet:
            print("  无法识别的命令：%s（可用：r/g/b、ping、q）" % text)
        return False
    frame, name = r
    ser.write(frame)
    if not quiet:
        print("  -> 已下发 %-11s %s" % (name, " ".join("%02X" % b for b in frame)))
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

def _sim_frame(t):
    """按 v4 造一帧：坐标直接就是 640x480 参考系里的**中心像素**。

    这里不再经过"blob -> 归一化"那套换算 —— v4 已经改成绝对像素坐标，
    模拟数据必须走和真相机一样的 P.build_uplink，否则测的就不是同一条路径。
    """
    colors = ("red", "green", "blue")
    color = colors[int(t / 6.0) % 3]

    # 开头 0.4 秒先"没找到目标"，这样 --duration 3 这种短跑也能同时
    # 看到"有目标"和"无目标"两种打印，方便确认 exists 分支没写反。
    # 之后每 7 秒里再抽 0.8 秒丢目标，模拟目标离开视野。
    if t < 0.4 or (t % 7.0) >= 6.2:
        return P.build_uplink(0, P.COLOR_VALUES[color],
                              P.encode_rotation(0.0, False), 0, 0, 0, 0)

    ang = t * 1.1
    cx = P.REF_W / 2.0 + math.cos(ang) * P.REF_W * 0.28
    cy = P.REF_H / 2.0 + math.sin(ang) * P.REF_H * 0.28
    w = 60.0 + 70.0 * (0.5 + 0.5 * math.sin(t * 0.4))
    h = w * 0.8
    is_ring = int(t / 2.0) % 2 == 1      # 每 2 秒换一种形状，环/块都覆盖到
    angle = (t * 90.0) % 360.0 if is_ring else 0.0
    return P.build_uplink(1, P.COLOR_VALUES[color],
                          P.encode_rotation(angle, is_ring),
                          int(cx), int(cy), int(w), int(h))


def simulate(args):
    """不开串口：自己造帧、偶尔插噪声和坏帧，验证解析与统计是否正确。"""
    print("模拟模式：用 P.build_uplink 生成 v4 帧（20 字节），不需要相机和串口")
    print("每帧一行，每秒一行速率；Ctrl+C 结束\n")
    parser = P.UplinkParser()
    stats = Stats()
    t0 = time.perf_counter()
    noise_at = 1.0
    next_at = t0
    try:
        while True:
            now = time.perf_counter()
            t = now - t0
            if args.duration and t >= args.duration:
                break

            data = _sim_frame(t)
            if t >= noise_at:
                # 故意把校验和字节改坏：验证解析器能把坏帧计数并重新同步。
                # 只翻转 checksum 本身（而不是坐标），这样坏帧的长度和帧头
                # 都合法，真正考验的是校验逻辑。
                bad = bytearray(data)
                bad[P.OFF_CHECKSUM] ^= 0xFF
                data = b"\x01\x02\xAA\xAA" + bytes(bad) + data
                print("  [自检] 已插入 3 字节噪声 + 1 个校验和错误帧")
                noise_at += 3.0

            for f in parser.feed(data):
                print(fmt_frame(f))
                stats.on_frame(f)

            stats.tick(parser)
            next_at += 1.0 / SIM_FPS
            time.sleep(max(0.0, next_at - time.perf_counter()))
    except KeyboardInterrupt:
        pass
    stats.summary(parser)


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

    # v4 里 --color 就是下发一条选色命令，一次就够：协议规定相机会一直
    # 按最后一次选色识别，不需要重复发（--repeat 已随 v3 的命令体系删掉）。
    if args.color:
        send_cmd(ser, args.color)
    if args.ping:
        send_cmd(ser, "ping")

    if sys.stdin.isatty():
        start_stdin_thread(ser, stop_flag)
        print("运行中可直接输入命令：r/g/b、ping、q")
    print("-" * 68)

    first_frame_seen = False
    try:
        while not stop_flag["stop"]:
            data = ser.read(256)
            if data:
                for f in parser.feed(data):
                    if not first_frame_seen:
                        first_frame_seen = True
                        print("收到相机第一帧，链路正常（v4，20 字节/帧）。")
                    print(fmt_frame(f))
                    stats.on_frame(f)
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
    解码复用这里已经测过的解析器。抓包文件里**没有时间戳**，所以只能
    数帧、不能算帧率；想测帧率请用 --port 实时看。
    """
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        print("打不开文件 %s：%s" % (path, e))
        return 2

    print("文件 %s，共 %d 字节（v4 上行帧 20 字节/帧）" % (path, len(data)))
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
            print("   （有数据但不是 v4 协议帧 —— 常见原因：波特率不是 115200，"
                  "或接到了别的信号；也可能是旧固件还在发 17 字节的 v3 帧）")
        return 1

    print("\n参考：v4 每帧 20 字节；实测 14~16 帧/秒 ≈ 280~320 字节/秒")
    print("\n" + "=" * 68)
    print("共解析出 %d 帧；无目标 %d；色环 %d；色块 %d；校验/帧尾失败 %d"
          % (stats.total, stats.no_target, stats.rings, stats.blocks,
             parser.bad_frames))
    print("抓包文件没有到达时间戳，无法算帧率；帧率请用 --port 实时看。")
    print("=" * 68)
    return 0


# ---------------------------------------------------------------- 入口

def main(argv=None):
    ap = argparse.ArgumentParser(description="相机串口协议调试上位机（v4）")
    ap.add_argument("--list", action="store_true", help="列出可用串口")
    ap.add_argument("--port", help="串口号，如 COM5")
    ap.add_argument("--baud", type=int, default=115200, help="波特率，默认 115200")
    ap.add_argument("--color", choices=["red", "green", "blue"],
                    help="启动时下发选色（之后相机会一直按这个颜色识别）")
    ap.add_argument("--ping", action="store_true", help="启动时发一次 PING")
    ap.add_argument("--simulate", action="store_true", help="模拟模式：不需要串口")
    ap.add_argument("--file", help="解析一个抓包文件（配合 pc\\serial_capture.ps1 使用）")
    ap.add_argument("--duration", type=float, default=0,
                    help="模拟模式运行秒数，0=一直跑")
    args = ap.parse_args(argv)

    if args.file:
        return decode_file(args.file)

    if args.simulate:
        print("协议帧长：上行 %d 字节，下行 %d 字节"
              % (P.UP_FRAME_LEN, P.DOWN_FRAME_LEN))
        simulate(args)
        return 0
    return run_serial(args)


if __name__ == "__main__":
    sys.exit(main())
