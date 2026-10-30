"""
MaixCAM 颜色物块追踪 —— 相机端程序
=====================================

用途：识别指定颜色的物块，把"是否存在 / 颜色 / 中心位置 / 尺寸"按归一化比例
      打包成 17 字节帧，通过 UART1 发给 STM32H723 主控。

协议：相机_主控串口通信协议_定稿.md（v3.0）
    上行 17 字节，下行 11 字节，115200 8N1

硬件：
    MaixCAM A19(UART1_TX) -> STM32 RX
    MaixCAM A18(UART1_RX) <- STM32 TX
    GND 共地
    ⚠ 不要用 UART0（A16/A17）：系统日志串口，且 A16 被拉低会导致无法开机

运行：用 MaixVision 打开本文件直接运行，或上传到设备后运行。
      退出程序后系统会自动回到桌面启动器，不会黑屏。

调阈值：设备自带的「找色块」应用里可以点屏幕取 LAB 值并自动生成阈值，
        把得到的 6 个数字填进下面 THRESHOLDS 即可（见 README）。
"""

# --- 标准库（协议层不依赖 maix，方便在电脑上做单元测试）---
import json
import math
import os
import sys

# 被启动器/init 拉起时 stdout 不是终端，默认是块缓冲，日志会卡在缓冲区里，
# 出问题时 tail 日志一个字都看不到。这里强制改成行缓冲，让日志实时落盘。
try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

PROTOCOL_VERSION = "v3.0"
PROGRAM_NAME = "MaixCAM 颜色物块追踪"

# ============================================================ 配置

CONFIG = {
    # ---- 串口 ----
    "uart_device": "/dev/ttyS1",   # UART1（MaixCAM）；不要用 /dev/ttyS0
    "uart_baud": 115200,           # 8N1，MaixCAM 上唯一被验证可靠的波特率
    "pin_tx": "A19",               # UART1_TX
    "pin_rx": "A18",               # UART1_RX

    # ---- 图像 ----
    "img_w": 640,                  # 协议已归一化，改分辨率不影响主控
    "img_h": 480,

    # ---- 识别 ----
    "default_color": "red",        # 上电默认追踪色：red / green / blue / auto（自动测色）
    "pixels_threshold": 500,       # 过滤小噪点，按实际情况调
    "send_fps": 30,                # 上行发送帧率，1-60

    # ---- v3.0 搬运任务（--mission）----
    # 两个"动作点"是整个精度设计的核心：不去标定相机内参、不算俯仰角，
    # 只标两个像素坐标。前提是执行该动作时云台俯仰角固定。
    "grab_point_n": (128, 128),    # 块中心落在这里 ≈ 夹爪正对块（现场标定）
    "drop_point_n": (128, 128),    # 环中心落在这里 ≈ 夹爪能放进环心（现场标定）
    "align_tol_n": 12,             # 对准容差，归一化单位（12/255 ≈ 画面宽的 4.7%）
    "grab_w_n": 90,                # 物料宽达到这个值才认为"够近了，可以抓"
    "drop_w_n": 110,               # 目标区宽达到这个值才认为"够近了，可以放"

    # ---- 比赛任务（附件1-智能搬运比赛规则）----
    # 任务码决定搬运顺序：'1'=红 '2'=绿 '3'=蓝，如 "123"。
    # 按顺序抓取每个 3 分（自动模式），所以顺序不能随便挑。
    "task_code": "123",
    # 搬运时车上物料在画面里的位置（归一化）：从这里排除自己抓着的那一个，
    # 否则会对着自己手里的物料"对准"，永远到不了目标区。
    "hold_point_n": None,
    "hold_ignore_n": 40,

    # ---- 现场触摸界面（没有电脑时用屏幕操作）----
    "ui": False,                   # true = 开机进触摸界面

    # ---- 配置文件（SAVE 命令会写它，上电读它）----
    "config_path": "/root/color_tracker_cfg.json",

    # ---- GPIO 硬急停（协议预留，默认不启用）----
    "estop_enabled": False,        # 启用后：低电平 = 停发（等同待机）
    "estop_pin": None,             # 待实机确认引脚，避开 A16/A17/A18/A19

    # ---- 重新标定的触发文件 ----
    # 想重标颜色时，SSH 里执行：touch /root/recalibrate
    # 程序每秒检查一次，发现文件就把物块放到画面中间重新测色（不用重启）
    "recal_file": "/root/recalibrate",
}

# LAB 阈值：[L_min, L_max, A_min, A_max, B_min, B_max]
#
# 红色那组已按真机实测放宽过（2026-10-16 实测一个红色警示灯）：
#   物体: L 10~100 均值47 | A -22~77 均值36 | B -23~94 均值30
#   背景: L 0~100 均值35 | A -8~26 均值2   | B -23~24 均值-6
# 关键结论：A > 40 就能把物体和背景干净分开；但官方的 L_max=80、B_max=80
# 会把物体的亮部和反光边缘切掉，所以放宽到 100 / 95。
# 绿色和蓝色仍是官方入门值，同样必须现场重新标定。
THRESHOLDS = {
    "red":   [[0, 100, 40, 110, 5, 95]],
    "green": [[0, 100, -128, -15, -20, 80]],
    "blue":  [[0, 100, -30, 70, -128, -30]],
}
# 2026-10-21 修正（用 pc/make_ring_test.py 生成的标准色图在真机上验出来的）：
#   绿色原为 [0,80,-120,-10,0,30]，B 只到 30，而实物绿色（深绿 20,120,40）
#   的 LAB 是 (43,-44,35)，b=35 直接被排除 —— **绿环根本检不出来**。
#   蓝色 A 下限 30 会漏掉偏紫的蓝，红色 A 上限 80 会让纯红(约80.1)正好卡边界。
# 三色现在靠 A/B 组合互斥：
#   红 A 40~110 且 B 5~95     绿 A -128~-15 且 B -20~80
#   蓝 A -30~70 且 B -128~-30
# 红与蓝的 A 有重叠，但 B 一个为正一个为负，不会互相误命中。

COLOR_CODE = {"red": 0x01, "green": 0x02, "blue": 0x03}
CODE_COLOR = {v: k for k, v in COLOR_CODE.items()}

# 调试用：全范围阈值，会匹配整个画面。
# 用它可以在没有目标物时验证"色块 -> 归一化 -> 组帧"整条链路是否算对：
# 正常结果应该是 xn≈128、yn≈128、wn/hn 很大（接近满屏）。
FULL_RANGE_THRESHOLD = [[0, 100, -128, 127, -128, 127]]

# ============================================================ 运行参数
# 设备上没有 timeout 命令，所以限时退出由程序自己实现；这样能走正常退出流程，
# 不会像 SIGKILL 那样把摄像头/显示驱动留在半开状态。
#
#   python3 maixcam_color_tracker.py                     正常跑
#   python3 maixcam_color_tracker.py --duration 15       跑 15 秒后自动退出
#   python3 maixcam_color_tracker.py --no-display        不开屏幕（SSH 调试用）
#   python3 maixcam_color_tracker.py --no-uart           不开串口（只验视觉）
#   python3 maixcam_color_tracker.py --color green --fps 20
#
# 上机调试建议分阶段：先 --no-display --no-uart 验视觉，再逐项打开。

def parse_args(argv):
    opts = {"duration": 0.0, "color": None, "fps": None,
            "no_display": False, "no_uart": False, "test_full": False,
            "save_image": None, "save_every": 3.0,
            "lock_ae": False, "lock_awb": False, "probe_lab": False,
            "auto_once": False,
            "mission": False, "grab_point": None, "drop_point": None,
            "grab_w": None, "drop_w": None,
            "task_code": None, "hold_point": None, "ui": False}

    def _point(s):
        try:
            a, b = s.split(",")
            return (int(a), int(b))
        except Exception:
            return None

    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--mission":
            opts["mission"] = True
            i += 1
            continue
        if a == "--ui":
            opts["ui"] = True
            i += 1
            continue
        if a == "--task-code" and i + 1 < len(argv):
            opts["task_code"] = argv[i + 1]
            i += 2
            continue
        if a == "--hold-point" and i + 1 < len(argv):
            opts["hold_point"] = _point(argv[i + 1])
            i += 2
            continue
        if a == "--grab-point" and i + 1 < len(argv):
            opts["grab_point"] = _point(argv[i + 1])
            i += 2
            continue
        if a == "--drop-point" and i + 1 < len(argv):
            opts["drop_point"] = _point(argv[i + 1])
            i += 2
            continue
        if a == "--grab-w" and i + 1 < len(argv):
            try:
                opts["grab_w"] = int(argv[i + 1])
            except ValueError:
                pass
            i += 2
            continue
        if a == "--drop-w" and i + 1 < len(argv):
            try:
                opts["drop_w"] = int(argv[i + 1])
            except ValueError:
                pass
            i += 2
            continue
        if a == "--duration" and i + 1 < len(argv):
            try:
                opts["duration"] = float(argv[i + 1])
            except ValueError:
                pass
            i += 2
            continue
        if a == "--color" and i + 1 < len(argv):
            # 除了 red/green/blue，也接受 auto（自动测色）
            if argv[i + 1] in THRESHOLDS or argv[i + 1] == "auto":
                opts["color"] = argv[i + 1]
            i += 2
            continue
        if a == "--auto-color":
            opts["color"] = "auto"
            i += 1
            continue
        if a == "--fps" and i + 1 < len(argv):
            try:
                v = int(argv[i + 1])
                if 1 <= v <= 60:
                    opts["fps"] = v
            except ValueError:
                pass
            i += 2
            continue
        if a == "--no-display":
            opts["no_display"] = True
        elif a == "--no-uart":
            opts["no_uart"] = True
        elif a == "--test-full":
            opts["test_full"] = True
        elif a == "--lock":
            # 故意只锁曝光：白平衡必须显式用 --lock-awb。
            # 实测本机 set_awb(0) 切手动白平衡后，增益被重置成 1:1:1，
            # 而 Bayer 的 G 分量是红蓝的两倍，整幅画面会变成一片绿。
            opts["lock_ae"] = True
        elif a == "--lock-ae":
            opts["lock_ae"] = True
        elif a == "--lock-awb":
            opts["lock_awb"] = True
        elif a == "--probe-lab":
            opts["probe_lab"] = True
        elif a == "--auto-once":
            # 只在开机测一次颜色，之后锁死（比赛时更稳，不会被别的颜色带跑）
            opts["auto_once"] = True
        elif a == "--save-image" and i + 1 < len(argv):
            opts["save_image"] = argv[i + 1]
            i += 2
            continue
        elif a == "--save-every" and i + 1 < len(argv):
            try:
                opts["save_every"] = max(0.5, float(argv[i + 1]))
            except ValueError:
                pass
            i += 2
            continue
        i += 1
    return opts


# ============================================================ 协议常量
#
# v4：与组内《相机_主控串口通信协议.docx》字节级一致。
# 和旧版 v3.0 的三处关键差别（改代码前务必看清）：
#   1. 坐标从"归一化 0-255"改回"像素 uint16 小端"，且 x/y 是**中心点**
#   2. 上行**没有帧序号**，所以主控/上位机算不了丢帧，只能看每秒帧率
#   3. 下行只有 4 字节、无校验和、无参数；相机也**不再回执**

UP_HEADER = b"\xAA\xAA\xBB\xBB"
UP_TAIL = b"\xFF\xFF\xEE\xEE"
UP_FRAME_LEN = 20

DOWN_HEADER = b"\xAA\xBB"
DOWN_TAIL = b"\xFF"
DOWN_FRAME_LEN = 4

# 下行指令：1/2/3 就是颜色号（主控发"识别哪种颜色"，相机一直按这个识别）
CMD_TRACK = {0x01: "red", 0x02: "green", 0x03: "blue"}
CMD_PING = 0x7F          # 扩展：调试用，收到立刻回一帧；主控不需要发

# 第 6 字节：bit7 = 色环标志，低 7 位 = 角度（0..127 映射 0..360 度）。
# 文档里这个字节是"旋转角度"，但圆柱没有旋转概念（文档自己也是这么说的），
# 所以把符号位拿来传"环还是块"——照文档写的解析器只会读到一个无意义的
# 旋转值，不受影响。
RING_BIT = 0x80
ANGLE_STEPS = 127

# 坐标参考系：不管内部用多少分辨率识别，送出去的坐标一律换算到 640x480，
# 这样主控那边的坐标含义永远不变（以后为了提帧率降分辨率也不会打架）。
REF_W = 640
REF_H = 480

# 上行帧内偏移
OFF_EXISTS = 4
OFF_COLOR = 5
OFF_ROTATION = 6
OFF_X = 7
OFF_Y = 9
OFF_W = 11
OFF_H = 13
OFF_CHECKSUM = 15
SUM_START = 4
SUM_END = 15             # 不含：校验覆盖偏移 4..14 共 11 字节

# ---- 任务阶段（**只用于本地**，v4 协议里不传）----
# 架构决定：搬运流程逻辑放在 STM32，相机只负责"看见并上报"。
# 这套状态机保留下来是为了本地演示/单测，不再出现在串口帧里。
# 按比赛规则（附件1-智能搬运比赛规则）定义：
#   第一批：原料区 3 个圆柱物料 --按任务码顺序--> 粗加工区对应颜色区域
#   第二批：粗加工区 3 个圆柱物料 --同样顺序--> 暂存区对应颜色区域
STATE_SEARCH = 0     # 搜索中 / 当前没有可用目标
STATE_MATERIAL = 1   # 正在对准物料（圆柱）
STATE_GRAB = 2       # 物料已就位，主控可以抓
STATE_DEST = 3       # 正在对准目标区域（同色色环/色区）
STATE_DROP = 4       # 目标区已就位，主控可以放
STATE_DONE = 5       # 两批都完成

# 兼容旧名字（v3.0 早期版本用的是"块/环"的说法）
STATE_BLOCK = STATE_MATERIAL
STATE_RING = STATE_DEST

# 任务码：'1'=红 '2'=绿 '3'=蓝（规则 3.3）
TASK_CODE_COLORS = {"1": "red", "2": "green", "3": "blue"}

# 两个批次
BATCH_ROUGH = "rough"   # 原料区 -> 粗加工区
BATCH_TEMP = "temp"     # 粗加工区 -> 暂存区
BATCH_TEXT = {BATCH_ROUGH: "原料区→粗加工区", BATCH_TEMP: "粗加工区→暂存区"}

STATE_TEXT = {
    STATE_SEARCH: "搜索",
    STATE_MATERIAL: "对准物料",
    STATE_GRAB: "可抓取",
    STATE_DEST: "对准目标区",
    STATE_DROP: "可放置",
    STATE_DONE: "完成",
}

# 屏幕叠加层专用：设备上的 draw_string 只有 ASCII 字库，
# 直接画中文会变成一排"?"（真机实测踩过）。所以叠加层一律用英文，
# 中文只出现在 SSH 日志里 —— 日志是 UTF-8，不受字库限制。
STATE_ASCII = {
    STATE_SEARCH: "SEARCH",
    STATE_MATERIAL: "MATERIAL",
    STATE_GRAB: "GRAB",
    STATE_DEST: "DEST",
    STATE_DROP: "DROP",
    STATE_DONE: "DONE",
}

STATUS_OK = 0x00
STATUS_FAIL = 0x01


# ============================================================ 协议实现
#
# v4：与组内《相机_主控串口通信协议.docx》字节级一致。
# 上行 20 字节、下行 4 字节；上行**没有帧序号**，坐标是**像素中心**。
# 参照实现见 pc/protocol.py，两边必须完全一样（tests/test_protocol.py 会比对）。

def uplink_checksum(payload):
    """payload = 偏移 4-14 的 11 字节，逐字节累加取低 8 位。"""
    return sum(payload) & 0xFF


def encode_rotation(angle_deg=0.0, is_ring=False):
    """把"是否色环 + 角度"压进第 6 字节。

    文档里这个字节是旋转角度，但圆柱没有旋转概念（文档也这么说），
    所以把符号位拿来传环/块标志，低 7 位放角度。
    """
    a = int(round(float(angle_deg) * ANGLE_STEPS / 360.0))
    a = 0 if a < 0 else (ANGLE_STEPS if a > ANGLE_STEPS else a)
    return (RING_BIT if is_ring else 0) | (a & 0x7F)


def to_ref(value, src, dst):
    """把任意分辨率下的像素长度换算到参考系，四舍五入并截断到 0-65535。"""
    if src <= 0:
        return 0
    n = int(round(value * float(dst) / src))
    return 0 if n < 0 else (65535 if n > 65535 else n)


def build_uplink(exists, color, rotation=0, x=0, y=0, w=0, h=0):
    """组一帧上行数据（20 字节）。exists=0 时坐标字段必须全 0。

    x/y 是**目标中心**的像素坐标，w/h 是长宽，全部按 640x480 参考系。
    """
    payload = bytes([
        exists & 0x01,
        color & 0xFF,
        rotation & 0xFF,
        x & 0xFF, (x >> 8) & 0xFF,
        y & 0xFF, (y >> 8) & 0xFF,
        w & 0xFF, (w >> 8) & 0xFF,
        h & 0xFF, (h >> 8) & 0xFF,
    ])
    return UP_HEADER + payload + bytes([uplink_checksum(payload)]) + UP_TAIL


def build_uplink_for_blob(exists, color, blob, img_w, img_h,
                          is_ring=False, angle_deg=0.0):
    """从 MaixPy 风格的 blob（x, y, w, h，左上角像素）直接组帧。

    内部会自动：把左上角换算成中心、把任意分辨率换算到 640x480 参考系。
    """
    rot = encode_rotation(angle_deg, is_ring)
    if not exists or blob is None:
        return build_uplink(0, color, rot, 0, 0, 0, 0)
    bx, by, bw, bh = blob[0], blob[1], blob[2], blob[3]
    cx = to_ref(bx + bw / 2.0, img_w, REF_W)
    cy = to_ref(by + bh / 2.0, img_h, REF_H)
    rw = to_ref(bw, img_w, REF_W)
    rh = to_ref(bh, img_w, REF_W)      # 宽高都用画面宽度归一化，和旧版保持一致
    return build_uplink(1, color, rot, cx, cy, rw, rh)


def build_downlink(cmd):
    """组一帧下行（4 字节）。主要给测试和上位机用。"""
    return DOWN_HEADER + bytes([cmd & 0xFF]) + DOWN_TAIL


def parse_downlink(frame):
    """解析一帧下行指令（4 字节），失败返回 None。"""
    if len(frame) != DOWN_FRAME_LEN:
        return None
    if frame[0:2] != DOWN_HEADER or frame[3:4] != DOWN_TAIL:
        return None
    cmd = frame[2]
    if cmd == CMD_PING:
        return {"cmd": cmd, "color": None, "cmd_name": "PING"}
    if cmd in CMD_TRACK:
        return {"cmd": cmd, "color": CMD_TRACK[cmd], "cmd_name": "TRACK"}
    return None


def norm255(value, total):
    """像素量 -> 0-255 归一化（四舍五入，越界截断）。"""
    if total <= 0:
        return 0
    n = int(value * 255.0 / total + 0.5)
    return 0 if n < 0 else (255 if n > 255 else n)


# ============================================================ 自动测色

# 未标定前用一个"什么都匹配不到"的阈值，保证不会乱追
NONE_THRESHOLD = [[0, 0, 0, 0, 0, 0]]


def lab_saturation(a_mean, b_mean):
    """LAB 的色度（A/B 平面上的半径）。越小越接近灰白，越大颜色越浓。"""
    return (a_mean * a_mean + b_mean * b_mean) ** 0.5


def classify_color(a_mean, b_mean):
    """由一块区域的 LAB 色度判断主色，返回 red / green / blue；
    分不出红绿蓝（灰、白、黄等）时返回 None。

    A 轴：负=绿，正=红；B 轴：负=蓝，正=黄。
    用 A、B 的符号判断比用角度更稳：
        A 明显为正      -> 红、橙（暖色）           -> red
        A 明显为负      -> 绿、青                   -> green
        A 居中但 B 负   -> 蓝、紫                   -> blue
        其余（A、B 都居中，或只有 B 正=黄）          -> None（不是红绿蓝）

    注意：这个分法分不开"红色物块"和"皮肤/木桌"——两者都在暖色区。
    它解决的是"不用手填阈值"，不是"人手指令"。
    """
    # 先看谁更极端：B 明显为负且压过 A，就是蓝/紫
    if b_mean <= -15.0 and abs(b_mean) > abs(a_mean):
        return "blue"
    if a_mean >= 15.0:
        return "red"      # 红、橙
    if a_mean <= -15.0:
        return "green"    # 绿、青
    return None


def threshold_from_stats(st, k=2.5, min_width=(20, 15, 15)):
    """把一块区域的 LAB 统计量变成一条阈值 [L0,L1,A0,A1,B0,B1]。

    k 是标准差倍数：以均值 ± k*σ 作为范围，并保证最小宽度，
    避免物体花纹单一时算出一个窄到"抖一下就丢"的阈值。
    """
    def clamp(v, lo, hi):
        v = int(round(v))
        return lo if v < lo else (hi if v > hi else v)

    def span(mean, sd, lo, hi, minw):
        half = max(sd * k, minw / 2.0)
        a = clamp(mean - half, lo, hi)
        b = clamp(mean + half, lo, hi)
        if b - a < minw:
            b = min(hi, a + minw)
            a = max(lo, b - minw)
        return a, b

    l0, l1 = span(st.l_mean(), st.l_std_dev(), 0, 100, min_width[0])
    a0, a1 = span(st.a_mean(), st.a_std_dev(), -128, 127, min_width[1])
    b0, b1 = span(st.b_mean(), st.b_std_dev(), -128, 127, min_width[2])
    return [l0, l1, a0, a1, b0, b1]


# 各色系的"宽松种子窗口"：先按色系把该颜色的像素筛出来，
# 再用它们的统计量算出精确阈值。L 一律全开——亮度不该参与筛选。
COLOR_SEED = {
    "red":   [0, 100, 25, 100, -30, 100],    # A 明显为正 = 红/橙
    "green": [0, 100, -128, -15, -70, 100],  # A 明显为负 = 绿/青
    "blue":  [0, 100, -70, 100, -128, -15],  # B 明显为负 = 蓝/紫
}

# 判定"这真是个有色物块"所需的最小色度（A-B 平面上的半径）。
# 种子窗口故意开得宽（用来搜索），真正把关的是这道门：
# 实测偏蓝的阴影/冷色白/瓶子标签 色度只有 25，而真正的红物块是 46~53。
# 没有这道门就会出现"放红色却框到白色"——淡色背景把面积刷上去了。
MIN_CHROMA = 30.0


def _intersect(a, b):
    return [max(a[0], b[0]), min(a[1], b[1]),
            max(a[2], b[2]), min(a[3], b[3]),
            max(a[4], b[4]), min(a[5], b[5])]


class ColorSwitchDecider:
    """连续自动测色时的"换色"判决。

    要求**连续 need 次**都判成同一个"别的颜色"才切换，中途判回原色就清零。
    这样避免物体晃一下、光照抖一下就把颜色切来切去。
    """

    def __init__(self, need=2):
        self.need = need
        self.candidate = None
        self.count = 0

    def feed(self, detected, current):
        """detected: 本次测出的颜色（None 表示没测出来）；current: 当前用的颜色。
        返回要切换到的颜色，或 None 表示维持现状。"""
        if not detected or detected == current:
            self.candidate = None
            self.count = 0
            return None
        if detected == self.candidate:
            self.count += 1
        else:
            self.candidate = detected
            self.count = 1
        if self.count >= self.need:
            self.candidate = None
            self.count = 0
            return detected
        return None


def auto_calibrate(img, img_w, img_h, pixels_threshold=500, min_area_frac=0.01):
    """在画面中心区域自动测出目标颜色并生成阈值。

    做法（比"看中心区平均色度"稳得多）：
      1. 分别用红/绿/蓝三个色系的宽松窗口去找色块，
         **谁找到的面积最大就用谁** —— 这样画面里有个偏色的背景也不会带偏。
      2. 用胜出者窗口在**该色块的框内**统计像素（只统计匹配窗口的像素），
         用 mean±kσ 生成 A/B 范围。
      3. **L（亮度）固定全开 [0,100]**：亮度随光照/距离/反光变化最大，
         一旦收窄，换个灯就不认了。MaixPy 官方文档也是这个建议。
      4. 面积太小的直接判失败，不硬塞一个坏阈值。

    返回 (color_name, threshold, note)；失败时 color_name 为 None，note 是原因。
    """
    rx, ry = int(img_w * 0.3), int(img_h * 0.3)
    rw, rh = int(img_w * 0.4), int(img_h * 0.4)
    roi = [rx, ry, rw, rh]
    min_area = max(pixels_threshold, int(rw * rh * min_area_frac))

    # 1) 三个色系各找一遍，各自算出"面积 + 色度"，只保留色度够强的
    candidates = []
    weak = []
    for name in ("red", "green", "blue"):
        seed = COLOR_SEED[name]
        try:
            blobs = img.find_blobs([seed],
                                   pixels_threshold=pixels_threshold, roi=roi)
        except Exception:
            blobs = None
        if not blobs:
            continue
        b = max(blobs, key=lambda x: x[2] * x[3])
        area = b[2] * b[3]
        if area < min_area:
            continue
        try:
            st = img.get_statistics(thresholds=[seed],
                                    roi=[b[0], b[1], b[2], b[3]])
        except Exception:
            continue
        chroma = lab_saturation(st.a_mean(), st.b_mean())
        if chroma < MIN_CHROMA:
            # 淡色背景/阴影也会匹配色系窗口，但它们色度很低，必须挡掉
            weak.append("%s(色度%.0f)" % (name, chroma))
            continue
        candidates.append((area, chroma, name, b, st))

    if not candidates:
        if weak:
            return None, None, ("中心区只有淡色（%s，需要色度≥%.0f），"
                                "把物块放近一点或换光照更好的角度"
                                % (" ".join(weak), MIN_CHROMA))
        return None, None, "中心区没看到有颜色的东西，请把物块放到画面正中间"

    # 2) 面积优先（物块通常比干扰大），面积接近时再看色度
    candidates.sort(key=lambda c: (c[0], c[1]), reverse=True)
    area, chroma, name, b, st = candidates[0]

    # 用"只含物块像素"的统计量最终确认颜色（比区域均值可靠）
    name_final = classify_color(st.a_mean(), st.b_mean()) or name

    th = _intersect(threshold_from_stats(st, k=2.5, min_width=(0, 30, 30)),
                    COLOR_SEED[name])
    th[0], th[1] = 0, 100          # 亮度全开，抗光照变化
    if th[2] > th[3] or th[4] > th[5]:
        return None, None, "算出的 A/B 范围无效，光线可能太差"

    note = "%s A均值%.0f B均值%.0f 色度%.0f 面积%d 框(%d,%d %dx%d)" % (
        name, st.a_mean(), st.b_mean(), chroma, area, b[0], b[1], b[2], b[3])
    if weak:
        note += " 已排除:" + " ".join(weak)
    return name_final, th, note


class DownlinkParser:
    """按字节喂入的下行指令解析器，自动处理粘包 / 拆包 / 重新同步。"""

    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data):
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(DOWN_HEADER)
            if i < 0:
                keep = len(DOWN_HEADER) - 1
                if len(self.buf) > keep:
                    del self.buf[: len(self.buf) - keep]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < DOWN_FRAME_LEN:
                break
            frame = bytes(self.buf[:DOWN_FRAME_LEN])
            parsed = parse_downlink(frame)
            if parsed is None:
                self.bad_frames += 1
                del self.buf[:1]
                continue
            out.append(parsed)
            del self.buf[:DOWN_FRAME_LEN]
        return out


def frame_from_blob(color_name, blob, img_w, img_h, is_ring=False):
    """把一个色块（blob）打包成 v4 上行帧。

    blob 需要支持 blob[0..3] = x, y, w, h（左上角像素，MaixPy find_blobs 格式）。
    blob 为 None 表示未检测到目标 —— 此时坐标字段全部填 0。

    返回 (帧, 像素信息字典或 None)。字典里是**原始分析分辨率**下的像素值，
    帧里是换算到 640x480 参考系之后的值，两者不一样，别混。
    """
    code = COLOR_CODE.get(color_name, 0x00)   # 自动模式尚未判定颜色时上报 0x00
    rot = encode_rotation(0.0, is_ring)
    if blob is None:
        return build_uplink(0, code, rot, 0, 0, 0, 0), None

    bx, by, bw, bh = blob[0], blob[1], blob[2], blob[3]
    frame = build_uplink_for_blob(1, code, blob, img_w, img_h, is_ring)
    info = {"x": bx, "y": by, "w": bw, "h": bh,
            "cx": bx + bw / 2.0, "cy": by + bh / 2.0, "is_ring": is_ring}
    return frame, info


# ============================================================ v3.0 目标模型
#
# 色环和物块是**同一种颜色**，只能靠形状区分：
#     物块 = 实心（正中也是这个颜色）
#     色环 = 空心（正中是地面）
#
# 判据不拟合圆、不算轮廓，只比两小块区域的 LAB 均值：
#     "环带"（框顶部中间一条）和"洞"（框正中一块）。
# 好处：斜着看压成椭圆不影响、环被压住一部分也不影响、几十行代码就能解释清楚。
#
# 另一个直觉上的坑：填充率（像素数/框面积）看着像判据，其实不能单独用 ——
# 环被斜看成细椭圆时像素数和框面积同时缩小，比值几乎不变，但薄壁环和
# 有高光的块会撞在一起。所以填充率只当"取样失败时的兜底"。

HOLE_RGB_TOL = 70.0           # 洞里的点和环带颜色的 RGB 距离小于它才算"同色"
# 填充率（= blob.density() = 像素数/框面积），2026-10-21 在真机上量的：
#   实心方块 1.000 | 实心圆片 π/4≈0.785 | 环 0.486~0.500（内径/外径≈0.6）
# 注意：真机上 solidity() 对环也返回 0.99，**不能用**（和 OpenMV 的语义不一样），
# 所以形状判据只认"中心取样"，填充率仅作取样失败时的兜底。
FILL_RING_MAX = 0.68
FILL_SOLID_MIN = 0.85
MIN_SHAPE_PX = 24             # 小于这个尺寸取样不可靠，一律当块


def rgb_dist(a, b):
    """两个 RGB 颜色的欧氏距离。"""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def hole_is_background_rgb(hole_px, band_rgb, tol=HOLE_RGB_TOL):
    """洞里的采样点和环带颜色比：多数不同色 → 中间是空的 → 这是个环。

    用 RGB 直接比，不走 LAB：这里只需要判断"是不是同一种颜色"，
    不需要精确的色相/色度，所以能用一个 0.06ms 的 get_pixel 搞定
    （get_statistics 一次要 1~4ms）。

    返回 True（空）/ False（同色）/ None（没采到样）。
    """
    if not hole_px or band_rgb is None:
        return None
    close = sum(1 for c in hole_px if rgb_dist(c, band_rgb) <= tol)
    return close * 2 < len(hole_px)


def decide_shape(hole_bg, fill=None):
    """综合取样结果判定形状，返回 "ring" / "block"。

    中心取样（实测"中间空不空"）是主判据；只有取样失败时才退回填充率。
    默认判成块：把环误判成块只是白跑一趟（到位抓不到，主控会重试），
    把块误判成环则会**永远抓不到**，代价大得多。
    """
    if hole_bg is True:
        return "ring"
    if hole_bg is False:
        return "block"
    if fill is not None:
        if fill < FILL_RING_MAX:
            return "ring"
        if fill >= FILL_SOLID_MIN:
            return "block"
    return "block"


def make_object(kind, color, rect, img_w, img_h, fill=None, extra=None):
    """把一个色块整理成统一的目标字典（像素 + 归一化，两种坐标都留着）。

    长宽都除以**画面宽度**，和协议里 w_n/h_n 的定义保持一致 ——
    这样 w_n 在任何分辨率下含义相同，主控不用改。
    """
    x, y, w, h = [int(v) for v in rect[:4]]
    cx, cy = x + w / 2.0, y + h / 2.0
    o = {"kind": kind, "color": color,
         "x": x, "y": y, "w": w, "h": h,
         "cx": cx, "cy": cy,
         "x_n": norm255(cx, img_w), "y_n": norm255(cy, img_h),
         "w_n": norm255(w, img_w), "h_n": norm255(h, img_w),
         "fill": fill}
    if extra:
        o.update(extra)
    return o


def obj_dist_n(o, point_n):
    """目标和动作点之间的归一化距离。"""
    return math.hypot(o["x_n"] - point_n[0], o["y_n"] - point_n[1])


def is_aligned(o, point_n, tol_n, min_w_n=0):
    """目标是否已经到达动作点，并且够近。

    x_n/y_n 管**方向**，w_n 管**距离**（w_n 越大越近），两者互相独立。
    """
    return (abs(o["x_n"] - point_n[0]) <= tol_n
            and abs(o["y_n"] - point_n[1]) <= tol_n
            and o["w_n"] >= min_w_n)


def _same_color_candidates(objs, color, hold_point=None, hold_ignore_n=40):
    """该颜色的候选目标，并排除车上已经抓着的那个。"""
    out = []
    for o in objs:
        if o["color"] != color:
            continue
        if hold_point is not None and obj_dist_n(o, hold_point) <= hold_ignore_n:
            continue
        out.append(o)
    return out


def pick_nearest(objs, color, point_n, hold_point=None, hold_ignore_n=40):
    """去抓物料时用：挑离抓取动作点最近的那一块。

    原料区里同一时刻只有一个该颜色的物料，所以"最近"就等于"要抓的那个"；
    远处的目标区色标即使同色，也不会比眼前的物料更近。
    """
    cands = _same_color_candidates(objs, color, hold_point, hold_ignore_n)
    if not cands:
        return None
    return min(cands, key=lambda o: obj_dist_n(o, point_n))


def pick_largest(objs, color, point_n=(128, 128),
                 hold_point=None, hold_ignore_n=40):
    """去放置时用：挑该颜色里**面积最大**的那一块。

    为什么不按形状挑：规则里的"色环"是 1.5mm 细线画的同心圆（φ50/60/70），
    在 640×480、镜头高度 300~400mm 下宽不到 2 像素，靠"空心/实心"判形状
    并不可靠；而**色环的中心和它所在颜色区域里最大同色块的中心是同一个点**，
    所以把"最大的同色块"居中，就等价于把物料放到 1 环（15 分）。
    实心色块、粗色环、细线色环三种画法都适用。

    hold_point：车上已经抓着的物料所在的图像位置，必须排除，
    否则搬运途中会对着自己抓着的物料做对准，永远到不了目标区。
    """
    cands = _same_color_candidates(objs, color, hold_point, hold_ignore_n)
    if not cands:
        return None
    return max(cands, key=lambda o: o["w"] * o["h"])


def parse_task_code(code):
    """把任务码 '123' 解析成 ('red','green','blue')。非法返回 None。

    规则：数字 1=红 2=绿 3=蓝，三位可以重复（如 '112'）。
    """
    if not isinstance(code, str):
        return None
    out = []
    for ch in code.strip():
        if ch not in TASK_CODE_COLORS:
            return None
        out.append(TASK_CODE_COLORS[ch])
    return tuple(out) if out else None


class FieldUI:
    """现场触摸界面（纯布局与状态，可单测；绘制在设备侧）。

    为什么要有它：现场很可能只有一台机器人、一块电池，没有电脑。
    所以"输任务码、学阈值、保存、开始"这些操作必须能在相机自己的
    触摸屏上完成，否则一改参数就得回去接电脑。

    屏幕 552x368（MaixCAM 2.3"），画面按屏幕尺寸缩放后 1:1 显示，
    触摸坐标也就是屏幕坐标，不做任何换算。
    """

    BUTTONS = [
        ("1", 0, 0), ("2", 1, 0), ("3", 2, 0), ("CLR", 3, 0),
        ("RUN", 0, 1), ("LEARN", 1, 1), ("SAVE", 2, 1), ("RST", 3, 1),
    ]
    LABEL = {"CLR": "CLR", "RUN": "RUN", "LEARN": "LEARN",
             "SAVE": "SAVE", "RST": "RST"}

    def __init__(self, w=552, h=368, cols=4, rows=2,
                 gap=4, row_h=46, margin=4):
        self.w, self.h = w, h
        self.code_buf = ""
        self.message = ""
        self.rects = {}
        total_h = rows * row_h + (rows - 1) * gap
        top = h - margin - total_h
        bw = (w - margin * 2 - gap * (cols - 1)) // cols
        for name, cx, cy in self.BUTTONS:
            self.rects[name] = (margin + cx * (bw + gap),
                                top + cy * (row_h + gap),
                                bw, row_h)

    # ---- 触摸命中 ----

    def hit(self, x, y):
        """返回被按到的按钮名；没按到返回 None。"""
        for name, (bx, by, bw, bh) in self.rects.items():
            if bx <= x < bx + bw and by <= y < by + bh:
                return name
        return None

    # ---- 任务码输入 ----

    def push_digit(self, d):
        """追加一位任务码，返回 True 表示已经凑满 3 位。"""
        if d not in ("1", "2", "3"):
            return False
        if len(self.code_buf) >= 3:
            return len(self.code_buf) == 3
        self.code_buf += d
        return len(self.code_buf) >= 3

    def clear(self):
        self.code_buf = ""

    @property
    def code(self):
        return self.code_buf

    @property
    def code_done(self):
        return len(self.code_buf) >= 3

    def set_message(self, text):
        self.message = text or ""


def _verify_threshold(img, th, img_w, img_h):
    """用刚算出的阈值回找一遍，确认画面中心确实还能抓到一块像样的东西。

    这是最后一道门 —— 没有它，"学出一段什么都匹配的窗口"会一路混到
    SAVE 里，然后在现场表现为"满屏一个目标 + 帧率暴跌"。
    """
    cx, cy = img_w / 2.0, img_h / 2.0
    try:
        blobs = img.find_blobs([th], pixels_threshold=200)
    except Exception:                                         # noqa: BLE001
        return False
    if not blobs:
        return False
    best = None
    for b in blobs:
        bcx, bcy = b[0] + b[2] / 2.0, b[1] + b[3] / 2.0
        d = ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5
        if best is None or d < best[0]:
            best = (d, b)
    d, b = best
    if d > min(img_w, img_h) * 0.30:
        return False                    # 中心附近根本没有，说明学歪了
    if b[2] < 8 or b[3] < 8:
        return False                    # 太小，等于没抓到
    if b[2] > img_w * 0.95 or b[3] > img_h * 0.95:
        return False                    # 满幅 = 阈值太宽，等于什么都没筛
    return True


def learn_from_center(img, img_w, img_h, k=2.5, pixels_threshold=400):
    """把画面**正中间那个物体**学成一段阈值（现场标定用，不需要电脑）。

    为什么不是"直接对中心一大块算 mean±kσ"（2026-10-21 现场证明不可用）：
        中心 200x160 里大半是桌面背景，算出来的窗口把"物体 + 背景"
        一起框了进去。当时存下来的实际值是：
            红 A-48..78  B-31..49      （A 负值是绿的地盘）
            蓝 B-124..+30              （B 正30是黄/灰的地盘，早不是蓝了）
        后果是蓝色阈值匹配了几乎整幅画面 —— 上报目标的 wn 变成满值 255、
        目标在"色标/物料/无"之间乱跳、帧率从 15 掉到 6。

    现在的做法（和 auto_calibrate 同一套把关）：
      1. 用红/绿/蓝三个色系种子分别找色块，只认**离画面中心最近**的那块。
      2. 只用**该色块自己的框内像素**算统计量（thresholds=[seed]），
         背景完全不参与。
      3. 色度 < MIN_CHROMA 直接判失败（淡色背景挡掉）。
      4. 与色系种子取交集、亮度强制全开 [0,100]，防止学到别的色系。
      5. 用学到的阈值回找一遍做自检，中心找不回来就判失败。

    返回 (颜色名, 6 个数阈值, 说明)；失败时颜色名为 None，说明是原因。
    说明文字只用 ASCII —— 屏幕上画不了中文。
    """
    cx, cy = img_w / 2.0, img_h / 2.0
    rx, ry = int(img_w * 0.3), int(img_h * 0.3)
    rw, rh = int(img_w * 0.4), int(img_h * 0.4)
    roi = [rx, ry, rw, rh]
    near = min(img_w, img_h) * 0.25

    best = None
    weak = []
    for name in ("red", "green", "blue"):
        seed = COLOR_SEED[name]
        try:
            blobs = img.find_blobs([seed], pixels_threshold=pixels_threshold,
                                   roi=roi)
        except Exception:                                     # noqa: BLE001
            blobs = None
        for b in blobs or []:
            bw, bh = b[2], b[3]
            if bw < 8 or bh < 8:
                continue
            bcx, bcy = b[0] + bw / 2.0, b[1] + bh / 2.0
            dist = ((bcx - cx) ** 2 + (bcy - cy) ** 2) ** 0.5
            if dist > near:
                continue                      # 跑到画面边上的不算"正中间"
            try:
                st = img.get_statistics(thresholds=[seed],
                                        roi=[b[0], b[1], bw, bh])
            except Exception:                                 # noqa: BLE001
                continue
            try:
                if st.pixel_count() < 50:      # 有些老版本没有 pixel_count()
                    continue
            except Exception:                                 # noqa: BLE001
                pass
            chroma = lab_saturation(st.a_mean(), st.b_mean())
            if chroma < MIN_CHROMA:
                weak.append("%s chroma %.0f" % (name, chroma))
                continue
            score = (dist, -(bw * bh), name)   # 先比谁更靠中心，再比谁更大
            if best is None or score < best[0]:
                best = (score, name, b, st, chroma, dist)

    if best is None:
        if weak:
            return None, None, "TOO PALE (%s) need chroma>=%d" % (
                ", ".join(weak[:2]), int(MIN_CHROMA))
        return None, None, "NO OBJECT AT CENTER"

    _, name, b, st, chroma, dist = best
    name_final = classify_color(st.a_mean(), st.b_mean()) or name
    th = _intersect(threshold_from_stats(st, k=k, min_width=(0, 30, 30)),
                    COLOR_SEED[name])
    th[0], th[1] = 0, 100                     # 亮度全开，抗光照变化
    if th[2] > th[3] or th[4] > th[5]:
        return None, None, "BAD RANGE (lighting?)"

    if not _verify_threshold(img, th, img_w, img_h):
        return None, None, "SELF-CHECK FAIL"

    note = "%s L0-100 A%d-%d B%d-%d ch%.0f sz%dx%d" % (
        name_final, th[2], th[3], th[4], th[5], chroma, b[2], b[3])
    return name_final, th, note


def draw_field_ui(canvas, ui, image, line1, line2, msg, running):
    """把触摸界面画到已经缩放到屏幕尺寸的画布上。

    按钮坐标和触摸坐标同源（都是屏幕坐标），所以按下就能命中，
    不需要任何换算。

    布局（552x368）：
        y   0..52   大号任务码（scale 3 ≈ 48px ≈ 5mm，满足规则"字高≥4mm"）
        y  52..56   分隔线
        y  58..    相机画面 + 目标框
        y 268..368  10 个按钮（5 列 x 2 行）：
                    1 2 3 CLR RUN / LEARN SAVE G-SZ D-SZ RST
    """
    try:
        canvas.draw_string(8, 4, line1, image.COLOR_YELLOW, 3)
        canvas.draw_string(250, 8, line2, image.COLOR_WHITE, 2)
        if msg:
            canvas.draw_string(250, 32, msg, image.COLOR_YELLOW, 1)
        canvas.draw_rect(0, 0, canvas.width(), 54, image.COLOR_WHITE, 1)

        for name, (x, y, w, h) in ui.rects.items():
            if name in ("1", "2", "3"):
                col = image.COLOR_YELLOW
            elif name == "RUN":
                col = image.COLOR_GREEN if running else image.COLOR_RED
            else:
                col = image.COLOR_WHITE
            canvas.draw_rect(x, y, w, h, col, 2)
            canvas.draw_string(x + 14, y + 12, ui.LABEL.get(name, name), col, 2)
    except Exception as e:                                    # noqa: BLE001
        print("[WARN] 画触摸界面失败：%s" % e)


class Mission:
    """搬运任务状态机（纯逻辑，不碰硬件，可以直接单测）。

    按《附件1-智能搬运比赛规则》实现的两批流程：

        第一批（A）原料区 → 粗加工区：按任务码顺序，逐个搬运
        第二批（B）粗加工区 → 暂存区：按**同样的顺序**再搬一遍

    每一步只有两段：
        对准物料 → 可抓取 --NEXT--> 对准同色目标区 → 可放置 --NEXT--> 下一个

    分工：相机是眼睛，主控是手。相机只报"该盯谁、到位没有"，
    夹爪动作由主控执行，做完发 CMD_NEXT，这里才推进。

    ⚠️ v4 起协议的架构改成"主控做流程"，这套状态机**默认不启用**，
    只有命令行加 --mission 才会跑，而且只驱动本地屏幕叠加层/日志，
    不再往串口帧里塞阶段号（v4 没有那个字段）。保留它是因为：
    规则理解、本地演示、以及单元测试都还靠它。
    """

    def __init__(self, order=("red", "green", "blue"),
                 grab_point=(128, 128), drop_point=(128, 128),
                 tol_n=12, grab_w_n=90, drop_w_n=110, hold_frames=3,
                 hold_point=None, hold_ignore_n=40):
        self.order = list(order)
        self.grab_point = (int(grab_point[0]), int(grab_point[1]))
        self.drop_point = (int(drop_point[0]), int(drop_point[1]))
        self.tol_n = tol_n
        self.grab_w_n = grab_w_n
        self.drop_w_n = drop_w_n
        self.hold_frames = max(1, int(hold_frames))
        self.hold_point = hold_point
        self.hold_ignore_n = hold_ignore_n
        self.reset()

    def reset(self):
        self.batch = BATCH_ROUGH   # 当前批次；None = 全部完成
        self.index = 0             # 当前批次里第几个颜色
        self.phase = "src"         # src（去抓） -> dst（去放）
        self.done = set()          # 本批次已完成的颜色
        self.state = STATE_SEARCH
        self._hold = 0
        self.last = None
        self.action_count = 0

    # ---------------- 派生属性 ----------------

    @property
    def color(self):
        """当前这一步的物料颜色（由任务码顺序决定）。"""
        if self.batch is None or self.index >= len(self.order):
            return None
        return self.order[self.index]

    @property
    def is_done(self):
        return self.batch is None

    @property
    def progress(self):
        """给屏幕/日志用的进度，例如 A2/3、B1/3。"""
        if self.batch is None:
            return "AB完成"
        return "%s%d/%d" % ("A" if self.batch == BATCH_ROUGH else "B",
                            self.index + 1, len(self.order))

    # ---------------- 主控回调 ----------------

    def advance(self):
        """主控抓取/放置完成，推进到下一阶段。返回新阶段名（便于日志）。"""
        if self.batch is None:
            return None
        if self.phase == "src":
            self.phase = "dst"                 # 抓到了，去找同色目标区
        else:
            self.done.add(self.order[self.index])
            self.index += 1                    # 这一色搬完
            self.phase = "src"
            if self.index >= len(self.order):
                if self.batch == BATCH_ROUGH:
                    self.batch = BATCH_TEMP    # 第一批搬完，开始第二批
                    self.index = 0
                    self.done = set()
                else:
                    self.batch = None          # 两批都完成
                    self.index = 0
        self.action_count += 1
        self._hold = 0
        return self.phase

    def abort(self):
        """主控放弃当前动作：回到"去抓"阶段，颜色/进度不变。"""
        self.phase = "src"
        self._hold = 0

    # ---------------- 每帧调用 ----------------

    def update(self, objs):
        """喂一帧检测结果，返回 {"state","color","target","note","batch","progress"}。"""
        color = self.color
        if color is None:
            self.state = STATE_DONE
            self._hold = 0
            self.last = {"state": self.state, "color": None, "target": None,
                         "note": "两批搬运全部完成", "batch": None,
                         "progress": self.progress}
            return self.last

        if self.phase == "src":
            # 去抓：离抓取动作点最近的那个（原料区该颜色只有一个物料）
            tgt = pick_nearest(objs, color, self.grab_point,
                               self.hold_point, self.hold_ignore_n)
        else:
            # 去放：该颜色里面积最大的那个（色标/色环比物料大得多）
            tgt = pick_largest(objs, color, self.drop_point,
                               self.hold_point, self.hold_ignore_n)
        if tgt is None:
            return self._no_target(color, "等待 %s 的目标" % color)

        if self.phase == "src":
            if is_aligned(tgt, self.grab_point, self.tol_n, self.grab_w_n):
                return self._settle(STATE_GRAB, tgt, "物料已就位，可以抓")
            return self._settle(STATE_MATERIAL, tgt, "对准物料")

        if is_aligned(tgt, self.drop_point, self.tol_n, self.drop_w_n):
            return self._settle(STATE_DROP, tgt, "目标区已就位，可以放")
        return self._settle(STATE_DEST, tgt, "对准目标区")

    # ---------------- 内部 ----------------

    def _no_target(self, color, note):
        """没有目标时立刻报 SEARCH —— 不给主控一个"有位标"的假状态。"""
        self.state = STATE_SEARCH
        self._hold = 0
        self.last = {"state": self.state, "color": color, "target": None,
                     "note": note, "batch": self.batch,
                     "progress": self.progress}
        return self.last

    def _settle(self, state, target, note):
        """状态连续稳 hold_frames 帧才对外发布，避免边界抖动让主控反复启停。"""
        if state == self.state:
            self._hold = 0
        else:
            self._hold += 1
            if self._hold >= self.hold_frames:
                self.state = state
                self._hold = 0
        published = (self.state == state)
        self.last = {"state": self.state,
                     "color": self.color,
                     # 还没发布的状态不能带目标，否则主控会拿到"半路"的位置
                     "target": target if published else None,
                     "note": note if published
                             else "确认中(->%s)" % STATE_TEXT.get(state, "?"),
                     "batch": self.batch,
                     "progress": self.progress}
        return self.last


def rect_iou(a, b):
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    x1, y1 = max(ax, bx), max(ay, by)
    x2 = min(ax + aw, bx + bw)
    y2 = min(ay + ah, by + bh)
    iw, ih = x2 - x1, y2 - y1
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    union = aw * ah + bw * bh - inter
    return inter / float(union) if union > 0 else 0.0


def dedup_objects(objs, iou_thresh=0.6):
    """同一个物体被多条阈值范围重复匹配时只留一个（留面积大的）。

    一条颜色的阈值可以写成多段（比如块用塑料红、环用纸胶带红），
    两段都命中同一个物体时就会产生重复目标，必须去重。
    """
    kept = []
    for o in sorted(objs, key=lambda o: o["w"] * o["h"], reverse=True):
        if any(rect_iou((o["x"], o["y"], o["w"], o["h"]),
                        (k["x"], k["y"], k["w"], k["h"])) > iou_thresh
               for k in kept):
            continue
        kept.append(o)
    return kept


# ============================================================ 图像侧取样

def blob_fill(blob, rect):
    """填充率 = 匹配像素数 / 框面积。

    优先用 MaixPy 自带的 density()（实测 0.03ms，比调 pixels() 自己算还快），
    没有这个接口才退回 pixels()/count()/area()。
    """
    for name in ("density", "extent"):
        f = getattr(blob, name, None)
        if callable(f):
            try:
                v = float(f())
                if 0.0 <= v <= 1.0:
                    return v
            except Exception:                                 # noqa: BLE001
                pass
    x, y, w, h = rect
    if w <= 0 or h <= 0:
        return None
    n = None
    for name in ("pixels", "count", "area"):
        f = getattr(blob, name, None)
        if callable(f):
            try:
                n = int(f())
                break
            except Exception:                                 # noqa: BLE001
                continue
    if n is None:
        return None
    return min(1.0, n / float(w * h))


def px_rgb(img, x, y):
    """读一个像素并统一成 (r,g,b)。MaixPy 的 get_pixel 返回形式不统一，这里兜住。"""
    v = img.get_pixel(int(x), int(y))
    if isinstance(v, (list, tuple)):
        if len(v) >= 3:
            return (int(v[0]), int(v[1]), int(v[2]))
        if not v:
            return None
        v = v[0]
    if isinstance(v, int):
        return ((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)
    return None


def probe_hole(img, rect, min_px=MIN_SHAPE_PX, tol=HOLE_RGB_TOL):
    """在色块中间和边缘各取几个点，判断中间是不是空的。

    取点策略（都不用 get_statistics，只要 get_pixel）：
        环带：左右边中点 + 上下边中点，各离边框 6%（环的四个"极点"都在带内）
        洞  ：正中 + 上下左右各偏 12%，共 5 个点，多数表决

    为什么环带取"四个极点"而不是顶部一条线：环在极点处带最厚，
    顶部一条线在圆弧上很容易偏出去采到背景，中位数就被带歪了。
    """
    x, y, w, h = rect
    if w < min_px or h < min_px:
        return None, {"probe": "太小(%dx%d)" % (w, h)}

    cx = x + w // 2
    cy = y + h // 2
    ex = max(2, int(w * 0.06))
    ey = max(2, int(h * 0.06))
    bands = []
    for px_, py_ in ((x + ex, cy), (x + w - ex, cy),
                     (cx, y + ey), (cx, y + h - ey)):
        c = px_rgb(img, px_, py_)
        if c:
            bands.append(c)
    if not bands:
        return None, {"probe": "环带取样失败"}

    # 取中位数当环带代表色：四个点里被背景/高光污染一两个也不影响
    band = tuple(sorted(c[i] for c in bands)[len(bands) // 2] for i in range(3))

    dx = max(2, int(w * 0.12))
    dy = max(2, int(h * 0.12))
    pts = [(cx, cy), (cx - dx, cy), (cx + dx, cy), (cx, cy - dy), (cx, cy + dy)]
    holes = []
    for px_, py_ in pts:
        c = px_rgb(img, px_, py_)
        if c:
            holes.append(c)
    if not holes:
        return None, {"probe": "洞取样失败"}

    hole_bg = hole_is_background_rgb(holes, band, tol)
    same = sum(1 for c in holes if rgb_dist(c, band) <= tol)
    return hole_bg, {"probe": "洞%d/%d点同色" % (same, len(holes)),
                     "band_rgb": band}


def scan_objects(img, img_w, img_h, pixels_threshold=None,
                 x_stride=2, y_stride=2):
    """扫全画面，返回所有候选目标（块和环混在一起，各自带 kind）。

    每色一趟 find_blobs（实测 15~24ms/色，是整条链路的瓶颈）。
    形状判定用 get_pixel 取样：0.06ms/点，一个目标 8 个点 ≈ 0.5ms，
    比 get_statistics 便宜 70 倍，所以**每个目标都实测**，不做近似。
    """
    pt = pixels_threshold or CONFIG["pixels_threshold"]
    objs = []
    for color in THRESHOLDS:
        try:
            blobs = img.find_blobs(THRESHOLDS[color], pixels_threshold=pt,
                                   area_threshold=pt, merge=False,
                                   x_stride=x_stride, y_stride=y_stride)
        except Exception as e:                                # noqa: BLE001
            print("[WARN] 找 %s 色块失败：%s" % (color, e))
            continue
        for b in (blobs or []):
            rect = (int(b[0]), int(b[1]), int(b[2]), int(b[3]))
            fill = blob_fill(b, rect)
            hole_bg, info = probe_hole(img, rect)
            kind = decide_shape(hole_bg, fill)
            objs.append(make_object(kind, color, rect, img_w, img_h, fill, info))
    return dedup_objects(objs)


def frame_from_object(obj, color=None, img_w=REF_W, img_h=REF_H):
    """把目标打包成 v4 上行帧。obj=None 表示没有目标。

    obj 里的 cx/cy/w/h 是**分析分辨率下的像素**，这里统一换算到 640x480 参考系；
    环/块标志走第 6 字节的 bit7（协议扩展，见文件头的说明）。
    """
    if obj is None:
        code = COLOR_CODE.get(color, 0x00) if color else 0x00
        return build_uplink(0, code, encode_rotation(0.0, False), 0, 0, 0, 0), None
    code = COLOR_CODE.get(obj["color"], 0x00)
    ring = bool(obj.get("kind") == "ring")
    frame = build_uplink(
        1, code, encode_rotation(0.0, ring),
        to_ref(obj["cx"], img_w, REF_W),
        to_ref(obj["cy"], img_h, REF_H),
        to_ref(obj["w"], img_w, REF_W),
        to_ref(obj["h"], img_w, REF_W))
    return frame, obj


# ============================================================ 配置读写

def load_config(path):
    """读取上次 SAVE 的配置，失败就用默认值。

    除了颜色和帧率，还支持 save_image / save_every：
    把这两个写进配置文件，开机自启的实例也会定期存调试图 ——
    现场排障时不用连电脑、不用改代码，写个文件就能让设备"自己交作业"。
    """
    try:
        # utf-8-sig：Windows 记事本/PowerShell 保存的 JSON 常带 BOM，
        # 用 utf-8 直接读会解析失败、静默回退默认值（现场极容易踩）。
        with open(path, "r", encoding="utf-8-sig") as f:
            saved = json.load(f)
        color = saved.get("color")
        fps = saved.get("fps")
        out = {}
        if color in THRESHOLDS or color == "auto":
            out["color"] = color
        if isinstance(fps, int) and 1 <= fps <= 60:
            out["fps"] = fps
        si = saved.get("save_image")
        if isinstance(si, str) and si:
            out["save_image"] = si
        se = saved.get("save_every")
        if isinstance(se, (int, float)) and se >= 0.5:
            out["save_every"] = float(se)
        for k in ("lock_ae", "lock_awb", "probe_lab"):
            if isinstance(saved.get(k), bool):
                out[k] = saved[k]
        for k in ("auto_continuous", "mission", "ui"):
            if isinstance(saved.get(k), bool):
                out[k] = saved[k]
        th = saved.get("threshold")
        if (isinstance(th, list) and len(th) == 6
                and all(isinstance(x, int) for x in th)):
            out["threshold"] = th
        # 搬运模式的动作点：现场标定一次就写进配置，掉电不丢
        for k in ("grab_point", "drop_point"):
            v = saved.get(k)
            if (isinstance(v, list) and len(v) == 2
                    and all(isinstance(x, int) and 0 <= x <= 255 for x in v)):
                out[k] = list(v)
        for k in ("grab_w_n", "drop_w_n"):
            v = saved.get(k)
            if isinstance(v, int) and 0 <= v <= 255:
                out[k] = v
        # 任务码与"车上物料位置"：比赛现场改一次就掉电保留
        tc = saved.get("task_code")
        if isinstance(tc, str) and parse_task_code(tc):
            out["task_code"] = tc
        hp = saved.get("hold_point")
        if (isinstance(hp, list) and len(hp) == 2
                and all(isinstance(x, int) and 0 <= x <= 255 for x in hp)):
            out["hold_point"] = list(hp)
        # 现场调阈值：直接在配置文件里写，不用重新上传程序
        #   "thresholds": {"red": [[0,100,40,110,5,95]], ...}
        ths = saved.get("thresholds")
        if isinstance(ths, dict):
            clean = {}
            for c, ranges in ths.items():
                if c not in THRESHOLDS or not isinstance(ranges, list):
                    continue
                ok_ranges = []
                for r in ranges:
                    if (isinstance(r, list) and len(r) == 6
                            and all(isinstance(x, int) for x in r)):
                        ok_ranges.append([int(x) for x in r])
                if ok_ranges:
                    clean[c] = ok_ranges
            if clean:
                out["thresholds"] = clean
        return out
    except Exception:
        return {}


def save_config(path, color, fps, save_image=None, save_every=None, opts=None,
                threshold=None, extra=None):
    try:
        d = os.path.dirname(path)
        if d and not os.path.exists(d):
            os.makedirs(d, exist_ok=True)
        data = {"color": color, "fps": fps, "version": PROTOCOL_VERSION}
        if color == "mission":
            # 搬运模式下没有"追踪色"，别把上次存的好颜色覆盖掉
            data.pop("color")
        if save_image:
            data["save_image"] = save_image
            data["save_every"] = save_every if save_every else 3.0
        if threshold and len(threshold) == 6:
            # 把自动标定出来的阈值一起存下来：换场地重新标一次，掉电不丢
            data["threshold"] = [int(x) for x in threshold]
        if opts:
            # 把锁定/探针这些开关也一起持久化，比赛现场改一次就够
            for k in ("lock_ae", "lock_awb", "mission", "ui"):
                data[k] = bool(opts.get(k))
        if extra:
            data.update(extra)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f)
        return True
    except Exception as e:
        print("[WARN] 保存配置失败:", e)
        return False


# ============================================================ 主程序

def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]
    opts = parse_args(argv)

    # maix 相关导入放在 main 里：协议层可脱离硬件做测试
    from maix import app, camera, display, err, image, pinmap, time, \
        touchscreen, uart

    print("=" * 48)
    print("%s  协议 %s" % (PROGRAM_NAME, PROTOCOL_VERSION))
    print("=" * 48)
    print("[INFO] 运行参数: %s" % opts)

    img_w, img_h = CONFIG["img_w"], CONFIG["img_h"]
    color_name = CONFIG["default_color"]
    send_fps = CONFIG["send_fps"]

    saved = load_config(CONFIG["config_path"])
    if saved:
        color_name = saved.get("color", color_name)
        send_fps = saved.get("fps", send_fps)
        print("[INFO] 已加载保存的配置:", saved)
    else:
        print("[INFO] 无保存配置，使用默认值")

    # 命令行参数优先级最高；没给就用配置文件里的
    if opts["color"]:
        color_name = opts["color"]
    if opts["fps"]:
        send_fps = opts["fps"]
    if not opts["save_image"] and saved.get("save_image"):
        opts["save_image"] = saved["save_image"]
        opts["save_every"] = saved.get("save_every", opts["save_every"])
    if opts["save_image"]:
        print("[INFO] 调试图：每 %.1f 秒存一次 -> %s" %
              (opts["save_every"], opts["save_image"]))
    for k in ("lock_ae", "lock_awb", "probe_lab"):
        if not opts[k] and saved.get(k):
            opts[k] = True

    # 配置文件里可以覆盖颜色阈值（现场调参不用重新上传程序）
    if saved.get("thresholds"):
        for c, ranges in saved["thresholds"].items():
            THRESHOLDS[c] = [list(r) for r in ranges]
        print("[INFO] 已用配置文件里的阈值覆盖：%s" % sorted(saved["thresholds"]))

    # ---- 触摸界面 ----
    if saved.get("ui"):
        opts["ui"] = True

    # ---- 搬运任务状态机（v4 起**默认关闭**）----
    # 架构决定：搬运流程逻辑放在 STM32，相机只负责"看见并上报"。
    # 所以这里不再从配置文件里自动打开 mission —— 配置里残留的 "mission": true
    # 一律忽略，只有命令行显式 --mission 才会启用（本地演示/单测用）。
    # 否则老配置会让相机跑状态机、上报的又是不带阶段信息的 v4 帧，现场极难排查。
    if saved.get("mission") and not opts["mission"]:
        print("[WARN] 配置里的 mission=true 已忽略（v4 由主控负责流程）；"
              "要本地演示请加 --mission")
        opts["mission"] = False
    mission = None
    task_code = opts["task_code"] or saved.get("task_code") or CONFIG["task_code"]

    # 动作点等参数：命令行 > 配置文件 > 代码里的默认值
    gp = opts["grab_point"] or saved.get("grab_point") or CONFIG["grab_point_n"]
    dp = opts["drop_point"] or saved.get("drop_point") or CONFIG["drop_point_n"]
    gw = opts["grab_w"] or saved.get("grab_w_n") or CONFIG["grab_w_n"]
    dw = opts["drop_w"] or saved.get("drop_w_n") or CONFIG["drop_w_n"]
    hold = opts["hold_point"] or saved.get("hold_point") or CONFIG["hold_point_n"]

    def make_mission(order):
        """按给定顺序建一个任务状态机（触摸界面改任务码时会重新建）。"""
        return Mission(
            order=order,
            grab_point=gp, drop_point=dp,
            tol_n=CONFIG["align_tol_n"],
            grab_w_n=gw, drop_w_n=dw,
            hold_point=tuple(hold) if hold else None,
            hold_ignore_n=CONFIG["hold_ignore_n"])

    if opts["mission"]:
        order = parse_task_code(task_code)
        if order is None:
            print("[WARN] 任务码 %r 非法（只能是 1/2/3 组合），按红绿蓝顺序执行"
                  % (task_code,))
            order = tuple(COLOR_CODE.keys())

        mission = make_mission(order)
        print("[INFO] 搬运任务：任务码=%s -> 顺序 %s"
              % (task_code, "→".join(order)))
        print("[INFO] 流程：原料区→粗加工区（3 个），再 粗加工区→暂存区（同样顺序）")
        print("[INFO] 抓取动作点(%d,%d) 放置动作点(%d,%d) 容差±%d "
              "抓取 w_n≥%d 放置 w_n≥%d"
              % (mission.grab_point[0], mission.grab_point[1],
                 mission.drop_point[0], mission.drop_point[1],
                 mission.tol_n, mission.grab_w_n, mission.drop_w_n))
        if mission.hold_point:
            print("[INFO] 车上物料位置(%d,%d) 半径 %d 内的同色目标会被忽略"
                  % (mission.hold_point[0], mission.hold_point[1],
                     mission.hold_ignore_n))
        # 注意：v4 协议里没有状态字段，也没有 NEXT 指令。这套状态机现在
        # 只在本地演示（屏幕叠加层 + 日志）时有用，不会出现在串口帧里。
        print("[INFO] 本地演示模式：状态机只驱动屏幕叠加层，"
              "串口上报的仍是 v4 的「一个目标 + 环/块标志」")

    # ---- 显示：待机/运行状态 ----
    standby = False

    # ---- 串口 ----
    if opts["no_uart"]:
        serial = None
        print("[INFO] --no-uart：跳过串口初始化（只验视觉）")
    else:
        # 先把引脚复用成 UART1（MaixCAM 默认串口是 UART0，UART1 需要显式映射）
        for pin, func in ((CONFIG["pin_tx"], "UART1_TX"), (CONFIG["pin_rx"], "UART1_RX")):
            try:
                err.check_raise(pinmap.set_pin_function(pin, func),
                                "set pin %s -> %s failed" % (pin, func))
            except Exception as e:
                print("[WARN] 设置 %s 为 %s 失败：%s（若串口本可用可忽略）" % (pin, func, e))

        serial = uart.UART(CONFIG["uart_device"], CONFIG["uart_baud"])
        print("[INFO] 串口：%s @ %d 8N1" % (CONFIG["uart_device"], CONFIG["uart_baud"]))

    parser = DownlinkParser()

    # ---- 摄像头与屏幕 ----
    cam = camera.Camera(img_w, img_h)

    # ---- 锁定曝光 / 白平衡 ----
    # 阈值法视觉最关键的一步。不锁的话 AE/AWB 会随着画面内容持续调整，
    # 同一个物块的 LAB 值会慢慢漂出阈值范围，表现就是
    # "一开始能识别，过一会儿就完全认不出来了"。
    if opts["lock_ae"] or opts["lock_awb"]:
        print("[INFO] 等待 AE/AWB 收敛 2.5 秒……")
        time.sleep(2.5)
        cam.clear_buff()
        if opts["lock_ae"]:
            try:
                exp = cam.exposure()
                gn = cam.gain()
                cam.exp_mode(1)                 # 1 = 手动
                if exp and exp > 0:
                    cam.exposure(exp)
                if gn and gn > 0:
                    cam.gain(gn)
                print("[INFO] 已锁定曝光：exposure=%s us  gain=%s" % (exp, gn))
            except Exception as e:
                print("[WARN] 锁定曝光失败：%s" % e)
        if opts["lock_awb"]:
            try:
                m = cam.set_awb(0)              # 0 = 手动，锁住当前白平衡
                print("[INFO] 已锁定白平衡：set_awb(0) -> %s" % m)
                print("[WARN] 注意：本机实测切手动白平衡后增益会变成 1:1:1，"
                      "画面会整片偏绿。若发现颜色不对，去掉 lock_awb 重启。")
            except Exception as e:
                print("[WARN] 锁定白平衡失败：%s" % e)

    if opts["no_display"] and not opts["ui"]:
        disp = None
        print("[INFO] --no-display：不初始化屏幕（SSH 调试用）")
    else:
        disp = display.Display()

    # ---- 现场触摸界面（没电脑也能操作）----
    ts = None
    ui = None
    ui_dw, ui_dh = 0, 0
    ui_pressed = False
    if opts["ui"]:
        try:
            ts = touchscreen.TouchScreen()
        except Exception as e:                                # noqa: BLE001
            print("[WARN] 触摸屏初始化失败：%s（界面只显示不响应）" % e)
        try:
            ui_dw, ui_dh = disp.width(), disp.height()
        except Exception:                                     # noqa: BLE001
            ui_dw, ui_dh = 552, 368
        ui = FieldUI(ui_dw, ui_dh)
        if len(task_code) == 3 and parse_task_code(task_code):
            ui.code_buf = task_code
        ui_msg_t = 0.0
        ui.set_message("")
        print("[INFO] 触摸界面 %dx%d：1/2/3 输任务码、RUN 暂停、"
              "LEARN 学阈值、SAVE 保存、RST 重置" % (ui_dw, ui_dh))

        def _ui_action(name):
            """触摸按钮处理。现场没有电脑，这些操作必须能在屏幕上完成。"""
            nonlocal mission, standby, task_code, ui_msg_t

            def say(text):
                """显示一条操作反馈（3 秒后自动回到实时信息）。"""
                nonlocal ui_msg_t
                ui.set_message(text)
                ui_msg_t = time.time()

            if name in ("1", "2", "3"):
                if ui.push_digit(name):
                    order2 = parse_task_code(ui.code_buf)
                    if order2:
                        task_code = ui.code_buf
                        mission = make_mission(order2)
                        say("CODE %s OK" % task_code)
                        print("[UI] 任务码 -> %s" % task_code)
                    else:
                        say("BAD CODE")
                else:
                    say("CODE %s_" % ui.code_buf)
                return

            if name == "CLR":
                ui.clear()
                say("CLR")
            elif name == "RUN":
                standby = not standby
                say("PAUSE" if standby else "RUN")
            elif name == "LEARN":
                nm, th, note = learn_from_center(img, img_w, img_h)
                if th:
                    THRESHOLDS[nm] = [th]
                    say("LEARNED " + note)
                    print("[UI] 学到 %s" % note)
                else:
                    say(note)
                    print("[UI] 学习失败：%s" % note)
            elif name == "SAVE":
                # v4：流程由主控负责，所以 mission 写 false（否则下次开机
                # 又会打开本地状态机，上报的帧却不带阶段信息，现场很难排查）。
                extra = {"mission": mission is not None, "ui": True,
                         "thresholds": {c: [list(r) for r in THRESHOLDS[c]]
                                        for c in THRESHOLDS}}
                if mission is not None:
                    extra["grab_point"] = list(mission.grab_point)
                    extra["drop_point"] = list(mission.drop_point)
                    extra["grab_w_n"] = mission.grab_w_n
                    extra["drop_w_n"] = mission.drop_w_n
                    if mission.hold_point:
                        extra["hold_point"] = list(mission.hold_point)
                ok = save_config(CONFIG["config_path"], color_name, send_fps,
                                 opts["save_image"], opts["save_every"], opts,
                                 None, extra)
                say("SAVED" if ok else "SAVE FAIL")
                print("[UI] SAVE -> %s" % ok)
            elif name == "RST":
                if mission is not None:
                    mission.reset()
                say("RESET")
                print("[UI] 进度重置")

    seq = 0
    frame_count = 0
    fps_t0 = time.time()
    fps_now = 0.0
    period_ms = int(1000.0 / send_fps)
    last_info = None
    mission_objs = []        # 搬运模式：本帧所有目标
    mission_dec = None       # 搬运模式：状态机本帧决策
    t_start = time.time()
    sent_frames = 0
    loop_frames = 0
    rx_bytes = 0          # 串口收到的字节数（配合回环线可验证 UART 电气通路）
    rx_frames = 0         # 收到的合法下行指令数
    t_last_save = 0.0
    t_last_probe = 0.0

    # ---- 阈值与自动测色模式 ----
    auto_mode = (color_name == "auto") and mission is None
    if mission is not None:
        color_name = "mission"
        active_threshold = NONE_THRESHOLD   # 搬运模式不用单一颜色阈值
        print("[INFO] 搬运模式：单色追踪 / 自动测色均已停用")
    elif auto_mode:
        active_threshold = NONE_THRESHOLD   # 未标定前什么都不追，避免乱抓
        print("[INFO] 自动测色模式：把物块放到画面正中间，相机会自己判断颜色并算出阈值")
    elif opts["test_full"]:
        active_threshold = FULL_RANGE_THRESHOLD
        print("[WARN] --test-full：使用全范围阈值，会匹配整个画面（仅用于自检）")
    elif saved.get("threshold"):
        active_threshold = [saved["threshold"]]
        print("[INFO] 使用上次自动标定并保存的阈值：%s" % saved["threshold"])
    else:
        active_threshold = THRESHOLDS[color_name]

    calibrated = not (auto_mode and not saved.get("threshold"))
    t_last_cal = 0.0
    t_last_recal = 0.0

    # 连续自动测色：每隔 2 秒重判一次，换物块时自己跟着换颜色。
    # 关掉的办法：命令行 --auto-once，或配置里写 "auto_continuous": false
    # （比赛时建议关掉，锁死一个颜色更稳，不会被画面里别的颜色带跑）
    auto_continuous = (auto_mode and not opts["auto_once"]
                       and saved.get("auto_continuous", True))
    decider = ColorSwitchDecider(need=2)
    t_last_scan = 0.0
    if auto_mode:
        print("[INFO] 连续自动测色：%s" %
              ("开启（换物块会自动跟着换色）" if auto_continuous
               else "关闭（只在开机测一次，之后锁死）"))

    print("[INFO] 追踪颜色：%s  图像：%dx%d  发送帧率：%d" %
          (color_name, img_w, img_h, send_fps))
    print("[INFO] 等待主控命令……（无命令时按默认颜色持续上报）")

    while not app.need_exit():
        loop_t0 = time.time()
        loop_frames += 1

        # 限时退出：设备上没有 timeout 命令，所以限时由程序自己实现，
        # 这样能走正常退出流程，不会把摄像头/显示驱动留在半开状态。
        if opts["duration"] and (loop_t0 - t_start) >= opts["duration"]:
            print("[INFO] 到达 --duration %.1f 秒，正常退出" % opts["duration"])
            break

        # ---------------- 1. 采集与识别 ----------------
        img = cam.read()

        # 重新标定触发：SSH 里 touch /root/recalibrate 即可，不用重启
        if mission is None and CONFIG["recal_file"] and (loop_t0 - t_last_recal) >= 1.0:
            t_last_recal = loop_t0
            try:
                if os.path.exists(CONFIG["recal_file"]):
                    os.remove(CONFIG["recal_file"])
                    auto_mode = True
                    calibrated = False
                    active_threshold = NONE_THRESHOLD
                    print("[CMD] 收到重新标定请求：请把物块放到画面正中间")
            except Exception as e:
                print("[WARN] 处理重新标定请求失败：%s" % e)

        # 自动测色：还没标定就每 0.5 秒试一次，标定成功后自动进入追踪
        if auto_mode and not calibrated and (loop_t0 - t_last_cal) >= 0.5:
            t_last_cal = loop_t0
            try:
                name, th, note = auto_calibrate(img, img_w, img_h,
                                                CONFIG["pixels_threshold"])
            except Exception as e:
                name, th, note = None, None, "自动测色异常：%s" % e
            if name:
                color_name = name
                active_threshold = [th]
                calibrated = True
                print("[CAL] 自动标定成功：主色=%s  阈值=%s  (%s)" % (name, th, note))
                print("[CAL] 想要掉电保留就发一条 SAVE 命令")
            else:
                print("[CAL] %s" % note)

        # 持续自动测色：换物块时自己跟着换颜色（连续两次判成新颜色才切，防抖）
        if (auto_mode and calibrated and auto_continuous
                and (loop_t0 - t_last_scan) >= 2.0):
            t_last_scan = loop_t0
            try:
                name, th, note = auto_calibrate(img, img_w, img_h,
                                                CONFIG["pixels_threshold"])
            except Exception:
                name, th, note = None, None, ""
            switch = decider.feed(name, color_name)
            if switch and th:
                color_name = switch
                active_threshold = [th]
                print("[CAL] 检测到颜色变化 -> %s  阈值=%s  (%s)"
                      % (switch, th, note))

        if mission is not None:
            # ---- v3.0 搬运模式：一帧把块和环全找出来，交给状态机决策 ----
            objs = scan_objects(img, img_w, img_h)
            dec = mission.update(objs)
            mission_objs, mission_dec = objs, dec
            frame, info = frame_from_object(dec["target"], dec["color"],
                                            img_w, img_h)
            last_info = info
        else:
            blobs = img.find_blobs(active_threshold,
                                   pixels_threshold=CONFIG["pixels_threshold"])
            best = None
            if blobs:
                # 同一颜色有多个色块时，只发面积最大的那个
                best = max(blobs, key=lambda b: b[2] * b[3])

            # LAB 探针：直接把"该填什么阈值"读出来。
            # 检测到目标时量的是目标框（这才是要填进 THRESHOLDS 的范围），
            # 没检测到时量画面中心的一个框（用来看看当前背景/物体是什么颜色）。
            if opts["probe_lab"] and (loop_t0 - t_last_probe) >= 1.0:
                t_last_probe = loop_t0
                try:
                    if best is not None:
                        roi = [best[0], best[1], best[2], best[3]]
                        tag = "目标框"
                    else:
                        roi = [img_w // 2 - 80, img_h // 2 - 60, 160, 120]
                        tag = "中心区"
                    st = img.get_statistics(roi=roi)
                    print("[LAB] %s(%d,%d %dx%d) L %d~%d 均值%d | A %d~%d 均值%d | B %d~%d 均值%d"
                          % (tag, roi[0], roi[1], roi[2], roi[3],
                             st.l_min(), st.l_max(), st.l_mean(),
                             st.a_min(), st.a_max(), st.a_mean(),
                             st.b_min(), st.b_max(), st.b_mean()))
                except Exception as e:
                    print("[WARN] LAB 探针失败：%s" % e)

            # v4：要把"环还是块"告诉主控（走第 6 字节的 bit7），
            # 所以这里对选中的目标做一次形状取样（约 0.5ms，可接受）。
            is_ring = False
            if best is not None:
                rect = (int(best[0]), int(best[1]), int(best[2]), int(best[3]))
                try:
                    hole_bg, _ = probe_hole(img, rect)
                    is_ring = (decide_shape(hole_bg, blob_fill(best, rect))
                               == "ring")
                except Exception:                             # noqa: BLE001
                    is_ring = False

            frame, info = frame_from_blob(color_name, best, img_w, img_h, is_ring)
            if info is not None:
                info["ring"] = is_ring
            last_info = info

        # ---------------- 2. 处理主控下行指令（v4：4 字节，无参数、无回执）----------------
        data = serial.read() if serial is not None else None
        if data:
            rx_bytes += len(data)
            for cmd in parser.feed(data):
                rx_frames += 1
                if cmd["cmd"] == CMD_PING:
                    # 扩展指令，调试用：立刻补发一帧，方便上位机确认链路是通的。
                    # 主控不需要发它。
                    if serial is not None:
                        serial.write(frame)
                        sent_frames += 1
                    print("[CMD] PING -> 立即补发一帧")
                elif cmd["color"]:
                    color_name = cmd["color"]
                    active_threshold = THRESHOLDS.get(color_name, NONE_THRESHOLD)
                    auto_mode = False
                    print("[CMD] 主控指定识别颜色 -> %s（一直按这个识别）" % color_name)

        # ---------------- 3. 发送上行帧 ----------------
        # v4 没有帧序号；文档要求"收到指令后一直识别下去"，所以除了本地
        # 暂停（触摸屏 RUN 键）之外不做任何发送门控。
        if serial is not None and not standby:
            serial.write(frame)
            seq = (seq + 1) & 0xFF        # 只给日志用，协议里已经没有序号了
            sent_frames += 1

        # ---------------- 4. 叠加层与显示 ----------------
        # 只要"要显示"或"要存调试图"就把叠加层画上去。
        # 注意：画叠加层和开屏幕是两件事 —— --no-display --save-image 可以在
        # 完全不碰屏幕（也就不会和启动器抢屏幕）的情况下把带框的画面存成文件。
        save_img_obj = img
        if disp is not None or opts["save_image"]:
            if mission is not None:
                # 搬运模式：所有目标都画出来（环=蓝框，块=绿框），当前目标加粗橙框
                for o in mission_objs:
                    col = image.COLOR_BLUE if o["kind"] == "ring" else image.COLOR_GREEN
                    img.draw_rect(o["x"], o["y"], o["w"], o["h"], col, 1)
                    img.draw_string(o["x"], max(0, o["y"] - 12),
                                    "%s%s" % (o["color"][0].upper(),
                                              "R" if o["kind"] == "ring" else "B"),
                                    col, 1)
                tgt = mission_dec["target"] if mission_dec else None
                if tgt is not None:
                    img.draw_rect(max(0, tgt["x"] - 3), max(0, tgt["y"] - 3),
                                  tgt["w"] + 6, tgt["h"] + 6,
                                  image.COLOR_ORANGE, 2)
                    img.draw_cross(int(tgt["cx"]), int(tgt["cy"]),
                                   image.COLOR_RED, 8, 2)
                # 两个动作点：抓=红，放=蓝
                img.draw_cross(mission.grab_point[0], mission.grab_point[1],
                               image.COLOR_RED, 10, 1)
                img.draw_cross(mission.drop_point[0], mission.drop_point[1],
                               image.COLOR_BLUE, 10, 1)

                n_ring = sum(1 for o in mission_objs if o["kind"] == "ring")
                line1 = "MISSION %s %s" % (
                    STATE_ASCII.get(mission.state, "?"),
                    mission.color.upper() if mission.color else "-")
                line2 = "%s ring=%d block=%d done=%d/%d" % (
                    mission.progress, n_ring, len(mission_objs) - n_ring,
                    len(mission.done), len(mission.order))

                # 目标框的 LAB 范围：现场调阈值时直接照着屏幕抄数，
                # 不需要连电脑、不需要 SSH（--probe-lab 打开）
                if opts["probe_lab"] and tgt is not None \
                        and (loop_t0 - t_last_probe) >= 1.0:
                    t_last_probe = loop_t0
                    try:
                        st = img.get_statistics(
                            roi=[tgt["x"], tgt["y"], tgt["w"], tgt["h"]])
                        lab_txt = "L%d~%d A%d~%d B%d~%d" % (
                            st.l_min(), st.l_max(), st.a_min(), st.a_max(),
                            st.b_min(), st.b_max())
                        print("[LAB] 目标 %s %s: %s" % (tgt["color"], tgt["kind"],
                                                        lab_txt))
                        img.draw_string(4, img_h - 92, lab_txt,
                                        image.COLOR_YELLOW, 1)
                    except Exception as e:                       # noqa: BLE001
                        print("[WARN] 目标 LAB 取样失败：%s" % e)
            else:
                if best is not None:
                    img.draw_rect(best[0], best[1], best[2], best[3],
                                  image.COLOR_GREEN, 2)
                    img.draw_cross(int(best[0] + best[2] / 2),
                                   int(best[1] + best[3] / 2),
                                   image.COLOR_RED, 6, 2)
                # 画面几何中心，作为"理想居中位置"的参考
                img.draw_cross(img_w // 2, img_h // 2, image.COLOR_BLUE, 8, 1)
                # LAB 探针的取样框：对着它读数
                if opts["probe_lab"]:
                    img.draw_rect(img_w // 2 - 80, img_h // 2 - 60, 160, 120,
                                  image.COLOR_YELLOW, 1)
                # 自动测色还没标定时，把采样区画出来，告诉用户物块该放哪
                if auto_mode and not calibrated:
                    img.draw_rect(int(img_w * 0.3), int(img_h * 0.3),
                                  int(img_w * 0.4), int(img_h * 0.4),
                                  image.COLOR_ORANGE, 2)

                line1 = "%s  %s" % (color_name.upper(),
                                    "STANDBY" if standby else "RUN")
                if info:
                    line2 = "xn=%d yn=%d wn=%d hn=%d" % (info["x_n"], info["y_n"],
                                                         info["w_n"], info["h_n"])
                else:
                    line2 = "NO TARGET"

            # 触摸界面模式下，这些文字由画布上的状态栏负责显示，
            # 不要在 img 上再画一遍，否则缩放后会和新状态栏叠在一起。
            if ui is None:
                img.draw_string(4, 4, line1, image.COLOR_WHITE, 2)
                img.draw_string(4, 30, line2, image.COLOR_WHITE, 1)
                img.draw_string(4, 52, "fps=%.1f seq=%d" % (fps_now, seq),
                                image.COLOR_WHITE, 1)

                # 任务码显示：比赛规则要求"显示任务"（5 分），
                # 字体高度不小于 4mm、亮光显示、持续到比赛结束。
                # 屏幕高 480px 对应约 44mm，所以 48px（scale 6）≈ 4.4mm。
                if task_code:
                    img.draw_string(8, img_h - 64, "CODE %s" % task_code,
                                    image.COLOR_YELLOW, 6)

            if disp is not None:
                if ui is not None:
                    canvas = None
                    try:
                        canvas = img.resize(ui_dw, ui_dh)
                    except Exception as e:                    # noqa: BLE001
                        print("[WARN] 缩放画面失败：%s" % e)
                    if canvas is not None:
                        # 第一行是放大的任务码（规则要求字高≥4mm），
                        # 第二行是进度+状态，第三行优先显示操作反馈，
                        # 3 秒后自动回到实时的目标/帧率
                        l1 = "CODE %s" % (ui.code or "---")
                        tgt_now = mission_dec["target"] if mission_dec else None
                        if mission is not None:
                            l2 = "%s %s" % (mission.progress,
                                            STATE_ASCII.get(mission.state, "?"))
                        else:
                            l2 = "no mission"
                        # 屏幕字库只有 ASCII，所有上屏文字必须是英文
                        live = ("TGT %s%s " % (
                            tgt_now["color"],
                            "R" if tgt_now["kind"] == "ring" else "B")
                            if tgt_now else "WAIT ") + "%.1ffps" % fps_now
                        msg = ui.message if (loop_t0 - ui_msg_t) < 3.0 else live
                        draw_field_ui(canvas, ui, image, l1, l2, msg,
                                      not standby)
                        disp.show(canvas)
                        save_img_obj = canvas

                        # 触摸：只在"按下"那一下触发，避免长按连发
                        if ts is not None:
                            try:
                                td = ts.read()
                            except Exception:                 # noqa: BLE001
                                td = None
                            pressed = bool(td and len(td) >= 3 and td[2])
                            if pressed and not ui_pressed:
                                hit = ui.hit(td[0], td[1])
                                # 回显坐标：现场按了没反应时，看这一行就知道
                                # 是坐标不对，还是没按在按钮上
                                print("[UI] touch (%d,%d) -> %s"
                                      % (td[0], td[1], hit or "none"))
                                if hit:
                                    _ui_action(hit)
                            ui_pressed = pressed
                else:
                    disp.show(img)

            # 存调试图：SSH 里把它 scp 下来就能看到设备"看到什么、画了什么"
            if opts["save_image"] and (loop_t0 - t_last_save) >= opts["save_every"]:
                try:
                    save_img_obj.save(opts["save_image"])
                    print("[DBG] 已存调试图 -> %s" % opts["save_image"])
                except Exception as e:
                    print("[WARN] 存调试图失败：%s" % e)
                t_last_save = loop_t0

        # ---------------- 5. 帧率统计与节流 ----------------
        frame_count += 1
        dt = time.time() - fps_t0
        if dt >= 1.0:
            fps_now = frame_count / dt
            frame_count = 0
            fps_t0 = time.time()
            # 每秒打一行日志：接了屏幕不接屏幕都能在 SSH 里看到实际识别结果。
            # rx= 是串口收到的字节数/合法下行帧数，用一根回环线就能验证 UART 通路。
            if mission is not None:
                tgt = mission_dec["target"] if mission_dec else None
                n_ring = sum(1 for o in mission_objs if o["kind"] == "ring")
                if tgt is not None:
                    tdesc = "%s%s(xn=%d yn=%d wn=%d)" % (
                        tgt["color"], "色标" if tgt["kind"] == "ring" else "物料",
                        tgt["x_n"], tgt["y_n"], tgt["w_n"])
                else:
                    tdesc = "无"
                print("[MSN] %.1ffps seq=%d [%s] %s %s | 目标 %s | 物体 %d环/%d块 | "
                      "本批已完成 %s | rx=%dB/%df"
                      % (fps_now, seq,
                         mission.progress,
                         STATE_TEXT.get(mission.state, "?"),
                         mission_dec["note"] if mission_dec else "",
                         tdesc, n_ring, len(mission_objs) - n_ring,
                         ",".join(sorted(mission.done)) or "-",
                         rx_bytes, rx_frames))
            elif info:
                print("[RUN] %.1ffps seq=%d 命中 xn=%d yn=%d wn=%d hn=%d 像素=(%d,%d %dx%d) rx=%dB/%df"
                      % (fps_now, seq, info["x_n"], info["y_n"], info["w_n"], info["h_n"],
                         info["x"], info["y"], info["w"], info["h"], rx_bytes, rx_frames))
            else:
                print("[RUN] %.1ffps seq=%d 未检测到目标 rx=%dB/%df"
                      % (fps_now, seq, rx_bytes, rx_frames))

        used_ms = int((time.time() - loop_t0) * 1000)
        sleep_ms = period_ms - used_ms
        if sleep_ms > 0:
            time.sleep_ms(sleep_ms)

    print("[INFO] 退出：识别 %d 帧，发送 %d 帧，收到 %d 字节/%d 指令，坏帧 %d" %
          (loop_frames, sent_frames, rx_bytes, rx_frames, parser.bad_frames))
    # 程序结束后系统会回到桌面启动器，屏幕不会黑屏


if __name__ == "__main__":
    main()
