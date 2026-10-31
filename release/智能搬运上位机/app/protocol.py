"""
相机 <-> 主控 串口协议（v4：与组内《相机_主控串口通信协议.docx》字节级一致）
=========================================================================

这份实现以那份文档为**唯一基准**。文档里的示例帧直接作为黄金向量写进了
tests/test_protocol.py，任何改动导致它不匹配都会立刻失败。

上行（相机 -> 主控）：20 字节定长
    0-3    帧头 AA AA BB BB
    4      目标是否存在   0x01 存在 / 0x00 不存在
    5      目标颜色       0x01 红 / 0x02 绿 / 0x03 蓝
    6      旋转角度       0-255 表示 0-360 度（圆柱没有旋转概念，见下面的扩展）
    7-8    中心 x         uint16 小端，像素
    9-10   中心 y         uint16 小端，像素
    11-12  目标长 w       uint16 小端，像素
    13-14  目标宽 h       uint16 小端，像素
    15     校验和         偏移 4-14 共 11 字节累加，取低 8 位
    16-19  帧尾 FF FF EE EE

    ★ 扩展（不影响文档解析器）：第 6 字节的 **bit7 用作"是不是色环"标志**，
      低 7 位才是角度。文档本身就写了"圆柱形物块没有旋转角度的概念，
      但我们仍然保留"——这个字节空着浪费，正好拿来传环/块。
      照着文档写的解析器读到的只是一个无意义的旋转值，不会出错。
      角度的映射：低 7 位 0-127 对应 0-360 度。

下行（主控 -> 相机）：4 字节定长，**无校验和**
    0-1    帧头 AA BB
    2      指令 0x01 识别红色 / 0x02 绿色 / 0x03 蓝色
           ★ 扩展 0x7F = PING（调试用，立刻回一帧；主控不需要发）
    3      帧尾 FF

    文档原文："一旦接收到一帧指令之后就按照这个指令一直识别下去。"
    也就是说相机记住这个颜色，一直发，直到收到下一条指令。

坐标基准：**固定按 640x480 换算**。相机如果在内部用 320x240 之类更低的分辨率
做识别（为了提帧率），送出去的坐标也要换算回 640x480 的参考系，这样主控那边
的坐标含义永远不变。

注意：本协议**没有帧序号**，所以无法用序号推算丢帧，只能看每秒实际收到多少帧。
"""

from __future__ import annotations

from typing import Dict, List, Optional

# ---------------------------------------------------------------- 常量

UP_HEADER = b"\xAA\xAA\xBB\xBB"
UP_TAIL = b"\xFF\xFF\xEE\xEE"
UP_FRAME_LEN = 20

DOWN_HEADER = b"\xAA\xBB"
DOWN_TAIL = b"\xFF"
DOWN_FRAME_LEN = 4

COLOR_RED = 0x01
COLOR_GREEN = 0x02
COLOR_BLUE = 0x03

COLOR_NAMES = {COLOR_RED: "red", COLOR_GREEN: "green", COLOR_BLUE: "blue"}
COLOR_VALUES = {v: k for k, v in COLOR_NAMES.items()}

# 下行指令：1/2/3 与颜色号同值，另有调试用的 PING
CMD_RED = 0x01
CMD_GREEN = 0x02
CMD_BLUE = 0x03
CMD_PING = 0x7F

CMD_NAMES = {
    CMD_RED: "TRACK_RED",
    CMD_GREEN: "TRACK_GREEN",
    CMD_BLUE: "TRACK_BLUE",
    CMD_PING: "PING",
}

VALID_CMDS = set(CMD_NAMES)

# 第 6 字节：bit7 = 色环标志，低 7 位 = 角度
RING_BIT = 0x80
ANGLE_MASK = 0x7F
ANGLE_STEPS = 127          # 低 7 位最大 127，对应 360 度

# 坐标参考系
REF_W = 640
REF_H = 480

# 偏移
OFF_EXISTS = 4
OFF_COLOR = 5
OFF_ROTATION = 6
OFF_X = 7
OFF_Y = 9
OFF_W = 11
OFF_H = 13
OFF_CHECKSUM = 15

# 校验和覆盖：偏移 4-14，共 11 字节
SUM_START = OFF_EXISTS
# 注意：h 占偏移 13-14，所以"结束（不含）"是 15，也就是校验和字节本身的位置。
# 这里不能写 OFF_H + 1（=14，会少算 h 的高字节 1 个字节）。
SUM_END = OFF_CHECKSUM
SUM_LEN = SUM_END - SUM_START      # 11


# ---------------------------------------------------------------- 校验和

def uplink_checksum(payload11: bytes) -> int:
    """payload 为偏移 4-14 的 11 字节。"""
    if len(payload11) != SUM_LEN:
        raise ValueError("payload 必须为 %d 字节，实际 %d" % (SUM_LEN, len(payload11)))
    return sum(payload11) & 0xFF


# ---------------------------------------------------------------- 角度/环

def encode_rotation(angle_deg: float = 0.0, is_ring: bool = False) -> int:
    """把"角度 + 是否色环"压进第 6 字节。"""
    a = int(round(float(angle_deg) * ANGLE_STEPS / 360.0))
    a = 0 if a < 0 else (ANGLE_STEPS if a > ANGLE_STEPS else a)
    return (RING_BIT if is_ring else 0) | (a & ANGLE_MASK)


def decode_rotation(byte: int):
    """拆开第 6 字节，返回 (角度, 是否色环)。"""
    return ((byte & ANGLE_MASK) * 360.0 / ANGLE_STEPS, bool(byte & RING_BIT))


# ---------------------------------------------------------------- 坐标换算

def to_ref(x, y, w, h, img_w, img_h):
    """把任意分辨率下的像素框换算到 640x480 参考系（四舍五入，截断到 0-65535）。"""
    def sc(v, src, dst):
        if src <= 0:
            return 0
        n = int(round(v * float(dst) / src))
        return 0 if n < 0 else (65535 if n > 65535 else n)
    return (sc(x, img_w, REF_W), sc(y, img_h, REF_H),
            sc(w, img_w, REF_W), sc(h, img_w, REF_W))


def _u8(v, name):
    if not 0 <= int(v) <= 0xFF:
        raise ValueError("%s 超出 uint8 范围: %s" % (name, v))
    return int(v)


def _u16(v, name):
    if not 0 <= int(v) <= 0xFFFF:
        raise ValueError("%s 超出 uint16 范围: %s" % (name, v))
    return int(v)


# ---------------------------------------------------------------- 组帧

def build_uplink(exists: int, color: int, rotation: int = 0,
                 x: int = 0, y: int = 0, w: int = 0, h: int = 0) -> bytes:
    """组一帧上行数据。exists=0 时坐标字段应为 0。"""
    payload = bytes([
        _u8(exists, "exists"),
        _u8(color, "color"),
        _u8(rotation, "rotation"),
        _u16(x, "x") & 0xFF, (_u16(x, "x") >> 8) & 0xFF,
        _u16(y, "y") & 0xFF, (_u16(y, "y") >> 8) & 0xFF,
        _u16(w, "w") & 0xFF, (_u16(w, "w") >> 8) & 0xFF,
        _u16(h, "h") & 0xFF, (_u16(h, "h") >> 8) & 0xFF,
    ])
    return UP_HEADER + payload + bytes([uplink_checksum(payload)]) + UP_TAIL


def build_uplink_for_blob(exists: int, color: int, blob,
                          img_w: int, img_h: int, is_ring: bool = False,
                          angle_deg: float = 0.0) -> bytes:
    """从 MaixPy 风格的 blob（x, y, w, h）直接组帧，自动换算到 640x480 参考系。

    blob 为 None 或 exists=0 时，坐标全 0。
    """
    rot = encode_rotation(angle_deg, is_ring)
    if not exists or blob is None:
        return build_uplink(0, color, rot, 0, 0, 0, 0)
    bx, by, bw, bh = blob[0], blob[1], blob[2], blob[3]
    cx = bx + bw / 2.0
    cy = by + bh / 2.0
    rx, ry, rw, rh = to_ref(cx, cy, bw, bh, img_w, img_h)
    return build_uplink(1, color, rot, rx, ry, rw, rh)


def build_downlink(cmd: int) -> bytes:
    cmd = _u8(cmd, "cmd")
    if cmd not in VALID_CMDS:
        raise ValueError("未知指令 0x%02X" % cmd)
    return DOWN_HEADER + bytes([cmd]) + DOWN_TAIL


# ---------------------------------------------------------------- 解帧

def parse_uplink(frame: bytes) -> Optional[Dict]:
    """解析一帧完整的上行数据（必须正好 20 字节）。失败返回 None。"""
    if len(frame) != UP_FRAME_LEN:
        return None
    if frame[0:4] != UP_HEADER or frame[16:20] != UP_TAIL:
        return None
    payload = frame[SUM_START:SUM_END]
    if uplink_checksum(payload) != frame[OFF_CHECKSUM]:
        return None

    x = frame[OFF_X] | (frame[OFF_X + 1] << 8)
    y = frame[OFF_Y] | (frame[OFF_Y + 1] << 8)
    w = frame[OFF_W] | (frame[OFF_W + 1] << 8)
    h = frame[OFF_H] | (frame[OFF_H + 1] << 8)
    angle, is_ring = decode_rotation(frame[OFF_ROTATION])
    return {
        "exists": frame[OFF_EXISTS],
        "color": frame[OFF_COLOR],
        "color_name": COLOR_NAMES.get(frame[OFF_COLOR], "?"),
        "rotation": frame[OFF_ROTATION],
        "angle_deg": angle,
        "is_ring": is_ring,
        "x": x, "y": y, "w": w, "h": h,
        "cx": x, "cy": y,          # 第 7 字节起就是**中心**坐标，不是左上角
        "raw": bytes(frame),
    }


def parse_downlink(frame: bytes) -> Optional[Dict]:
    """解析一帧完整的下行指令（必须正好 4 字节）。失败返回 None。"""
    if len(frame) != DOWN_FRAME_LEN:
        return None
    if frame[0:2] != DOWN_HEADER or frame[3:4] != DOWN_TAIL:
        return None
    cmd = frame[2]
    if cmd not in VALID_CMDS:
        return None
    # 注意：color 给的是**颜色名字符串**（"red"/"green"/"blue"），不是数字。
    # 和相机端 parse_downlink() 保持一致，省得调用方到处做两次映射。
    # PING 不是颜色指令，color 为 None。
    color_name = COLOR_NAMES.get(cmd) if cmd in (CMD_RED, CMD_GREEN, CMD_BLUE) else None
    return {
        "cmd": cmd,
        "cmd_name": CMD_NAMES[cmd],
        "color": color_name,
        "color_code": cmd if color_name else 0,
        "raw": bytes(frame),
    }


# ---------------------------------------------------------------- 流式解析

class _StreamParser:
    """按字节喂入，自动处理粘包 / 拆包 / 校验失败后重新同步。"""

    def __init__(self, header: bytes, tail: bytes, frame_len: int, parser):
        self._header = header
        self._tail = tail
        self._len = frame_len
        self._parser = parser
        self.buf = bytearray()
        self.bad_frames = 0

    def feed(self, data: bytes) -> List[Dict]:
        out = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(self._header)
            if i < 0:
                keep = len(self._header) - 1
                if len(self.buf) > keep:
                    del self.buf[: len(self.buf) - keep]
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < self._len:
                break
            frame = bytes(self.buf[: self._len])
            parsed = self._parser(frame)
            if parsed is None:
                self.bad_frames += 1
                del self.buf[:1]
                continue
            out.append(parsed)
            del self.buf[: self._len]
        return out


class UplinkParser(_StreamParser):
    """主控 / 上位机侧：解析相机发来的上行帧。"""

    def __init__(self):
        super().__init__(UP_HEADER, UP_TAIL, UP_FRAME_LEN, parse_uplink)


class DownlinkParser(_StreamParser):
    """相机侧：解析主控发来的下行指令。"""

    def __init__(self):
        super().__init__(DOWN_HEADER, DOWN_TAIL, DOWN_FRAME_LEN, parse_downlink)


# ---------------------------------------------------------------- 统计辅助

def frame_rate(timestamps: List[float]):
    """给一串到达时间算帧率和间隔统计（没有序号，只能靠时间）。"""
    if len(timestamps) < 2:
        return 0.0, 0.0, 0.0, 0.0
    gaps = [(b - a) * 1000.0 for a, b in zip(timestamps, timestamps[1:])]
    span = timestamps[-1] - timestamps[0]
    fps = (len(timestamps) - 1) / span if span > 0 else 0.0
    return fps, min(gaps), sum(gaps) / len(gaps), max(gaps)


__all__ = [
    "UP_HEADER", "UP_TAIL", "UP_FRAME_LEN",
    "DOWN_HEADER", "DOWN_TAIL", "DOWN_FRAME_LEN",
    "COLOR_RED", "COLOR_GREEN", "COLOR_BLUE", "COLOR_NAMES", "COLOR_VALUES",
    "CMD_RED", "CMD_GREEN", "CMD_BLUE", "CMD_PING", "CMD_NAMES", "VALID_CMDS",
    "RING_BIT", "ANGLE_MASK", "REF_W", "REF_H",
    "OFF_EXISTS", "OFF_COLOR", "OFF_ROTATION", "OFF_X", "OFF_Y",
    "OFF_W", "OFF_H", "OFF_CHECKSUM", "SUM_START", "SUM_END", "SUM_LEN",
    "uplink_checksum", "encode_rotation", "decode_rotation", "to_ref",
    "build_uplink", "build_uplink_for_blob", "build_downlink",
    "parse_uplink", "parse_downlink",
    "UplinkParser", "DownlinkParser", "frame_rate",
]
