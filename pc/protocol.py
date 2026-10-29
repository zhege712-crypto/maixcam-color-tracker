"""
相机 <-> 主控 串口协议 v2.0 —— 纯逻辑实现（不依赖串口库、不依赖硬件）

对应文档：相机_主控串口通信协议_定稿.md

上行（相机 -> 主控）：17 字节定长
    0-3   帧头 AA AA BB BB
    4     帧序号 seq (0-255 循环)
    5     目标是否存在 01/00
    6     目标颜色 01红 02绿 03蓝
    7     旋转角度（圆柱物块恒 00）
    8     x_n   0-255 横向相对位置，中心约 128
    9     y_n   0-255 纵向相对位置，中心约 128
    10    w_n   0-255 长边 / 画面宽 * 255
    11    h_n   0-255 短边 / 画面宽 * 255
    12    校验和 = 偏移 4-11 共 8 字节累加 & 0xFF
    13-16 帧尾 FF FF EE EE

下行（主控 -> 相机）：11 字节定长
    0-3   帧头 CC CC DD DD
    4     命令字 CMD
    5     参数 ARG
    6     校验和 = (CMD + ARG) & 0xFF
    7-10  帧尾 FF FF EE EE

回执（相机 -> 主控）：11 字节定长
    4     回执命令字 = CMD + 0x80
    5     状态 00 成功 / 01 失败
    6     校验和 = (回执命令字 + 状态) & 0xFF
"""

from __future__ import annotations

from typing import Dict, Iterator, List, Optional

# ---------------------------------------------------------------- 常量

UP_HEADER = b"\xAA\xAA\xBB\xBB"
UP_TAIL = b"\xFF\xFF\xEE\xEE"
UP_FRAME_LEN = 17

DOWN_HEADER = b"\xCC\xCC\xDD\xDD"
DOWN_TAIL = b"\xFF\xFF\xEE\xEE"
DOWN_FRAME_LEN = 11
ACK_CMD_OFFSET = 0x80

COLOR_RED = 0x01
COLOR_GREEN = 0x02
COLOR_BLUE = 0x03

COLOR_NAMES = {COLOR_RED: "red", COLOR_GREEN: "green", COLOR_BLUE: "blue"}
COLOR_VALUES = {v: k for k, v in COLOR_NAMES.items()}

CMD_SET_COLOR = 0x01
CMD_RUN_STANDBY = 0x02
CMD_SET_FPS = 0x03
CMD_CALIBRATE = 0x04
CMD_NEXT = 0x05      # v3.0：主控"已执行完当前动作"
CMD_ABORT = 0x06     # v3.0：主控放弃本次任务
CMD_SAVE = 0x10
CMD_PING = 0x7F

CMD_NAMES = {
    CMD_SET_COLOR: "SET_COLOR",
    CMD_RUN_STANDBY: "RUN/STANDBY",
    CMD_SET_FPS: "SET_FPS",
    CMD_CALIBRATE: "CALIBRATE",
    CMD_NEXT: "NEXT",
    CMD_ABORT: "ABORT",
    CMD_SAVE: "SAVE",
    CMD_PING: "PING",
}

# v3.0 上行 state 字段：相机现在处于哪个任务阶段
STATE_SEARCH = 0
STATE_MATERIAL = 1
STATE_GRAB = 2
STATE_DEST = 3
STATE_DROP = 4
STATE_DONE = 5

STATE_NAMES = {
    STATE_SEARCH: "搜索",
    STATE_MATERIAL: "对准物料",
    STATE_GRAB: "可抓取",
    STATE_DEST: "对准目标区",
    STATE_DROP: "可放置",
    STATE_DONE: "完成",
}

STATUS_OK = 0x00
STATUS_FAIL = 0x01

# 上行帧字段在整帧中的偏移
OFF_SEQ = 4
OFF_EXISTS = 5
OFF_COLOR = 6
OFF_STATE = 7          # v3.0：原 rotation，现在表示任务阶段
OFF_XN = 8
OFF_YN = 9
OFF_WN = 10
OFF_HN = 11
OFF_CHECKSUM = 12

# 校验和覆盖范围：偏移 4-11，共 8 字节
SUM_START = OFF_SEQ
SUM_END = OFF_HN  # 含

# 归一化中心理论值
CENTER_N = 128


# ---------------------------------------------------------------- 校验和

def uplink_checksum(payload: bytes) -> int:
    """payload 为偏移 4-11 的 8 字节。"""
    if len(payload) != SUM_END - SUM_START + 1:
        raise ValueError(f"payload 必须为 8 字节，实际 {len(payload)}")
    return sum(payload) & 0xFF


def downlink_checksum(cmd: int, arg: int) -> int:
    return (cmd + arg) & 0xFF


# ---------------------------------------------------------------- 组帧

def _u8(v: int, name: str) -> int:
    if not 0 <= int(v) <= 0xFF:
        raise ValueError(f"{name} 超出 uint8 范围: {v}")
    return int(v)


def build_uplink(
    seq: int,
    exists: int,
    color: int,
    state: int = 0,
    x_n: int = 0,
    y_n: int = 0,
    w_n: int = 0,
    h_n: int = 0,
) -> bytes:
    """组一帧上行数据。exists=0 时坐标字段应全为 0。"""
    payload = bytes(
        [
            _u8(seq, "seq"),
            _u8(exists, "exists"),
            _u8(color, "color"),
            _u8(state, "state"),
            _u8(x_n, "x_n"),
            _u8(y_n, "y_n"),
            _u8(w_n, "w_n"),
            _u8(h_n, "h_n"),
        ]
    )
    return UP_HEADER + payload + bytes([uplink_checksum(payload)]) + UP_TAIL


def build_downlink(cmd: int, arg: int = 0x00) -> bytes:
    cmd = _u8(cmd, "cmd")
    arg = _u8(arg, "arg")
    return DOWN_HEADER + bytes([cmd, arg, downlink_checksum(cmd, arg)]) + DOWN_TAIL


def build_ack(cmd: int, status: int = STATUS_OK) -> bytes:
    ack_cmd = (_u8(cmd, "cmd") + ACK_CMD_OFFSET) & 0xFF
    status = _u8(status, "status")
    return DOWN_HEADER + bytes([ack_cmd, status, downlink_checksum(ack_cmd, status)]) + DOWN_TAIL


# ---------------------------------------------------------------- 解帧

def parse_uplink(frame: bytes) -> Optional[Dict]:
    """解析一帧完整的上行数据（必须正好 17 字节）。失败返回 None。"""
    if len(frame) != UP_FRAME_LEN:
        return None
    if frame[0:4] != UP_HEADER or frame[13:17] != UP_TAIL:
        return None
    payload = frame[SUM_START:SUM_END + 1]
    if uplink_checksum(payload) != frame[OFF_CHECKSUM]:
        return None
    x_n, y_n = frame[OFF_XN], frame[OFF_YN]
    w_n, h_n = frame[OFF_WN], frame[OFF_HN]
    exists = frame[OFF_EXISTS]
    return {
        "seq": frame[OFF_SEQ],
        "exists": exists,
        "color": frame[OFF_COLOR],
        "color_name": COLOR_NAMES.get(frame[OFF_COLOR], "?"),
        "state": frame[OFF_STATE],
        "x_n": x_n,
        "y_n": y_n,
        "w_n": w_n,
        "h_n": h_n,
        "dx": x_n - CENTER_N,
        "dy": y_n - CENTER_N,
        "area_n": w_n * h_n,
        "raw": bytes(frame),
    }


def parse_downlink(frame: bytes) -> Optional[Dict]:
    """解析一帧完整的下行指令或回执（必须正好 11 字节）。失败返回 None。"""
    if len(frame) != DOWN_FRAME_LEN:
        return None
    if frame[0:4] != DOWN_HEADER or frame[7:11] != DOWN_TAIL:
        return None
    cmd, arg = frame[4], frame[5]
    if downlink_checksum(cmd, arg) != frame[6]:
        return None
    return {
        "cmd": cmd,
        "arg": arg,
        "is_ack": bool(cmd & ACK_CMD_OFFSET),
        "ack_of": cmd & ~ACK_CMD_OFFSET,
        "cmd_name": CMD_NAMES.get(cmd & ~ACK_CMD_OFFSET, "?"),
        "raw": bytes(frame),
    }


# ---------------------------------------------------------------- 流式解析

class _StreamParser:
    """按字节喂入，自动处理粘包 / 拆包 / 校验失败后重新同步。"""

    def __init__(self, header: bytes, tail: bytes, frame_len: int, parser, sum_start: int, sum_end: int):
        self._header = header
        self._tail = tail
        self._len = frame_len
        self._parser = parser
        self._sum_start = sum_start
        self._sum_end = sum_end
        self.buf = bytearray()
        self.bad_frames = 0  # 帧尾或校验和不合法的次数

    def feed(self, data: bytes) -> List[Dict]:
        out: List[Dict] = []
        self.buf.extend(data)
        while True:
            i = self.buf.find(self._header)
            if i < 0:
                # 只保留可能是帧头前缀的尾巴
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
                del self.buf[:1]  # 丢掉一个字节重新找帧头
                continue
            out.append(parsed)
            del self.buf[: self._len]
        return out


class UplinkParser(_StreamParser):
    """主控 / 上位机侧：解析相机发来的上行帧。"""

    def __init__(self):
        super().__init__(UP_HEADER, UP_TAIL, UP_FRAME_LEN, parse_uplink, SUM_START, SUM_END)


class DownlinkParser(_StreamParser):
    """相机侧：解析主控发来的下行指令。"""

    def __init__(self):
        super().__init__(DOWN_HEADER, DOWN_TAIL, DOWN_FRAME_LEN, parse_downlink, 4, 5)


class MixedParser:
    """上位机侧：同时处理「上行数据帧(AAAA/17B)」和「相机回执帧(CCCC/11B)」。

    为什么需要它：相机每收到一条下行命令都会立刻回一帧 11 字节回执，
    而回执和 17 字节的数据帧走的是**同一根 TX 线**。只认 AAAA 的
    UplinkParser 会把这 11 字节当垃圾吞掉（看不到命令到底成功没有），
    只认 CCCC 的又看不到数据。所以这里按「哪个帧头先出现」分派。

    返回的每个 dict 都带 kind = "uplink" / "downlink"。
    """

    def __init__(self):
        self.buf = bytearray()
        self.bad_frames = 0
        self.uplink_count = 0
        self.downlink_count = 0

    def feed(self, data: bytes) -> List[Dict]:
        out: List[Dict] = []
        self.buf.extend(data)
        while True:
            i_up = self.buf.find(UP_HEADER)
            i_dn = self.buf.find(DOWN_HEADER)

            if i_up < 0 and i_dn < 0:
                # 两个头都没有：只保留可能是帧头前缀的尾巴
                if len(self.buf) > 3:
                    del self.buf[: len(self.buf) - 3]
                break

            if i_dn < 0 or (0 <= i_up < i_dn):
                idx, frame_len, parser, kind = i_up, UP_FRAME_LEN, parse_uplink, "uplink"
            else:
                idx, frame_len, parser, kind = i_dn, DOWN_FRAME_LEN, parse_downlink, "downlink"

            if idx > 0:
                del self.buf[:idx]
            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            parsed = parser(frame)
            if parsed is None:
                self.bad_frames += 1
                del self.buf[:1]          # 丢掉一个字节重新找帧头
                continue

            parsed["kind"] = kind
            if kind == "uplink":
                self.uplink_count += 1
            else:
                self.downlink_count += 1
            out.append(parsed)
            del self.buf[:frame_len]
        return out


# ---------------------------------------------------------------- 归一化

def norm255(value: float, total: float) -> int:
    """把像素量换算成 0-255 的归一化值（四舍五入，越界截断）。"""
    if total <= 0:
        return 0
    n = int(value * 255.0 / total + 0.5)
    return 0 if n < 0 else (255 if n > 255 else n)


def seq_gap(prev: Optional[int], cur: int) -> int:
    """返回两帧之间的丢帧数（0 表示连续）。prev 为 None 时返回 0。"""
    if prev is None:
        return 0
    return (cur - prev - 1) & 0xFF


__all__ = [
    "UP_FRAME_LEN", "DOWN_FRAME_LEN", "CENTER_N",
    "COLOR_RED", "COLOR_GREEN", "COLOR_BLUE", "COLOR_NAMES", "COLOR_VALUES",
    "CMD_SET_COLOR", "CMD_RUN_STANDBY", "CMD_SET_FPS", "CMD_CALIBRATE",
    "CMD_NEXT", "CMD_ABORT", "CMD_SAVE", "CMD_PING", "CMD_NAMES",
    "STATE_SEARCH", "STATE_MATERIAL", "STATE_GRAB",
    "STATE_DEST", "STATE_DROP", "STATE_DONE", "STATE_NAMES",
    "STATUS_OK", "STATUS_FAIL",
    "uplink_checksum", "downlink_checksum",
    "build_uplink", "build_downlink", "build_ack",
    "parse_uplink", "parse_downlink",
    "UplinkParser", "DownlinkParser", "MixedParser",
    "norm255", "seq_gap",
]
