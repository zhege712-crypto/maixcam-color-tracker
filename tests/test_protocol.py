"""
协议一致性测试（v4）
====================

基准是组内那份《相机_主控串口通信协议.docx》——文档里给了一条示例帧，
这里把它当作**黄金向量**：两个实现（相机端 / 上位机侧）都必须逐字节复现它。

三套实现共享同一份定义：
    1. camera/maixcam_color_tracker.py   （相机端，MaixPy）
    2. pc/protocol.py                    （上位机）
    3. stm32/mx_protocol.c               （主控，另有 host_test.c 用 gcc 跑）

v4 相对 v3.0 的变化（见《协议定稿》文档）：
    上行 20 字节、像素 uint16 小端、校验覆盖偏移 4~14 共 11 字节、
    没有帧序号、没有回执、下行 4 字节

运行：
    python tests/test_protocol.py
"""

import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "pc"))

import protocol as P  # noqa: E402


def load_module(name, rel_path):
    path = os.path.join(ROOT, rel_path)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CAM = load_module("maixcam_color_tracker",
                  os.path.join("camera", "maixcam_color_tracker.py"))

# 文档给的示例帧：存在、绿色、旋转 0x15、中心(100,200)、长50 宽30
GOLDEN = bytes.fromhex("aa aa bb bb 01 02 15 64 00 c8 00 32 00 1e 00 94 ff ff ee ee")
GOLDEN_DOWN = bytes.fromhex("aa bb 01 ff")

_PASS = 0
_FAIL = 0
_SECTION = ""


def section(title):
    global _SECTION
    _SECTION = title
    print("\n── %s" % title)


def check(cond, msg, *args):
    global _PASS, _FAIL
    if cond:
        _PASS += 1
    else:
        _FAIL += 1
        detail = (msg % args) if args else msg
        print("   [FAIL] (%s) %s" % (_SECTION, detail))


def hexs(data):
    return " ".join("%02X" % b for b in data)


def up(seq_unused=None, **kw):
    """按字段名造一帧（测试里写起来顺手一点）。"""
    return P.build_uplink(kw.get("exists", 1), kw.get("color", 2),
                          kw.get("rotation", 0), kw.get("x", 0),
                          kw.get("y", 0), kw.get("w", 0), kw.get("h", 0))


# ---------------------------------------------------------------- 用例

def test_lengths():
    section("帧长")
    check(P.UP_FRAME_LEN == 20, "上行帧长应为 20，得到 %d", P.UP_FRAME_LEN)
    check(P.DOWN_FRAME_LEN == 4, "下行帧长应为 4，得到 %d", P.DOWN_FRAME_LEN)
    check(len(GOLDEN) == 20, "黄金向量的长度本身就该是 20")
    check(CAM.UP_FRAME_LEN == P.UP_FRAME_LEN,
          "相机端上行帧长 %d 与上位机 %d 不一致", CAM.UP_FRAME_LEN, P.UP_FRAME_LEN)
    check(CAM.DOWN_FRAME_LEN == P.DOWN_FRAME_LEN,
          "相机端下行帧长 %d 与上位机 %d 不一致",
          CAM.DOWN_FRAME_LEN, P.DOWN_FRAME_LEN)
    check(P.SUM_LEN == 11, "校验应覆盖 11 字节，得到 %d", P.SUM_LEN)
    check(CAM.SUM_END - CAM.SUM_START == 11,
          "相机端校验覆盖 %d 字节，应为 11", CAM.SUM_END - CAM.SUM_START)


def test_golden():
    section("文档示例帧（黄金向量）")
    f = P.build_uplink(1, 2, 0x15, 100, 200, 50, 30)
    check(f == GOLDEN, "上位机组帧与文档不一致\n      实际 %s\n      文档 %s",
          hexs(f), hexs(GOLDEN))

    fc = CAM.build_uplink(1, 2, 0x15, 100, 200, 50, 30)
    check(fc == GOLDEN, "相机端组帧与文档不一致\n      实际 %s\n      文档 %s",
          hexs(fc), hexs(GOLDEN))

    d = P.parse_uplink(GOLDEN)
    check(d is not None, "上位机解析文档示例帧失败")
    if d:
        check(d["exists"] == 1 and d["color_name"] == "green",
              "示例帧应是「存在 + 绿色」，得到 %s/%s", d["exists"], d["color_name"])
        check(d["x"] == 100 and d["y"] == 200, "中心坐标应为 (100,200)，得到 (%d,%d)",
              d["x"], d["y"])
        check(d["w"] == 50 and d["h"] == 30, "尺寸应为 50x30，得到 %dx%d",
              d["w"], d["h"])
        check(d["is_ring"] is False, "示例帧不是色环")

    dc = CAM.parse_downlink(GOLDEN_DOWN)
    check(dc is not None and dc["cmd"] == 1 and dc["color"] == "red",
          "相机端解析下行示例帧失败：%s", dc)

    check(P.build_downlink(1) == GOLDEN_DOWN, "下行示例帧不一致：%s",
          hexs(P.build_downlink(1)))
    check(CAM.build_downlink(1) == GOLDEN_DOWN, "相机端下行示例帧不一致")


def test_checksum():
    section("校验和")
    import itertools
    # 文档算式：偏移 4-14 共 11 字节累加取低 8 位
    payload = bytes([1, 2, 0x15, 0x64, 0x00, 0xC8, 0x00, 0x32, 0x00, 0x1E, 0x00])
    expect = sum(payload) & 0xFF
    check(expect == 0x94, "手算的校验应为 0x94，得到 0x%02X", expect)
    check(P.uplink_checksum(payload) == 0x94, "上位机校验和算错")
    check(CAM.uplink_checksum(payload) == 0x94, "相机端校验和算错")
    # 覆盖范围必须包含 h 的高字节（这是当初踩过的坑：少算 1 字节）
    f = P.build_uplink(1, 1, 0, 1, 1, 1, 0x0100)
    check(f[P.OFF_CHECKSUM] == P.uplink_checksum(f[4:15]),
          "校验应覆盖到 h 的高字节")
    check(P.parse_uplink(f) is not None, "h=0x100 的帧应能解出来")
    if P.parse_uplink(f):
        check(P.parse_uplink(f)["h"] == 0x100, "h 高字节丢了")

    try:
        P.uplink_checksum(b"\x00" * 10)
        check(False, "长度不对时应当抛异常")
    except ValueError:
        check(True, "")


def test_little_endian():
    section("uint16 小端")
    f = P.build_uplink(1, 3, 0, 0x1234, 0x00FF, 640, 480)
    check(f[P.OFF_X:P.OFF_X + 2] == b"\x34\x12", "x 应是 34 12（小端），得到 %s",
          hexs(f[P.OFF_X:P.OFF_X + 2]))
    check(f[P.OFF_Y:P.OFF_Y + 2] == b"\xFF\x00", "y 应是 FF 00")
    check(f[P.OFF_W:P.OFF_W + 2] == b"\x80\x02", "640 应是 80 02")
    check(f[P.OFF_H:P.OFF_H + 2] == b"\xE0\x01", "480 应是 E0 01")
    d = P.parse_uplink(f)
    check(d["x"] == 0x1234 and d["y"] == 0x00FF and d["w"] == 640 and d["h"] == 480,
          "小端解析回来不对：%s", d and (d["x"], d["y"], d["w"], d["h"]))


def test_ring_bit():
    section("第 6 字节：色环标志 + 角度")
    check(P.encode_rotation(0, True) == 0x80, "色环标志位应为 bit7")
    check(P.encode_rotation(0, False) == 0x00, "物块应是 0x00")
    angle, ring = P.decode_rotation(P.encode_rotation(0, True))
    check(ring is True and angle == 0, "解码回来应是 环+0 度，得到 %s/%s",
          ring, angle)
    a, r = P.decode_rotation(P.encode_rotation(180, False))
    check(r is False and abs(a - 180) < 3, "180 度解回来应接近 180，得到 %.1f", a)

    # 相机端实现必须与上位机一致（逐值比对）
    bad = 0
    for deg in range(0, 361, 5):
        if P.encode_rotation(deg, False) != CAM.encode_rotation(deg, False):
            bad += 1
        if P.encode_rotation(deg, True) != CAM.encode_rotation(deg, True):
            bad += 1
    check(bad == 0, "相机端与上位机的角度编码有 %d 处不一致", bad)

    # 一个真实色环帧：第 6 字节 bit7 置位，解析回来 is_ring 为真
    f = P.build_uplink(1, 2, P.encode_rotation(0, True), 320, 240, 60, 60)
    d = P.parse_uplink(f)
    check(d["is_ring"] is True, "色环帧解析回来 is_ring 应为真")


def test_to_ref():
    section("分辨率换算：内部 320x240 → 协议 640x480")
    # 320x240 下中心 (160,120)、宽 40 高 30
    r = P.to_ref(160, 120, 40, 30, 320, 240)
    check(r == (320, 240, 80, 60), "换算错误，得到 %s", (r,))
    # 640x480 输入应原样返回
    r2 = P.to_ref(320, 240, 80, 60, 640, 480)
    check(r2 == (320, 240, 80, 60), "640x480 输入应原样返回，得到 %s", (r2,))
    # 相机端同样的换算（相机端 to_ref 是单值版本：value, src, dst）
    check(CAM.to_ref(160, 320, 640) == 320, "相机端 to_ref 不对")
    # build_uplink_for_blob：blob 给左上角，帧里必须是中心
    frame = P.build_uplink_for_blob(1, 2, [140, 100, 40, 40], 320, 240)
    d = P.parse_uplink(frame)
    check(d["x"] == 320 and d["y"] == 240,
          "320x240 下 (140,100,40,40) 的中心换算应得 (320,240)，得到 (%d,%d)",
          d["x"], d["y"])
    check(d["w"] == 80 and d["h"] == 80, "尺寸应换算成 80x80，得到 %dx%d",
          d["w"], d["h"])

    # 同一目标在两种分辨率下组出的帧必须一致（契约不受分辨率影响）
    a = P.build_uplink_for_blob(1, 2, [280, 200, 80, 80], 640, 480)
    b = P.build_uplink_for_blob(1, 2, [140, 100, 40, 40], 320, 240)
    da, db = P.parse_uplink(a), P.parse_uplink(b)
    check(abs(da["x"] - db["x"]) <= 1 and abs(da["y"] - db["y"]) <= 1,
          "两种分辨率下中心应一致：%s vs %s",
          (da["x"], da["y"]), (db["x"], db["y"]))
    check(abs(da["w"] - db["w"]) <= 1, "两种分辨率下尺寸应一致：%d vs %d",
          da["w"], db["w"])


def test_no_target():
    section("没有目标时的帧")
    f = P.build_uplink(0, 3, 0, 0, 0, 0, 0)
    check(f == CAM.build_uplink(0, 3, 0, 0, 0, 0, 0), "两端无目标帧应一致")
    d = P.parse_uplink(f)
    check(d["exists"] == 0, "exists 应为 0")
    check(d["x"] == d["y"] == d["w"] == d["h"] == 0, "无目标时坐标必须全 0")
    check(d["color_name"] == "blue", "颜色字段仍应保留（方便判断在找什么）")
    fb = P.build_uplink_for_blob(0, 3, [10, 10, 5, 5], 640, 480)
    check(P.parse_uplink(fb)["x"] == 0, "exists=0 时即使给了 blob 坐标也必须是 0")


def test_downlink():
    section("下行指令（4 字节）")
    for cmd, name in ((1, "red"), (2, "green"), (3, "blue")):
        f = P.build_downlink(cmd)
        check(f == bytes([0xAA, 0xBB, cmd, 0xFF]), "下行帧格式错误：%s", hexs(f))
        d = P.parse_downlink(f)
        check(d and d["color"] == name, "指令 %d 应解析成 %s", cmd, name)
        cd = CAM.parse_downlink(f)
        check(cd and cd["color"] == name, "相机端解析指令 %d 失败", cmd)

    ping = P.build_downlink(P.CMD_PING)
    check(ping == bytes([0xAA, 0xBB, 0x7F, 0xFF]), "PING 帧错误：%s", hexs(ping))
    check(CAM.parse_downlink(ping)["cmd"] == 0x7F, "相机端应识别 PING")

    # 非法指令要拒绝
    check(P.parse_downlink(bytes([0xAA, 0xBB, 0x09, 0xFF])) is None, "未知指令应拒绝")
    check(P.parse_downlink(bytes([0xAA, 0xBB, 0x01, 0x00])) is None, "帧尾不对应拒绝")
    check(P.parse_downlink(bytes([0xCC, 0xBB, 0x01, 0xFF])) is None, "帧头不对应拒绝")
    check(P.parse_downlink(b"\xAA\xBB\x01") is None, "长度不对应拒绝")
    try:
        P.build_downlink(0x09)
        check(False, "组非法指令应抛异常")
    except ValueError:
        check(True, "")


def test_stream():
    section("流式解析（粘包 / 拆包 / 噪声 / 重同步）")
    f1 = P.build_uplink(1, 1, 0, 100, 200, 50, 30)
    f2 = P.build_uplink(0, 2, 0x80, 0, 0, 0, 0)

    p = P.UplinkParser()
    out = p.feed(f1 + f2)
    check(len(out) == 2, "粘包应解出 2 帧，得到 %d", len(out))
    check(out[0]["x"] == 100 and out[1]["exists"] == 0, "解析内容不对")

    # 逐字节喂
    p2 = P.UplinkParser()
    got = []
    for b in f1 + f2:
        got += p2.feed(bytes([b]))
    check(len(got) == 2, "逐字节喂入应解出 2 帧，得到 %d", len(got))

    # 前面有噪声
    p3 = P.UplinkParser()
    out3 = p3.feed(b"\x00\x11\x22\xAA" + f1)
    check(len(out3) == 1 and out3[0]["x"] == 100, "噪声之后应能找回帧")

    # 坏帧之后重同步
    p4 = P.UplinkParser()
    bad = bytearray(f1)
    bad[15] ^= 0xFF
    out4 = p4.feed(bytes(bad) + f1)
    check(p4.bad_frames == 1, "应记 1 个坏帧，得到 %d", p4.bad_frames)
    check(len(out4) == 1 and out4[0]["x"] == 100, "坏帧后应重新同步")

    # 下行流式
    dp = P.DownlinkParser()
    out5 = dp.feed(P.build_downlink(1) + P.build_downlink(2))
    check(len(out5) == 2 and out5[1]["color"] == "green", "下行粘包解析失败")


def test_seq_removed():
    section("v4 已经删掉的东西（防止误用）")
    for name in ("seq_gap", "MixedParser", "build_ack", "norm255",
                 "CMD_NEXT", "CMD_ABORT", "CMD_SAVE", "CMD_CALIBRATE",
                 "CMD_SET_FPS", "CMD_SET_COLOR", "ACK_CMD_OFFSET"):
        check(not hasattr(P, name), "protocol.py 里不该再有 %s", name)
    d = P.parse_uplink(GOLDEN)
    check("seq" not in d, "上行帧里不该有 seq 字段")
    check("state" not in d, "上行帧里不该有 state 字段")
    check("rotation" in d and "is_ring" in d, "应该给出 rotation 和 is_ring")


def test_frame_rate_helper():
    section("帧率统计（没有序号，只能靠时间）")
    fps, lo, avg, hi = P.frame_rate([0.0, 0.1, 0.2, 0.3])
    check(abs(fps - 10.0) < 0.01, "4 帧跨 0.3 秒应约 10fps，得到 %.2f", fps)
    check(abs(lo - 100.0) < 0.5 and abs(hi - 100.0) < 0.5,
          "间隔统计不对：%.1f/%.1f", lo, hi)
    check(P.frame_rate([0.0]) == (0.0, 0.0, 0.0, 0.0), "单帧应返回全 0")


def main():
    print("=" * 64)
    print("协议一致性测试 v4（基准：文档示例帧）")
    print("=" * 64)
    test_lengths()
    test_golden()
    test_checksum()
    test_little_endian()
    test_ring_bit()
    test_to_ref()
    test_no_target()
    test_downlink()
    test_stream()
    test_seq_removed()
    test_frame_rate_helper()

    print("\n" + "=" * 64)
    print("结果：通过 %d 项，失败 %d 项" % (_PASS, _FAIL))
    print("=" * 64)
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
