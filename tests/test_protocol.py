"""
跨实现一致性测试
================

同一份协议现在有三套实现：
    1. camera/maixcam_color_tracker.py   （相机端，MaixPy）
    2. pc/protocol.py                    （上位机）
    3. stm32/mx_protocol.c               （主控，另有用 gcc 跑的 host_test.c）

它们必须字节级一致，否则现场会出现"看起来都发了，就是对不上"的问题。
本测试以《相机_主控串口通信协议_定稿.md》里的示例帧为唯一基准，逐个比对。

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


CAM = load_module("maixcam_color_tracker", os.path.join("camera", "maixcam_color_tracker.py"))
MON = load_module("serial_monitor", os.path.join("pc", "serial_monitor.py"))

# ---------------------------------------------------------------- 测试框架

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


def hexs(b):
    return " ".join("%02X" % x for x in b)


# ---------------------------------------------------------------- 基准示例

EX1 = bytes([0xAA, 0xAA, 0xBB, 0xBB, 0x2A, 0x01, 0x02, 0x00, 0x64,
             0x80, 0x3C, 0x28, 0x75, 0xFF, 0xFF, 0xEE, 0xEE])
EX2 = bytes([0xAA, 0xAA, 0xBB, 0xBB, 0x2B, 0x00, 0x02, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x2D, 0xFF, 0xFF, 0xEE, 0xEE])
EX_SET_GREEN = bytes([0xCC, 0xCC, 0xDD, 0xDD, 0x01, 0x02, 0x03, 0xFF, 0xFF, 0xEE, 0xEE])
EX_ACK = bytes([0xCC, 0xCC, 0xDD, 0xDD, 0x81, 0x00, 0x81, 0xFF, 0xFF, 0xEE, 0xEE])


# ---------------------------------------------------------------- 用例

def test_lengths():
    section("帧长")
    check(P.UP_FRAME_LEN == 17, "上行帧长 %d != 17", P.UP_FRAME_LEN)
    check(P.DOWN_FRAME_LEN == 11, "下行帧长 %d != 11", P.DOWN_FRAME_LEN)
    check(len(EX1) == 17 and len(EX2) == 17, "示例帧长不对")
    check(len(EX_SET_GREEN) == 11, "下行示例帧长不对")
    check(CAM.UP_FRAME_LEN == 17 and CAM.DOWN_FRAME_LEN == 11, "相机端帧长常量不一致")


def test_doc_examples():
    section("文档示例帧（基准）")
    f1 = P.build_uplink(seq=0x2A, exists=1, color=2, state=0,
                        x_n=100, y_n=128, w_n=60, h_n=40)
    check(f1 == EX1, "上位机组帧 != 文档示例一\n      得到 %s\n      期望 %s", hexs(f1), hexs(EX1))

    f1c = CAM.build_uplink(0x2A, 1, 2, 0, 100, 128, 60, 40)
    check(f1c == EX1, "相机端组帧 != 文档示例一\n      得到 %s", hexs(f1c))

    f2 = P.build_uplink(0x2B, 0, 2, 0, 0, 0, 0, 0)
    check(f2 == EX2, "上位机组帧 != 文档示例二\n      得到 %s", hexs(f2))
    check(CAM.build_uplink(0x2B, 0, 2, 0, 0, 0, 0, 0) == EX2, "相机端组帧 != 文档示例二")

    check(P.build_downlink(P.CMD_SET_COLOR, P.COLOR_GREEN) == EX_SET_GREEN,
          "SET_COLOR 组帧不符")
    check(P.build_ack(P.CMD_SET_COLOR, P.STATUS_OK) == EX_ACK, "回执组帧不符")
    check(CAM.build_ack(CAM.CMD_SET_COLOR, CAM.STATUS_OK) == EX_ACK, "相机端回执组帧不符")
    check(CAM.build_uplink(0x2A, 1, 2, 0, 100, 128, 60, 40) ==
          P.build_uplink(0x2A, 1, 2, 0, 100, 128, 60, 40),
          "两套实现的组帧结果不一致")


def test_checksum_consistency():
    section("校验和：两套实现 + 溢出行为")
    for value in range(0, 256, 7):
        payload = bytes([value, 0xFF, 0x01, 0x80, value, 0x10, 0xEE, 0x00])
        a = P.uplink_checksum(payload)
        b = CAM.uplink_checksum(payload)
        check(a == b, "payload %s 校验和 %02X != %02X", hexs(payload), a, b)

    payload = bytes([0x2A, 0x01, 0x02, 0x00, 0x64, 0x80, 0x3C, 0x28])
    check(P.uplink_checksum(payload) == 0x75, "基准校验和 != 0x75，得到 %02X",
          P.uplink_checksum(payload))
    check(CAM.uplink_checksum(payload) == 0x75, "相机端基准校验和错误")

    over = bytes([0xFF] * 8)
    check(P.uplink_checksum(over) == 0xF8, "溢出校验和 %02X != 0xF8", P.uplink_checksum(over))


def test_parse_roundtrip():
    section("解析往返")
    f = P.parse_uplink(EX1)
    check(f is not None, "示例一解析失败")
    check(f["seq"] == 0x2A and f["exists"] == 1 and f["color_name"] == "green",
          "字段解析错误: %s", f)
    check(f["x_n"] == 100 and f["y_n"] == 128 and f["w_n"] == 60 and f["h_n"] == 40,
          "坐标解析错误")
    check(f["dx"] == -28 and f["dy"] == 0, "偏差计算错误 dx=%s dy=%s", f["dx"], f["dy"])

    check(P.parse_uplink(EX2)["exists"] == 0, "示例二 exists 应为 0")
    check(P.parse_uplink(EX2)["x_n"] == 0, "丢失目标时 x_n 必须是 0")

    bad = bytearray(EX1)
    bad[12] ^= 0xFF
    check(P.parse_uplink(bytes(bad)) is None, "坏校验和不该解析成功")

    short = EX1[:16]
    check(P.parse_uplink(short) is None, "长度不对不该解析成功")

    wrong_tail = bytearray(EX1)
    wrong_tail[16] = 0x00
    check(P.parse_uplink(bytes(wrong_tail)) is None, "帧尾不对不该解析成功")

    d = P.parse_downlink(EX_SET_GREEN)
    check(d is not None and d["cmd"] == P.CMD_SET_COLOR and d["arg"] == P.COLOR_GREEN,
          "下行解析失败")
    a = P.parse_downlink(EX_ACK)
    check(a is not None and a["is_ack"] and a["ack_of"] == P.CMD_SET_COLOR,
          "回执解析失败")


def test_stream_parser():
    section("流式解析：拆包 / 粘包 / 噪声 / 坏帧 / 重同步")
    # 一字节一字节喂
    p = P.UplinkParser()
    got = []
    for b in EX1:
        got += p.feed(bytes([b]))
    check(len(got) == 1 and got[0]["seq"] == 0x2A, "逐字节喂入失败，收到 %d 帧", len(got))

    # 两帧粘在一起
    p = P.UplinkParser()
    got = p.feed(EX1 + EX2)
    check(len(got) == 2, "粘包解析失败，收到 %d 帧", len(got))
    check(P.seq_gap(got[0]["seq"], got[1]["seq"]) == 0, "相邻帧不该算丢帧")

    # 前面有噪声
    p = P.UplinkParser()
    got = p.feed(b"\x01\x02\xAA\xAA\x00\xBB\x7F" + EX1)
    check(len(got) == 1 and got[0]["seq"] == 0x2A, "噪声后同步失败")
    check(p.bad_frames == 0, "噪声被误判为坏帧 %d 次", p.bad_frames)

    # 坏帧之后必须还能收到好帧
    p = P.UplinkParser()
    bad = bytearray(EX1)
    bad[12] ^= 0xFF
    got = p.feed(bytes(bad) + EX2)
    check(len(got) == 1 and got[0]["seq"] == 0x2B,
          "坏帧之后没能恢复，收到 %d 帧", len(got))
    check(p.bad_frames >= 1, "坏帧没被计数")

    # 假帧头 AA AA BB 干扰
    p = P.UplinkParser()
    stream = bytes([0xAA, 0xAA, 0xBB]) + EX1
    got = p.feed(stream)
    check(len(got) == 1 and got[0]["seq"] == 0x2A, "假帧头导致同步失败")

    # 半个帧留在缓冲区里，等下一批数据补齐
    p = P.UplinkParser()
    got = p.feed(EX1[:9])
    check(len(got) == 0, "半个帧不该产出结果")
    got = p.feed(EX1[9:])
    check(len(got) == 1, "补齐后应产出 1 帧")


def test_camera_side_downlink():
    section("相机端下行解析器")
    p = CAM.DownlinkParser()
    got = p.feed(P.build_downlink(P.CMD_SET_COLOR, P.COLOR_BLUE))
    check(len(got) == 1 and got[0]["cmd"] == CAM.CMD_SET_COLOR and got[0]["arg"] == 3,
          "相机端解析 SET_COLOR 失败: %s", got)

    # 拆成两半喂
    frame = P.build_downlink(P.CMD_SET_FPS, 25)
    p = CAM.DownlinkParser()
    got = p.feed(frame[:5])
    check(len(got) == 0, "半个下行帧不该解析成功")
    got = p.feed(frame[5:])
    check(len(got) == 1 and got[0]["arg"] == 25, "下行拆包解析失败")

    # 坏校验和
    bad = bytearray(P.build_downlink(P.CMD_PING, 0))
    bad[6] ^= 0xFF
    p = CAM.DownlinkParser()
    got = p.feed(bytes(bad) + P.build_downlink(P.CMD_PING, 0))
    check(len(got) == 1, "坏下行帧之后的恢复失败")
    check(p.bad_frames >= 1, "坏下行帧没被计数")

    # 所有命令字都能被相机端识别
    for cmd in (CAM.CMD_SET_COLOR, CAM.CMD_RUN_STANDBY, CAM.CMD_SET_FPS,
                CAM.CMD_SAVE, CAM.CMD_PING):
        p = CAM.DownlinkParser()
        got = p.feed(P.build_downlink(cmd, 0x01))
        check(len(got) == 1 and got[0]["cmd"] == cmd, "命令 0x%02X 解析失败", cmd)


def test_normalization():
    section("归一化换算")
    check(P.norm255(0, 640) == 0, "左边缘应为 0")
    check(P.norm255(639, 640) == 255, "右边缘应为 255")
    check(P.norm255(320, 640) == 128, "画面中心 320/640 应为 128，得到 %d",
          P.norm255(320, 640))
    check(P.norm255(240, 480) == 128, "纵向中心应为 128")
    check(P.norm255(640, 640) == 255, "整幅宽度应截断到 255")
    check(P.norm255(700, 640) == 255, "越界应截断到 255")
    check(P.norm255(-5, 640) == 0, "负值应截断到 0")
    check(CAM.norm255(320, 640) == P.norm255(320, 640), "相机端归一化不一致")

    # 长宽都除以画面宽度：面积同尺度、且 h_n 不会超过 255
    w_n = P.norm255(320, 640)
    h_n = P.norm255(480, 640)   # 4:3 画面里的最大高度
    check(h_n == 191, "4:3 画面 h_n 上限应为 191，得到 %d", h_n)
    check(w_n * h_n <= 255 * 255, "面积不会溢出")


def test_frame_from_blob():
    section("相机端：色块 -> 帧")
    # 正中央 100x80 的块
    blob = [320 - 50, 240 - 40, 100, 80]
    frame, info = CAM.frame_from_blob(0x10, "green", blob, 640, 480)
    check(len(frame) == 17, "组帧长度错误")
    parsed = P.parse_uplink(frame)
    check(parsed is not None, "相机端组帧无法被上位机解析")
    check(parsed["exists"] == 1 and parsed["color_name"] == "green", "字段错误")
    check(parsed["x_n"] == 128 and parsed["y_n"] == 128, "居中色块应给出 128/128，得到 %d/%d",
          parsed["x_n"], parsed["y_n"])
    check(parsed["w_n"] == P.norm255(100, 640), "w_n 换算错误")
    check(parsed["state"] == 0, "state 默认必须是 0（v3.0 由 rotation 改名而来）")
    check(info is not None and info["x"] == 270, "info 返回错误")

    # 未检测到目标
    frame, info = CAM.frame_from_blob(0x11, "red", None, 640, 480)
    parsed = P.parse_uplink(frame)
    check(parsed is not None, "丢失目标的帧无法解析")
    check(parsed["exists"] == 0, "exists 应为 0")
    check((parsed["x_n"], parsed["y_n"], parsed["w_n"], parsed["h_n"]) == (0, 0, 0, 0),
          "丢失目标时坐标必须全 0（不能是 128）")
    check(parsed["color_name"] == "red", "颜色字段应保留当前追踪色")
    check(info is None, "无目标时 info 应为 None")

    # 颜色代码映射
    check(CAM.COLOR_CODE == {"red": 1, "green": 2, "blue": 3}, "颜色编码不符协议")

    # 帧序号循环
    f0 = P.parse_uplink(CAM.frame_from_blob(0xFF, "blue", blob, 640, 480)[0])
    check(f0["seq"] == 0xFF, "序号 0xFF 传递错误")
    check(P.seq_gap(0xFF, 0x00) == 0, "序号 255 到 0 不该算丢帧")


def test_thresholds_and_config():
    section("相机端：阈值与配置")
    for name in ("red", "green", "blue"):
        check(name in CAM.THRESHOLDS, "缺少 %s 阈值" % name)
        for rng in CAM.THRESHOLDS[name]:
            check(len(rng) == 6, "%s 阈值必须是 6 个数（L/A/B 的 min/max）" % name)
            check(-128 <= rng[2] <= 127 and -128 <= rng[3] <= 127,
                  "%s 的 A 通道超出 LAB 范围" % name)
    check(set(CAM.THRESHOLDS.keys()) == set(CAM.COLOR_CODE.keys()),
          "阈值表和颜色编码表不一致")
    check(CAM.CONFIG["uart_device"] == "/dev/ttyS1", "必须用 UART1 而不是 UART0")
    check(CAM.CONFIG["pin_tx"] == "A19" and CAM.CONFIG["pin_rx"] == "A18", "引脚不符")
    check(CAM.CONFIG["uart_baud"] == 115200, "波特率应为 115200")
    check(CAM.CONFIG["estop_enabled"] is False, "GPIO 硬急停本版本不应启用")

    # 配置读写
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, "cfg.json")
        check(CAM.load_config(path) == {}, "不存在的配置文件应返回空字典")
        check(CAM.save_config(path, "blue", 20), "保存配置失败")
        loaded = CAM.load_config(path)
        check(loaded.get("color") == "blue" and loaded.get("fps") == 20,
              "配置读取不一致: %s", loaded)

        # 排障用的调试图配置：写进文件后，开机自启的实例也会定期存图
        check(CAM.save_config(path, "green", 25,
                              save_image="/root/dbg.jpg", save_every=5),
              "带调试图的配置保存失败")
        loaded = CAM.load_config(path)
        check(loaded.get("color") == "green" and loaded.get("fps") == 25,
              "配置读取不一致: %s", loaded)
        check(loaded.get("save_image") == "/root/dbg.jpg",
              "save_image 没被读出来: %s", loaded)
        check(loaded.get("save_every") == 5.0, "save_every 没被读出来: %s", loaded)

        # 锁定开关也要能持久化（现场改一次就够，不用每次带命令行参数）
        check(CAM.save_config(path, "green", 25, None, None,
                              {"lock_ae": True, "lock_awb": True}),
              "带锁定开关的配置保存失败")
        loaded = CAM.load_config(path)
        check(loaded.get("lock_ae") is True and loaded.get("lock_awb") is True,
              "锁定开关没被读出来: %s", loaded)

        # 非法值必须被挡住
        bad = {"color": "purple", "fps": 999, "save_image": 123, "save_every": 0.1}
        with open(path, "w", encoding="utf-8") as f:
            import json as _json
            _json.dump(bad, f)
        loaded = CAM.load_config(path)
        check("color" not in loaded, "非法颜色不该被接受")
        check("fps" not in loaded, "越界帧率不该被接受")
        check("save_image" not in loaded, "非字符串 save_image 不该被接受")
        check("save_every" not in loaded, "过小的 save_every 不该被接受")

        # auto（自动测色）必须能存能读 —— 曾经因为只认红绿蓝而被丢掉
        check(CAM.save_config(path, "auto", 30), "保存 auto 配置失败")
        loaded = CAM.load_config(path)
        check(loaded.get("color") == "auto",
              "color=auto 被丢掉了（自动测色模式起不来）: %s", loaded)

        # 自动标定出来的阈值要能落盘并读回
        check(CAM.save_config(path, "red", 30, None, None, None,
                              [27, 67, 25, 51, 14, 44]), "保存阈值失败")
        loaded = CAM.load_config(path)
        check(loaded.get("threshold") == [27, 67, 25, 51, 14, 44],
              "阈值没被读回来: %s", loaded)


def test_upper_computer():
    section("上位机命令映射")
    cases = {
        "r": (P.CMD_SET_COLOR, P.COLOR_RED),
        "red": (P.CMD_SET_COLOR, P.COLOR_RED),
        "g": (P.CMD_SET_COLOR, P.COLOR_GREEN),
        "green": (P.CMD_SET_COLOR, P.COLOR_GREEN),
        "b": (P.CMD_SET_COLOR, P.COLOR_BLUE),
        "blue": (P.CMD_SET_COLOR, P.COLOR_BLUE),
        "run": (P.CMD_RUN_STANDBY, 0x01),
        "stby": (P.CMD_RUN_STANDBY, 0x00),
        "standby": (P.CMD_RUN_STANDBY, 0x00),
        "ping": (P.CMD_PING, 0x00),
        "save": (P.CMD_SAVE, 0x00),
        "fps 30": (P.CMD_SET_FPS, 30),
        "FPS 60": (P.CMD_SET_FPS, 60),
    }
    for text, (cmd, arg) in cases.items():
        r = MON.make_cmd(text)
        check(r is not None, "命令 %r 无法识别" % text)
        if r:
            check(r[0] == P.build_downlink(cmd, arg),
                  "命令 %r 组帧不符: %s", text, hexs(r[0]))

    for bad in ("", "xyz", "fps 0", "fps 61", "fps", "color"):
        check(MON.make_cmd(bad) is None, "非法命令 %r 不该被接受" % bad)

    check(MON.pct(255) == 100.0, "百分比换算错误")
    check(MON.pct(0) == 0.0, "百分比换算错误")


def test_sim_frames():
    section("上位机模拟模式：与相机端换算一致")
    blob = [100.0, 50.0, 200.0, 150.0]
    sim = MON._sim_frame(0x30, "red", blob, 640, 480)
    cam = CAM.frame_from_blob(0x30, "red", blob, 640, 480)[0]
    check(sim == cam, "模拟帧与相机端帧不一致\n      模拟 %s\n      相机 %s",
          hexs(sim), hexs(cam))

    sim_none = MON._sim_frame(0x31, "blue", None, 640, 480)
    cam_none = CAM.frame_from_blob(0x31, "blue", None, 640, 480)[0]
    check(sim_none == cam_none, "无目标帧不一致")


def test_cli_args():
    section("相机端：命令行参数")
    d = CAM.parse_args([])
    expect = {"duration": 0.0, "color": None, "fps": None,
              "no_display": False, "no_uart": False, "test_full": False,
              "save_image": None, "save_every": 3.0,
              "lock_ae": False, "lock_awb": False, "probe_lab": False,
              "auto_once": False,
              # v3.0 搬运任务
              "mission": False, "grab_point": None, "drop_point": None,
              "grab_w": None, "drop_w": None,
              "task_code": None, "hold_point": None, "ui": False}
    check(d == expect, "默认参数不对: %s", d)

    a = CAM.parse_args(["--duration", "15", "--color", "green", "--fps", "20",
                        "--no-display", "--no-uart", "--test-full"])
    check(a["duration"] == 15.0, "duration 解析错误")
    check(a["color"] == "green", "color 解析错误")
    check(a["fps"] == 20, "fps 解析错误")
    check(a["no_display"] and a["no_uart"] and a["test_full"], "开关解析错误")

    si = CAM.parse_args(["--save-image", "/root/dbg.jpg", "--save-every", "2"])
    check(si["save_image"] == "/root/dbg.jpg", "save-image 解析错误")
    check(si["save_every"] == 2.0, "save-every 解析错误")
    check(CAM.parse_args(["--save-every", "0.1"])["save_every"] == 0.5,
          "save-every 应被限制到最小 0.5 秒，避免写盘过频")
    check(CAM.parse_args(["--save-every", "abc"])["save_every"] == 3.0,
          "非法 save-every 应保持默认")

    lk = CAM.parse_args(["--lock"])
    check(lk["lock_ae"], "--lock 应锁曝光")
    check(not lk["lock_awb"],
          "--lock 不该顺带锁白平衡（实测会让画面整片变绿）")
    lk = CAM.parse_args(["--lock-ae"])
    check(lk["lock_ae"] and not lk["lock_awb"], "--lock-ae 不应顺带锁白平衡")
    lk = CAM.parse_args(["--lock-awb"])
    check(lk["lock_awb"] and not lk["lock_ae"], "--lock-awb 不应顺带锁曝光")
    check(CAM.parse_args(["--probe-lab"])["probe_lab"], "--probe-lab 解析错误")
    check(CAM.parse_args(["--auto-once"])["auto_once"], "--auto-once 解析错误")
    check(not CAM.parse_args([])["auto_once"], "默认应是持续自动测色")

    # 回执帧必须能被识别成"回执"（最高位为 1）：相机端靠这个忽略它，避免 ACK 风暴
    for c in (P.CMD_SET_COLOR, P.CMD_PING, P.CMD_SET_FPS):
        ack = P.build_ack(c, P.STATUS_OK)
        parsed = P.parse_downlink(ack)
        check(parsed is not None, "回执无法解析")
        if parsed:
            check(parsed["cmd"] & 0x80, "回执命令字最高位必须为 1")
            check(parsed["ack_of"] == c, "回执对应的原命令字解析错误")
    for c in (P.CMD_SET_COLOR, P.CMD_PING, P.CMD_SET_FPS, P.CMD_SAVE):
        check((c & 0x80) == 0, "命令字 0x%02X 不该带最高位" % c)

    # 非法值必须被忽略而不是崩掉
    bad = CAM.parse_args(["--color", "purple", "--fps", "99", "--duration", "abc"])
    check(bad["color"] is None, "非法颜色应被忽略")
    check(bad["fps"] is None, "越界帧率应被忽略")
    check(bad["duration"] == 0.0, "非法 duration 应保持默认")

    # 缺少参数值时不能越界
    check(CAM.parse_args(["--duration"])["duration"] == 0.0, "缺参数值不该崩")

    # 全范围阈值：必须是一段覆盖 L/A/B 全区的 6 元组
    check(len(CAM.FULL_RANGE_THRESHOLD) == 1, "全范围阈值应为单段")
    rng = CAM.FULL_RANGE_THRESHOLD[0]
    check(len(rng) == 6, "全范围阈值应为 6 个数")
    check(rng[1] >= 100 and rng[2] <= -128 and rng[3] >= 127,
          "全范围阈值没有覆盖全 LAB 空间: %s", rng)


class _FakeStats:
    """假的 image.Statistics，用来在电脑上测阈值生成。"""

    def __init__(self, l, a, b, lsd=7.0, asd=9.0, bsd=8.0):
        self._l, self._a, self._b = l, a, b
        self._lsd, self._asd, self._bsd = lsd, asd, bsd

    def l_mean(self):
        return self._l

    def a_mean(self):
        return self._a

    def b_mean(self):
        return self._b

    def l_std_dev(self):
        return self._lsd

    def a_std_dev(self):
        return self._asd

    def b_std_dev(self):
        return self._bsd


def test_auto_color():
    section("自动测色：颜色判定")
    cases = [
        (36, 29, "red"), (60, 20, "red"), (20, 45, "red"),
        (-50, 30, "green"), (-40, -40, "green"),
        (20, -55, "blue"), (5, -60, "blue"),
        (60, -60, "red"),
        (2, -6, None), (0, 60, None), (10, 10, None), (-10, 5, None),
    ]
    for a, b, exp in cases:
        got = CAM.classify_color(a, b)
        check(got == exp, "classify_color(A=%s,B=%s)=%s 期望 %s", a, b, got, exp)

    # 色度：灰白接近 0，浓色很大
    check(CAM.lab_saturation(2, -6) < 12, "灰白背景的色度应小于 12")
    check(CAM.lab_saturation(36, 29) > 40, "红物块的色度应大于 40")

    section("自动测色：阈值生成")
    # 模拟实测到的红物块：A 均值 55、B 均值 35、L 均值 50
    th = CAM.threshold_from_stats(_FakeStats(50, 55, 35), k=2.5, min_width=(25, 30, 30))
    check(len(th) == 6, "阈值应为 6 个数")
    check(th[0] < th[1] and th[2] < th[3] and th[4] < th[5], "阈值上下限颠倒了: %s", th)
    check(th[2] <= 55 <= th[3], "A 均值应落在生成的 A 区间内: %s", th)
    check(th[4] <= 35 <= th[5], "B 均值应落在生成的 B 区间内: %s", th)
    check(th[1] - th[0] >= 25, "L 区间太窄，抖动一下就丢: %s", th)
    check(th[3] - th[2] >= 30, "A 区间太窄: %s", th)

    # 标准差很大时不能越界
    wide = CAM.threshold_from_stats(_FakeStats(50, 0, 0, 200, 200, 200), k=2.5)
    check(wide[0] >= 0 and wide[1] <= 100, "L 越界: %s", wide)
    check(wide[2] >= -128 and wide[3] <= 127, "A 越界: %s", wide)

    section("自动测色：种子窗口与求交")
    for name, seed in CAM.COLOR_SEED.items():
        check(len(seed) == 6, "%s 种子应为 6 个数" % name)
        check(seed[0] < seed[1] and seed[2] < seed[3] and seed[4] < seed[5],
              "%s 种子窗口上下限颠倒: %s" % (name, seed))
        check(name in CAM.COLOR_CODE, "%s 种子没有对应的颜色编码" % name)
    # 种子必须真的"宽"：能把该色系的典型值包含进去
    check(CAM.COLOR_SEED["red"][2] <= 40 <= CAM.COLOR_SEED["red"][3],
          "红色种子没包含典型 A=40")
    check(CAM.COLOR_SEED["green"][2] <= -40 <= CAM.COLOR_SEED["green"][3],
          "绿色种子没包含典型 A=-40")
    check(CAM.COLOR_SEED["blue"][4] <= -40 <= CAM.COLOR_SEED["blue"][5],
          "蓝色种子没包含典型 B=-40")

    inter = CAM._intersect(CAM.COLOR_SEED["red"], [27, 67, 21, 51, 14, 44])
    check(inter == [27, 67, 25, 51, 14, 44], "求交结果不对: %s", inter)
    check(CAM._intersect([0, 10, 0, 10, 0, 10], [20, 30, 20, 30, 20, 30])[0] > 0,
          "不相交时应自然重叠为空，交给调用方判断")

    # 各色系种子窗口的 L 必须是全开的：亮度不该参与筛选
    for name, seed in CAM.COLOR_SEED.items():
        check(seed[0] == 0 and seed[1] == 100,
              "%s 种子窗口的 L 应为全开 [0,100]，实际 %s" % (name, seed[:2]))


class _FakeBlob:
    """假的色块：blob[0..3] = x, y, w, h"""

    def __init__(self, x, y, w, h):
        self._b = [x, y, w, h]

    def __getitem__(self, i):
        return self._b[i]


class _FakeImg:
    """假图像：按"种子的 A 下限"分别返回色块和统计量，用来测自动测色的挑色逻辑。

    种子的 A 下限：红=30，绿=-128，蓝=-70（见 CAM.COLOR_SEED）。
    """

    def __init__(self, blobs_by_key, stats_by_key):
        self._blobs = blobs_by_key
        self._stats = stats_by_key

    def find_blobs(self, thresholds, pixels_threshold=0, roi=None):
        return list(self._blobs.get(thresholds[0][2], []))

    def get_statistics(self, thresholds=None, roi=None):
        if thresholds:
            return self._stats.get(thresholds[0][2], _FakeStats(0.0, 0.0, 0.0))
        return self._stats.get("all", _FakeStats(0.0, 0.0, 0.0))


# 三个色系种子的 A 下限：直接从代码里取，避免改了窗口后测试桩对不上
RED = CAM.COLOR_SEED["red"][2]
GREEN = CAM.COLOR_SEED["green"][2]
BLUE = CAM.COLOR_SEED["blue"][2]


def test_auto_calibrate_selection():
    section("自动测色：挑色与生成阈值（用假图像）")
    red_st = _FakeStats(50, 55, 35)          # 强红：色度 46
    weak_blue_st = _FakeStats(50, -1, -25)   # 淡蓝：色度 25，应被挡掉
    strong_blue_st = _FakeStats(50, -3, -49)  # 强蓝：色度 49
    empty = _FakeStats(0.0, 0.0, 0.0)

    # 只有红色系找到大色块 -> 判红，且 L 必须全开
    img = _FakeImg({RED: [_FakeBlob(200, 150, 150, 150)], GREEN: [], BLUE: []},
                   {RED: red_st})
    name, th, note = CAM.auto_calibrate(img, 640, 480, pixels_threshold=500)
    check(name == "red", "应判成 red，实际 %s (%s)" % (name, note))
    check(th is not None and len(th) == 6, "阈值生成失败: %s" % note)
    if th:
        check(th[0] == 0 and th[1] == 100,
              "L 通道必须全开 [0,100] 以抗光照变化，实际 %s", th[:2])
        check(th[2] <= 55 <= th[3], "A 均值应落在区间内: %s", th)
        check(th[3] - th[2] >= 30, "A 区间太窄: %s", th)
        check(CAM.COLOR_SEED["red"][2] <= th[2] and th[3] <= CAM.COLOR_SEED["red"][3],
              "阈值跑出红色种子窗口了: %s", th)

    # 蓝色系色块更大 -> 应判蓝
    img = _FakeImg({RED: [_FakeBlob(200, 150, 80, 80)], GREEN: [],
                    BLUE: [_FakeBlob(100, 100, 200, 200)]},
                   {RED: red_st, BLUE: strong_blue_st})
    name, th, note = CAM.auto_calibrate(img, 640, 480, pixels_threshold=500)
    check(name == "blue", "面积最大的是蓝色，应判 blue，实际 %s (%s)" % (name, note))

    # 三个色系都没找到 -> 失败并提示
    img = _FakeImg({RED: [], GREEN: [], BLUE: []}, {})
    name, th, note = CAM.auto_calibrate(img, 640, 480, pixels_threshold=500)
    check(name is None and th is None, "没有色块时应失败")
    check("物块" in (note or ""), "失败提示应告诉用户把物块放中间: %s", note)

    # 色块太小 -> 失败（面积不够）
    img = _FakeImg({RED: [_FakeBlob(200, 150, 10, 10)], GREEN: [], BLUE: []},
                   {RED: red_st})
    name, th, note = CAM.auto_calibrate(img, 640, 480, pixels_threshold=500)
    check(name is None, "色块太小时应失败，实际 %s" % name)
    check("没看到" in (note or "") or "淡色" in (note or ""),
          "应给出可读的失败原因: %s", note)

    # 淡色（色度不足）必须被挡掉：这是"放红色却框到白色"的回归测试
    img = _FakeImg({RED: [], GREEN: [], BLUE: [_FakeBlob(100, 100, 220, 200)]},
                   {BLUE: weak_blue_st})
    name, th, note = CAM.auto_calibrate(img, 640, 480, pixels_threshold=500)
    check(name is None, "淡色区域不该被当成物块，实际判成 %s" % name)
    check("淡色" in (note or ""), "应提示颜色太淡: %s", note)
    check("色度" in (note or ""), "提示里应带上实测色度，方便排查: %s", note)

    # 强色物块必须压过"面积更大但很淡"的干扰
    img = _FakeImg({RED: [_FakeBlob(200, 150, 120, 120)],   # 面积 14400
                    GREEN: [],
                    BLUE: [_FakeBlob(60, 60, 240, 220)]},   # 面积 52800 但很淡
                   {RED: red_st, BLUE: weak_blue_st})
    name, th, note = CAM.auto_calibrate(img, 640, 480, pixels_threshold=500)
    check(name == "red",
          "强色物块应压过更大面积的淡色干扰，实际 %s (%s)", name, note)
    check("已排除" in (note or ""), "提示里应说明排除了哪些淡色干扰: %s", note)


def test_color_switch_decider():
    section("自动测色：换色判决（防抖）")
    d = CAM.ColorSwitchDecider(need=2)
    # 一开始就是当前色 -> 不切
    check(d.feed("red", "red") is None, "同色不该切")
    # 连续两次判成蓝色才切
    check(d.feed("blue", "red") is None, "第一次判成蓝色不该立刻切")
    check(d.feed("blue", "red") == "blue", "连续两次应切成蓝色")
    # 切完之后计数清零，不会连切
    check(d.feed("blue", "blue") is None, "刚切完不该再切")

    # 中途判回原色 -> 计数清零，要重新数
    d = CAM.ColorSwitchDecider(need=2)
    check(d.feed("green", "red") is None, "第一次")
    check(d.feed("red", "red") is None, "判回原色应清零")
    check(d.feed("green", "red") is None, "清零后第一次不该切")
    check(d.feed("green", "red") == "green", "再次连续两次应切")

    # 中间蹦出第三个颜色 -> 换候选，也要重新数
    d = CAM.ColorSwitchDecider(need=2)
    check(d.feed("green", "red") is None, "第一次绿")
    check(d.feed("blue", "red") is None, "换成蓝候选，重新数")
    check(d.feed("blue", "red") == "blue", "蓝连续两次应切")

    # 没测出颜色 -> 清零
    d = CAM.ColorSwitchDecider(need=2)
    check(d.feed("green", "red") is None, "第一次绿")
    check(d.feed(None, "red") is None, "没测出来")
    check(d.feed("green", "red") is None, "清零后重新数")
    check(d.feed("green", "red") == "green", "连续两次切")


def test_capture_file_decode():
    section("抓包文件解码（不需要 pyserial 的那条路）")
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        # 正常抓包：前面带噪声、中间 5 帧、尾部半截
        good = (b"\x01\x02"
                + b"".join(P.build_uplink(i % 256, 1, 1, 0, 100 + i, 128, 50, 40)
                           for i in range(5))
                + b"\xAA\xBB")
        p_good = os.path.join(d, "good.bin")
        with open(p_good, "wb") as f:
            f.write(good)
        check(MON.decode_file(p_good) == 0, "正常抓包应该解码成功")

        # 空文件：线上没数据，必须给出接线提示
        p_empty = os.path.join(d, "empty.bin")
        open(p_empty, "wb").close()
        check(MON.decode_file(p_empty) == 1, "空文件应返回失败")

        # 垃圾数据：有数据但不是协议帧（多半是波特率不对）
        p_junk = os.path.join(d, "junk.bin")
        with open(p_junk, "wb") as f:
            f.write(bytes(range(64)))
        check(MON.decode_file(p_junk) == 1, "垃圾数据应返回失败")

        # 不存在的文件
        check(MON.decode_file(os.path.join(d, "nope.bin")) == 2,
              "不存在的文件应返回 2")


def test_mixed_parser():
    section("上位机：混合流解析（17 字节数据帧 + 11 字节回执同一根线）")
    up = P.build_uplink(7, 1, P.COLOR_RED, P.STATE_GRAB, 120, 130, 40, 42)
    ack = P.build_ack(P.CMD_NEXT, P.STATUS_OK)
    stream = b"\x00\xAA" + up + ack + up

    # 一次喂进去
    p = P.MixedParser()
    out = p.feed(stream)
    kinds = [f["kind"] for f in out]
    check(kinds == ["uplink", "downlink", "uplink"],
          "应解出 数据/回执/数据，得到 %s", kinds)
    check(p.uplink_count == 2 and p.downlink_count == 1,
          "计数错误：上行 %d 下行 %d", p.uplink_count, p.downlink_count)
    check(out[1]["ack_of"] == P.CMD_NEXT, "回执应对应 NEXT，得到 %s",
          hex(out[1]["ack_of"]))
    check(out[0]["seq"] == 7, "序号解析错误，得到 %d", out[0]["seq"])

    # 一字节一字节喂（验证拆包/重新同步）
    p2 = P.MixedParser()
    out2 = []
    for i in range(len(stream)):
        out2 += p2.feed(stream[i:i + 1])
    check([f["kind"] for f in out2] == kinds, "逐字节喂入结果应一致")

    # 坏帧之后要能重新同步
    p3 = P.MixedParser()
    bad = bytearray(up)
    bad[12] ^= 0xFF
    out3 = p3.feed(bytes(bad) + up)
    check(p3.bad_frames == 1, "应记录 1 个坏帧，实际 %d", p3.bad_frames)
    check(len(out3) == 1 and out3[0]["seq"] == 7,
          "坏帧之后应重新同步到好帧，得到 %d 帧", len(out3))

    # 纯噪声不能卡死，也不能吐出假帧
    p4 = P.MixedParser()
    check(p4.feed(b"\x01" * 40) == [], "纯噪声不该解出帧")


def test_ground_station():
    section("上位机：画面换算 + 演示数据源")
    GS = load_module("ground_station", os.path.join("pc", "ground_station.py"))

    # 归一化 -> 画布像素。h_n 也是按**画面宽度**归一化的，这条最容易搞错。
    x, y, w, h = GS.norm_box(128, 128, 51, 51, 640, 480)
    # 注意 128/255 并不等于 0.5，所以"中心"是 321.25 / 240.94 而不是 320 / 240
    check(abs((x + w / 2) - 128 / 255.0 * 640) < 0.6,
          "中心 x 换算错误，得到 %.2f", x + w / 2)
    check(abs((y + h / 2) - 128 / 255.0 * 480) < 0.6,
          "中心 y 换算错误，得到 %.2f", y + h / 2)
    check(abs(w - 128.0) < 0.6, "w_n=51 -> 128px，得到 %.1f", w)
    check(abs(h - 128.0) < 0.6, "h_n 也应按宽度换算 -> 128px，得到 %.1f", h)

    # 演示脚本要覆盖全部状态、两种有无目标、三种颜色
    poses = [GS.demo_pose(i * 0.05) for i in range(700)]
    check(set(p[2] for p in poses) == {0, 1, 2, 3, 4, 5},
          "演示应覆盖 6 个状态，实际 %s", sorted(set(p[2] for p in poses)))
    check(set(p[0] for p in poses) == {0, 1}, "演示应同时有「有目标/无目标」")
    check(len(set(p[1] for p in poses)) == 3, "演示应覆盖三种颜色")
    check(all(0 <= v <= 255 for p in poses for v in p[3:]),
          "演示坐标必须落在 0-255 内")

    # 演示造出来的帧必须和相机端字节一致
    exists, color, state, xn, yn, wn, hn = GS.demo_pose(4.2)
    code = P.COLOR_VALUES[color]
    args = (3, 1, code, state, int(round(xn)), int(round(yn)),
            int(round(wn)), int(round(hn)))
    check(P.build_uplink(*args) == CAM.build_uplink(*args),
          "演示帧必须与相机端 build_uplink 字节一致")
    check(P.parse_uplink(P.build_uplink(*args)) is not None, "演示帧应能被解析")

    # 下发命令要收到回执；搬运模式下 CALIBRATE 必须是 FAIL
    link = GS.DemoLink()
    link.open()
    link.send(P.build_downlink(P.CMD_NEXT, 0))
    acks = [f for f in link.drain() if f["kind"] == "downlink"]
    check(len(acks) == 1, "NEXT 应回一条回执，得到 %d", len(acks))
    if acks:
        check(acks[0]["ack_of"] == P.CMD_NEXT, "回执应对应 NEXT")
        check(acks[0]["arg"] == P.STATUS_OK, "NEXT 应成功")

    link.send(P.build_downlink(P.CMD_CALIBRATE, 0))
    acks = [f for f in link.drain() if f["kind"] == "downlink"]
    check(bool(acks) and acks[0]["arg"] == P.STATUS_FAIL,
          "搬运模式下 CALIBRATE 应回 FAIL（与相机端行为一致）")


def _up(seq, state=0, color=None, exists=1, x=128, y=128, w=50, h=50):
    """造一帧上行数据并解析回来（和真机上收到的东西同构）。"""
    color = P.COLOR_RED if color is None else color
    raw = P.build_uplink(seq, exists, color, state, x, y, w, h)
    return P.parse_uplink(raw)


def test_frame_stats():
    section("上位机：帧统计（丢帧 / 断线重同步 / 帧率 / 断流）")
    GS = load_module("ground_station", os.path.join("pc", "ground_station.py"))

    st = GS.FrameStats()
    check(st.on_frame(0.00, _up(0)), "第一帧状态从无到有，应算变化")
    check(not st.on_frame(0.02, _up(1)), "状态没变不该报变化")
    st.on_frame(0.04, _up(3))              # 序号 1 -> 3，中间少了 1 帧
    check(st.lost == 1, "序号断口应记 1 帧丢失，得到 %d", st.lost)
    check(st.on_frame(0.06, _up(4, state=P.STATE_GRAB)),
          "状态变了应返回 True")
    st.on_frame(0.08, _up(5, exists=0, color=0))
    check(st.no_target == 1, "无目标帧计数错误")
    check(st.total == 5, "总帧数应为 5，得到 %d", st.total)

    # 断线之后的第一帧不该被记成"丢失几十帧"
    lost_before, res_before = st.lost, st.resyncs
    st.on_frame(5.00, _up(120))            # 4.94 秒没收到帧
    check(st.resyncs == res_before + 1, "断线后的第一帧应记一次重新同步")
    check(st.lost == lost_before,
          "断线后的第一帧不该记丢帧（否则真实丢帧会被淹没），得到 %d vs %d",
          st.lost, lost_before)
    # 恢复之后又连续的帧，照常按序号算
    st.on_frame(5.02, _up(123))
    check(st.lost == lost_before + 2, "恢复后应继续按序号算丢帧，得到 %d",
          st.lost)

    # 帧率：1 秒窗口内收 20 帧 -> 20fps
    st2 = GS.FrameStats(window=1.0, stale_after=1.0)
    for i in range(20):
        st2.on_frame(i * 0.02, _up(i))
    st2.tick(0.39)
    check(abs(st2.fps - 20.0) < 0.01, "1 秒 20 帧应算 20fps，得到 %.2f", st2.fps)
    check(not st2.stale, "刚收到帧不该判断流")

    # 断流：不再来帧，界面不能继续显示虚高的帧率
    st2.tick(1.60)
    check(st2.stale, "1.2 秒没收到帧应判断流")
    check(st2.fps == 0.0, "断流时帧率必须归零，得到 %.2f", st2.fps)
    check(st2.idle > 1.0, "断流时长计算错误，得到 %.2f", st2.idle)

    # 丢帧率：按"应有帧数"算分母
    st3 = GS.FrameStats()
    st3.on_frame(0.0, _up(0))
    st3.on_frame(0.02, _up(3))             # 丢 2
    check(abs(st3.loss_pct - 100.0 * 2 / 4) < 0.01,
          "丢帧率应为 2/4=50%%，得到 %.2f", st3.loss_pct)


def main():
    print("=" * 64)
    print("协议跨实现一致性测试（基准：协议定稿文档的示例帧）")
    print("=" * 64)
    test_lengths()
    test_doc_examples()
    test_checksum_consistency()
    test_parse_roundtrip()
    test_stream_parser()
    test_camera_side_downlink()
    test_normalization()
    test_frame_from_blob()
    test_thresholds_and_config()
    test_cli_args()
    test_auto_color()
    test_auto_calibrate_selection()
    test_color_switch_decider()
    test_capture_file_decode()
    test_upper_computer()
    test_sim_frames()
    test_mixed_parser()
    test_ground_station()
    test_frame_stats()

    print("\n" + "=" * 64)
    print("结果：通过 %d 项，失败 %d 项" % (_PASS, _FAIL))
    print("=" * 64)
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
