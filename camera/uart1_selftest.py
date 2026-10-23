#!/usr/bin/env python3
"""
UART1 电气通路自检（在 MaixCAM 上运行）
=======================================

用途：在把相机接到 STM32 之前，先把 UART1 这条线本身验死。
      只碰串口，不碰摄像头、不碰屏幕，SSH 里跑不会有任何副作用。

用法：
    # 1) 裸测：只验驱动和 TX 能不能写（RX 收不到东西是正常的）
    python3 uart1_selftest.py

    # 2) 回环测：把一根杜邦线插在 A19 和 A18 之间，再跑
    python3 uart1_selftest.py --loopback
       → 这时"发出去的字节"应该原样出现在"收到的字节"里
       → 一次同时证明 TX 和 RX 两条电路都是通的

    # 3) 外部对端测：CH340 的 TXD 接 A18，PC 端发命令
    python3 uart1_selftest.py --listen 10
       → 只监听 10 秒，把收到的每个字节和解析出的命令帧打出来

判定：
    退出码 0 = 通过，1 = 失败。每一行都以 [ OK ] / [FAIL] / [INFO] 开头。
"""

import os
import sys
import time

try:
    sys.stdout.reconfigure(line_buffering=True)
except Exception:
    pass

DEV = "/dev/ttyS1"          # UART1：不要用 /dev/ttyS0（系统控制台）
BAUD = 115200
PIN_TX = "A19"
PIN_RX = "A18"

# 复用主程序里已经跑通的协议实现，保证"自检发的帧"和"实战发的帧"是同一套字节
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
DOWN_HEADER = bytes([0xCC, 0xCC, 0xDD, 0xDD])
DOWN_TAIL = bytes([0xFF, 0xFF, 0xEE, 0xEE])
CMD_PING = 0x7F
try:
    import maixcam_color_tracker as core
    DOWN_HEADER = core.DOWN_HEADER
    DOWN_TAIL = core.DOWN_TAIL
    CMD_PING = core.CMD_PING
    parse_downlink = core.parse_downlink
    print("[INFO] 已复用主程序协议层：%s %s" % (core.PROGRAM_NAME, core.PROTOCOL_VERSION))
except Exception as e:                                        # noqa: BLE001
    print("[WARN] 无法导入主程序协议层（%s），改用内置最小实现" % e)
    core = None

    def parse_downlink(b):
        return None


def build_ping(arg=0x00):
    """组一帧下行 PING（11 字节），字节布局与协议文档一致。"""
    cmd = CMD_PING
    return (DOWN_HEADER + bytes([cmd, arg & 0xFF, (cmd + (arg & 0xFF)) & 0xFF])
            + DOWN_TAIL)


def hexs(b, limit=64):
    b = bytes(b)
    s = " ".join("%02X" % x for x in b[:limit])
    if len(b) > limit:
        s += " ...(共 %d 字节)" % len(b)
    return s


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    expect_loopback = "--loopback" in argv
    listen_s = 0.0
    for i, a in enumerate(argv):
        if a == "--listen":
            try:
                listen_s = float(argv[i + 1])
            except Exception:                                 # noqa: BLE001
                listen_s = 5.0

    print("=" * 56)
    print("UART1 自检  设备=%s  %d 8N1  TX=%s RX=%s"
          % (DEV, BAUD, PIN_TX, PIN_RX))
    print("模式：%s" % ("回环验证（A19-A18 短接）" if expect_loopback
                       else ("外部对端监听 %.0fs" % listen_s if listen_s
                             else "裸测（只验 TX）")))
    print("=" * 56)

    fails = []

    # ---------- 1. 设备节点 ----------
    if os.path.exists(DEV):
        print("[ OK ] 串口节点存在：%s" % DEV)
    else:
        print("[FAIL] 串口节点不存在：%s" % DEV)
        return 1

    from maix import pinmap, uart, err                             # noqa: F401

    # ---------- 2. 引脚复用 ----------
    for pin, func in ((PIN_TX, "UART1_TX"), (PIN_RX, "UART1_RX")):
        try:
            err.check_raise(pinmap.set_pin_function(pin, func),
                            "set %s -> %s failed" % (pin, func))
            print("[ OK ] 引脚复用：%s -> %s" % (pin, func))
        except Exception as e:                                    # noqa: BLE001
            print("[FAIL] 引脚复用：%s -> %s 失败：%s" % (pin, func, e))
            fails.append("pinmap %s" % pin)

    # 回读引脚功能（MaixPy 版本差异，没有这个接口就跳过）
    getter = getattr(pinmap, "get_pin_function", None)
    if getter:
        for pin in (PIN_TX, PIN_RX):
            try:
                got = getter(pin)
                tag = "[ OK ]" if got and "UART1" in str(got) else "[WARN]"
                print("%s 引脚回读：%s -> %s" % (tag, pin, got))
            except Exception as e:                                # noqa: BLE001
                print("[WARN] 引脚回读失败（可忽略）：%s" % e)
    else:
        print("[INFO] 当前 MaixPy 没有 pinmap.get_pin_function，跳过回读")

    # ---------- 3. 打开串口 ----------
    try:
        ser = uart.UART(DEV, BAUD)
        print("[ OK ] 打开串口成功：%s @ %d" % (DEV, BAUD))
    except Exception as e:                                        # noqa: BLE001
        print("[FAIL] 打开串口失败：%s" % e)
        return 1

    # ---------- 4. TX：写 100 帧 PING ----------
    frames = [build_ping(i & 0xFF) for i in range(100)]
    blob = b"".join(frames)
    wrote_ok = 0
    t0 = time.time()
    try:
        for f in frames:
            ser.write(f)
            wrote_ok += 1
    except Exception as e:                                        # noqa: BLE001
        print("[FAIL] 串口写入异常：%s" % e)
        fails.append("tx")

    dt = time.time() - t0
    if wrote_ok == len(frames):
        print("[ OK ] TX：写入 %d 帧 / %d 字节，耗时 %.1f ms（%.0f B/s）"
              % (wrote_ok, len(blob), dt * 1000.0, len(blob) / max(dt, 1e-6)))
    else:
        print("[FAIL] TX：只写成功 %d/%d 帧" % (wrote_ok, len(frames)))
        fails.append("tx_count")

    # ---------- 5. RX：把刚发的东西原样收回来（回环）----------
    # 先清一次缓冲，免得读到"上一次"的残留（本机踩过这个坑）
    try:
        ser.read()
    except Exception:                                             # noqa: BLE001
        pass

    want = blob
    got = bytearray()
    deadline = time.time() + (listen_s if listen_s > 0 else 2.0)
    while time.time() < deadline:
        d = ser.read()
        if d:
            got.extend(d)
            if listen_s <= 0 and len(got) >= len(want):
                break
        else:
            time.sleep(0.002)

    print("[INFO] RX：收到 %d 字节" % len(got))
    if got:
        print("[INFO] 收到内容：%s" % hexs(got))
        # 解析一下，顺便证明"收到的东西能被协议层认出来"
        buf = bytearray(got)
        n_cmd = 0
        while True:
            r = parse_downlink(buf)
            if not r:
                break
            n_cmd += 1
        print("[INFO] 协议层解析出 %d 条下行帧" % n_cmd)

    # ---------- 6. 结论 ----------
    if listen_s > 0:
        # 外部对端模式：只要能收到字节就算 RX 通了
        if len(got) > 0:
            print("[ OK ] RX：外部对端 → A18 有数据进来，接收方向通的")
        else:
            print("[FAIL] RX：%.0f 秒内一个字节都没收到" % listen_s)
            fails.append("rx_none")
    elif expect_loopback:
        if bytes(got) == want:
            print("[ OK ] 回环：收发的 %d 字节完全一致 —— "
                  "TX 和 RX 两条电路同时验通" % len(want))
        elif len(got) == 0:
            print("[FAIL] 回环：一个字节都没回来。"
                  "检查 A19-A18 那根线是否插紧、是否插在正确的两个焊盘上")
            fails.append("loopback_none")
        else:
            print("[FAIL] 回环：收到 %d 字节，但内容和发出的不一致"
                  "（期望 %d 字节）" % (len(got), len(want)))
            # 找出第一个不一致的位置，便于判断是丢字节还是电平问题
            for i in range(min(len(got), len(want))):
                if got[i] != want[i]:
                    print("[INFO] 第一个不一致处：第 %d 字节 期望 %02X 实际 %02X"
                          % (i, want[i], got[i]))
                    break
            fails.append("loopback_mismatch")
    else:
        # 裸测：收不到是正常的，收到说明已经接了回环线
        if len(got) > 0:
            print("[INFO] 未要求回环，但收到了数据 —— 说明 A19-A18 之间有短接，"
                  "或者对端正在发包")
        else:
            print("[INFO] 未要求回环，RX 没收到数据 —— 符合预期（没接线时本来就收不到）")

    try:
        del ser
    except Exception:                                             # noqa: BLE001
        pass

    print("-" * 56)
    if fails:
        print("[FAIL] 自检未通过：%s" % ", ".join(fails))
        return 1
    print("[ OK ] 自检通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
