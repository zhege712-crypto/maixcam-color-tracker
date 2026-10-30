"""
盯住串口变化 —— 用来判断"线/驱动/转接板"到底通没通
====================================================

用途：插拔 CH340 的时候，看电脑有没有真的认出来。
      比在设备管理器里手动刷新靠谱，也不会漏掉一闪而过的设备。

用法：
    python watch_ports.py            盯 60 秒
    python watch_ports.py -s 120     盯 120 秒

每个端口出现时会自动试一下"能不能打开 + 能不能写 11 字节"，
因为插上但接口焊坏/只挂电不传数据的情况很常见。
"""

import argparse
import time

try:
    import serial
    from serial.tools import list_ports
except Exception:                                            # noqa: BLE001
    print("需要 pyserial：pip install pyserial")
    raise

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol as P  # noqa: E402


def snapshot():
    out = {}
    for p in list_ports.comports():
        out[p.device] = (p.description, p.hwid)
    return out


def probe(dev):
    """试开一下并写 11 字节，返回一句人话。"""
    try:
        ser = serial.Serial(dev, 115200, timeout=0.05, write_timeout=1.0)
    except Exception as e:                                   # noqa: BLE001
        return "打不开：%s" % e
    try:
        n = ser.write(P.build_downlink(P.CMD_PING, 0))
        got = 0
        t = time.time()
        while time.time() - t < 1.0:
            got += len(ser.read(ser.in_waiting or 1))
        return "打开 OK，写 %d 字节 OK，1 秒内读回 %d 字节" % (n, got)
    except Exception as e:                                   # noqa: BLE001
        return "打开 OK，但写失败：%s" % e
    finally:
        try:
            ser.close()
        except Exception:                                    # noqa: BLE001
            pass


def main():
    ap = argparse.ArgumentParser(description="盯串口插拔")
    ap.add_argument("-s", "--seconds", type=float, default=60.0)
    args = ap.parse_args()

    base = snapshot()
    if base:
        print("开始时已有的端口：")
        for dev, (desc, hwid) in sorted(base.items()):
            print("   %-8s %s   [%s]" % (dev, desc, hwid))
    else:
        print("开始时没有任何串口。")
    print("\n>>> 现在请拔掉 CH340，等 2 秒，再插回去（盯 %.0f 秒）\n"
          % args.seconds)

    deadline = time.time() + args.seconds
    last = dict(base)
    any_change = False
    while time.time() < deadline:
        cur = snapshot()
        added = [d for d in cur if d not in last]
        removed = [d for d in last if d not in cur]
        for d in sorted(removed):
            any_change = True
            print("[-] %s  %s   消失了" % (time.strftime("%H:%M:%S"),
                                          last[d][0]))
        for d in sorted(added):
            any_change = True
            desc, hwid = cur[d]
            print("[+] %s  %s   出现了" % (time.strftime("%H:%M:%S"), d))
            print("       描述: %s" % desc)
            print("       硬件: %s" % hwid)
            print("       自检: %s" % probe(d))
        last = cur
        time.sleep(0.4)

    new = [d for d in last if d not in base]
    print()
    if new:
        print("结论：这次新出现 %s" % ", ".join(new))
    elif any_change:
        print("结论：期间有设备消失又出现，但没有新端口 —— "
              "多半是同一个口重新枚举，注意看上面的自检结果。")
    else:
        print("结论：%.0f 秒内没有任何变化。" % args.seconds)
        print("       说明 CH340 根本没被电脑认出来：")
        print("       · USB 线是充电线（只有 2 根线），换一根能传数据的")
        print("       · 没插紧 / 插在坏的口上")
        print("       · CH340 板子坏了")
        print("       排掉这些之后，设备管理器里应该出现"
              "「USB-SERIAL CH340 (COMx)」。")


if __name__ == "__main__":
    main()
