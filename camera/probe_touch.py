#!/usr/bin/env python3
"""
触摸屏 / 按键能力探针
====================

现场可能没有电脑，所以想用相机自己的触摸屏做操作界面。
先确认这台设备到底提供了什么接口。

用法：
    python3 probe_touch.py
"""

import sys


def try_import(name):
    try:
        mod = __import__("maix." + name, fromlist=[name])
        return mod
    except Exception as e:                    # noqa: BLE001
        print("[ -- ] maix.%s 不可用：%s" % (name, e))
        return None


def main():
    print("=" * 56)
    print("触摸 / 输入设备能力探针")
    print("=" * 56)

    for name in ("touchscreen", "display", "key", "app", "pinmap"):
        mod = try_import(name)
        if mod is not None:
            print("[ OK ] maix.%s" % name)

    ts_mod = try_import("touchscreen")
    if ts_mod is not None:
        try:
            ts = ts_mod.TouchScreen()
            print("[INFO] 构造成功：%s" % ts)
            names = [m for m in dir(ts) if not m.startswith("_")]
            print("[INFO] 可用方法：%s" % " ".join(sorted(names)))
            try:
                r = ts.read()
                print("[INFO] 读一次：%s" % (r,))
            except Exception as e:            # noqa: BLE001
                print("[WARN] read() 失败：%s" % e)
        except Exception as e:                # noqa: BLE001
            print("[FAIL] 构造触摸屏失败：%s" % e)

    key_mod = try_import("key")
    if key_mod is not None:
        print("[INFO] key 模块函数：%s"
              % " ".join(m for m in dir(key_mod) if not m.startswith("_")))

    # 看看 /dev/input 下有没有触摸设备
    try:
        import os
        devs = []
        for d in ("/dev/input", "/dev"):
            if os.path.isdir(d):
                for f in sorted(os.listdir(d)):
                    if "event" in f or "touch" in f.lower() or "ts" == f:
                        devs.append(os.path.join(d, f))
        print("[INFO] 输入设备节点：%s" % (devs or "无"))
    except Exception as e:                    # noqa: BLE001
        print("[WARN] 列设备失败：%s" % e)

    return 0


if __name__ == "__main__":
    sys.exit(main())
