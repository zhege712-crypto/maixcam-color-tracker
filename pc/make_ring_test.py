#!/usr/bin/env python3
"""
生成"环 + 块"测试图
==================

给设备端自检用（camera/vision_selftest.py）：一张图里放三种标准目标，
让相机程序在**真实的 MaixPy image 对象**上跑一遍环/块判定。

颜色是特意选的 —— 都落在程序出厂阈值范围内，这样连"阈值配错了"
也能一起验出来：
    砖红 (178,34,34)   -> THRESHOLDS['red']   [[0,100,40,80,10,95]]
    深绿 (20,120,40)   -> THRESHOLDS['green'] [[0,80,-120,-10,0,30]]
    深蓝 (30,60,200)   -> THRESHOLDS['blue']  [[0,80,30,100,-120,-60]]

用法：
    python pc/make_ring_test.py [输出路径]
"""

import os
import sys

from PIL import Image, ImageDraw


RED = (178, 34, 34)
GREEN = (20, 120, 40)
BLUE = (30, 60, 200)

# 和相机端一致：640x480
W, H = 640, 480

# 期望结果：(颜色, 形状, 中心x, 中心y, 外径)
EXPECT = [
    ("red", "ring", 160, 240, 180),    # 红环，外径 180
    ("red", "block", 400, 200, None),  # 红方块 70x70
    ("blue", "ring", 480, 380, 140),   # 蓝环，外径 140
    ("green", "ring", 560, 110, 120),  # 绿环，外径 120
]


def rect(x, y, w, h):
    return (x, y, x + w, y + h)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    out = argv[0] if argv else os.path.join(root, "tests", "ring_test.png")

    img = Image.new("RGB", (W, H), (235, 235, 230))     # 浅灰白桌面
    d = ImageDraw.Draw(img)

    # 红环（外径 180，内径 110）
    d.ellipse(rect(160 - 90, 240 - 90, 180, 180), fill=RED)
    d.ellipse(rect(160 - 55, 240 - 55, 110, 110), fill=(235, 235, 230))

    # 红方块（实心）
    d.rectangle(rect(400 - 35, 200 - 35, 70, 70), fill=RED)

    # 蓝环（外径 140，内径 86）
    d.ellipse(rect(480 - 70, 380 - 70, 140, 140), fill=BLUE)
    d.ellipse(rect(480 - 43, 380 - 43, 86, 86), fill=(235, 235, 230))

    # 绿环（外径 120，内径 72）
    d.ellipse(rect(560 - 60, 110 - 60, 120, 120), fill=GREEN)
    d.ellipse(rect(560 - 36, 110 - 36, 72, 72), fill=(235, 235, 230))

    img.save(out)
    print("wrote %s (%d bytes, %dx%d)" % (out, os.path.getsize(out), W, H))
    print("期望检测到 %d 个目标：" % len(EXPECT))
    for color, kind, cx, cy, dia in EXPECT:
        print("  %-5s %-5s 中心(%d,%d) %s" %
              (color, kind, cx, cy, ("外径 %d" % dia) if dia else "70x70"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
