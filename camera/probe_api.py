#!/usr/bin/env python3
"""
API 能力探针
============

在做优化之前先确认这版 MaixPy 到底支不支持两件关键的事：

    1. find_blobs 一次传多段阈值，并用 blob.code() 区分是哪一段命中的
       -> 支持的话，三色扫描从 3 次合并成 1 次，直接省一半时间
    2. get_pixel 拿到的 RGB 能不能用来替代 get_statistics 做"洞是不是空的"
       -> get_pixel 比 get_statistics 便宜约 80 倍

用法：
    python3 probe_api.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maix import camera                    # noqa: E402
import maixcam_color_tracker as core       # noqa: E402


def ms(t0):
    return (time.time() - t0) * 1000.0


def main():
    cam = camera.Camera(640, 480)
    img = cam.read()

    print("=" * 56)
    print("MaixPy API 能力探针")
    print("=" * 56)

    # ---------------- 1. 多段阈值 + code() ----------------
    flat = []
    owner = []
    for color in core.THRESHOLDS:
        for rng in core.THRESHOLDS[color]:
            flat.append(rng)
            owner.append(color)
    print("[INFO] 一次传入 %d 段阈值：%s" % (len(flat), owner))

    N = 10
    t0 = time.time()
    blobs = None
    for _ in range(N):
        blobs = img.find_blobs(flat, pixels_threshold=500, area_threshold=500,
                               merge=False, x_stride=2, y_stride=2)
    t_multi = ms(t0) / N
    print("find_blobs 一次多段   : %6.2f ms  (%d 个色块)" % (t_multi, len(blobs or [])))

    has_code = False
    if blobs:
        b = blobs[0]
        code = getattr(b, "code", None)
        print("[INFO] blob.code 存在: %s" % (code is not None))
        if code is not None:
            try:
                v = code()
                print("[INFO] blob.code() = %s  -> 归属颜色 %s"
                      % (v, owner[v] if isinstance(v, int) and 0 <= v < len(owner) else "?"))
                has_code = True
            except Exception as e:             # noqa: BLE001
                print("[WARN] blob.code() 调用失败: %s" % e)
        # 顺带看看还有哪些属性
        names = [n for n in dir(b) if not n.startswith("_")]
        print("[INFO] blob 可用方法: %s" % " ".join(sorted(names)))
    else:
        print("[WARN] 当前画面没有色块，无法验证 code()")

    # ---------------- 2. get_pixel 成本 ----------------
    t0 = time.time()
    for _ in range(500):
        px = img.get_pixel(320, 240)
    t_pix = ms(t0) / 500
    print("get_pixel            : %6.3f ms  返回 %s" % (t_pix, px))

    # ---------------- 3. 单段 vs 多段 对比 ----------------
    t0 = time.time()
    for _ in range(N):
        img.find_blobs(core.THRESHOLDS["red"], pixels_threshold=500,
                       area_threshold=500, merge=False, x_stride=2, y_stride=2)
    t_one = ms(t0) / N
    print("find_blobs 单段      : %6.2f ms" % t_one)
    print("-" * 56)
    print("结论：多段合并 %s，code() %s"
          % ("更快" if t_multi < t_one * 2 else "没有明显优势",
             "可用" if has_code else "不可用（需要按颜色分开调用）"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
