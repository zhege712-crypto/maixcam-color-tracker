#!/usr/bin/env python3
"""
性能探针：找出 scan_objects 里到底哪一步最贵
============================================

在真机上分别计时：
    · find_blobs（单色 / 三色，stride 2）
    · get_statistics（小 ROI，也就是"洞/环带"取样用的大小）
    · get_pixel

用法：
    python3 bench_probe.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maix import camera, image                    # noqa: E402
import maixcam_color_tracker as core             # noqa: E402


def ms(t0):
    return (time.time() - t0) * 1000.0


def main():
    cam = camera.Camera(640, 480)
    img = cam.read()
    print("=" * 56)
    print("scan_objects 性能分解   %dx%d" % (img.width(), img.height()))
    print("=" * 56)

    # 预热，避免第一次调用把驱动初始化时间算进去
    img.find_blobs(core.THRESHOLDS["red"], pixels_threshold=500,
                   x_stride=2, y_stride=2)
    img.get_statistics(roi=[100, 100, 40, 20])

    N = 20

    # ---- find_blobs：单色 ----
    t0 = time.time()
    for _ in range(N):
        b = img.find_blobs(core.THRESHOLDS["red"], pixels_threshold=500,
                           area_threshold=500, merge=False,
                           x_stride=2, y_stride=2)
    t_blobs1 = ms(t0) / N
    print("find_blobs 单色 stride2 : %6.2f ms  (%d 个色块)" % (t_blobs1, len(b or [])))

    # ---- find_blobs：三色 ----
    t0 = time.time()
    for _ in range(N):
        for c in core.THRESHOLDS:
            img.find_blobs(core.THRESHOLDS[c], pixels_threshold=500,
                           area_threshold=500, merge=False,
                           x_stride=2, y_stride=2)
    t_blobs3 = ms(t0) / N
    print("find_blobs 三色 stride2 : %6.2f ms" % t_blobs3)

    # ---- get_statistics：和 probe_hole 一样大的两块 ROI ----
    band = [200, 100, 40, 20]
    hole = [200, 120, 68, 68]
    t0 = time.time()
    for _ in range(N * 10):
        img.get_statistics(roi=band)
    t_stat = ms(t0) / (N * 10)
    print("get_statistics 环带(40x20): %6.2f ms" % t_stat)

    t0 = time.time()
    for _ in range(N * 10):
        img.get_statistics(roi=hole)
    t_stat2 = ms(t0) / (N * 10)
    print("get_statistics 洞(68x68)  : %6.2f ms" % t_stat2)

    # ---- get_pixel ----
    t0 = time.time()
    for _ in range(N * 50):
        img.get_pixel(320, 240)
    t_pix = ms(t0) / (N * 50)
    print("get_pixel 单点           : %6.3f ms" % t_pix)

    # ---- 整帧读取 ----
    t0 = time.time()
    for _ in range(N):
        cam.read()
    t_read = ms(t0) / N
    print("cam.read()               : %6.2f ms" % t_read)

    print("-" * 56)
    # 一个目标 = 2 次 get_statistics
    per_obj = t_stat + t_stat2
    print("每个目标取样成本         : %6.2f ms" % per_obj)
    for n in (1, 3, 6, 10):
        total = t_read + t_blobs3 + n * per_obj
        print("  %2d 个目标 -> 预估 %.1f fps" % (n, 1000.0 / total))

    print("-" * 56)
    t0 = time.time()
    objs = core.scan_objects(img, img.width(), img.height())
    t_scan = ms(t0)
    print("scan_objects 实测        : %6.2f ms  (%d 个目标) -> %.1f fps"
          % (t_scan, len(objs), 1000.0 / (t_read + t_scan)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
