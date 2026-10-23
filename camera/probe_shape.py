#!/usr/bin/env python3
"""
形状判据探针
============

问一个问题：环和块，能不能只靠 find_blobs 自带的几何量分开，
完全不调用 get_statistics？

候选（MaixPy 的 blob 都提供）：
    solidity()  = 像素数 / 凸包面积   —— 环的凸包是外圆，所以 1-k²
    density()   = 像素数 / 框面积     —— 也就是"填充率"
    extent()    = 和 density 类似
    roundness() = 越接近 1 越圆

用法：
    python3 probe_shape.py [图片路径]
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import maixcam_color_tracker as core       # noqa: E402

DEFAULT_PATH = "/root/ring_test.png"


def load_image(path):
    from maix import image
    for maker in (lambda: image.load(path), lambda: image.Image(path)):
        try:
            return maker()
        except Exception:              # noqa: BLE001
            continue
    return None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else DEFAULT_PATH

    img = load_image(path)
    if img is None:
        print("[FAIL] 载入失败：%s" % path)
        return 1

    print("=" * 62)
    print("形状判据探针   %s" % path)
    print("=" * 62)
    print("%-6s %-6s %6s %8s %8s %8s %8s %6s"
          % ("颜色", "框", "像素", "密度", "实心度", "圆度", "外接", "w_n"))

    t_sol = t_pix = 0.0
    n_calls = 0
    for color in core.THRESHOLDS:
        ths = core.THRESHOLDS[color]
        blobs = img.find_blobs(ths, pixels_threshold=500, area_threshold=500,
                               merge=False, x_stride=2, y_stride=2)
        for b in (blobs or []):
            # 计时：solidity / pixels 各调 50 次
            t0 = time.time()
            for _ in range(50):
                s = b.solidity()
            t_sol += (time.time() - t0) * 1000.0 / 50.0
            t0 = time.time()
            for _ in range(50):
                p = b.pixels()
            t_pix += (time.time() - t0) * 1000.0 / 50.0
            n_calls += 1
            print("%-6s %-6s %6d %8.3f %8.3f %8.3f %8.3f %6d"
                  % (color, "%dx%d" % (b[2], b[3]), b.pixels(),
                     b.density(), b.solidity(), b.roundness(),
                     b.extent() if hasattr(b, "extent") else -1,
                     core.norm255(b[2], img.width())))

    if n_calls:
        print("-" * 62)
        print("单次调用成本：solidity %.3f ms   pixels %.3f ms"
              % (t_sol / n_calls, t_pix / n_calls))
        print()
        print("判读方法：")
        print("  固体块的实心度应接近 1.0；环 = 1-k²（k=内径/外径），")
        print("  例如 k=0.6 -> 0.64，k=0.7 -> 0.51，k=0.8 -> 0.36。")
        print("  若两者差距明显，就可以只用 solidity 判形状，不用再取样。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
