"""真机验证 LEARN 那条路（新 learn_from_center）。

要确认的事，按顺序：
  1) get_statistics(thresholds=..., roi=...) 真机到底认不认 —— 这是这次修复的核心，
     learn 靠它只统计"物体自己的像素"，不把背景算进去。
  2) 统计对象的 a_std_dev / b_std_dev / pixel_count 是否存在（threshold_from_stats 依赖）。
  3) 学出来的阈值是否被限制在该色系种子内、且回找时不是满幅。
  4) 这套调用的耗时（按一次按钮，慢点无所谓，但不能卡死）。

运行：python3 probe_learn.py
"""

import time

from maix import camera, image                     # noqa: F401
import maixcam_color_tracker as T

W, H = 640, 480
cam = camera.Camera(W, H)
img = cam.read()
print("image %dx%d" % (img.width(), img.height()))

# ---------------------------------------------------------------- 1) API 探底
print("\n--- 1) get_statistics 支持哪些参数")
roi = [W // 2 - 50, H // 2 - 40, 100, 80]

try:
    st0 = img.get_statistics(roi=roi)
    print("  roi only        : ok  a=%.1f b=%.1f" % (st0.a_mean(), st0.b_mean()))
except Exception as e:                                     # noqa: BLE001
    print("  roi only        : FAIL %s" % e)
    st0 = None

seed = T.COLOR_SEED["red"]
try:
    t0 = time.time()
    st1 = img.get_statistics(thresholds=[seed], roi=roi)
    dt = (time.time() - t0) * 1000
    print("  thresholds+roi  : ok  %.1fms a=%.1f b=%.1f" %
          (dt, st1.a_mean(), st1.b_mean()))
except Exception as e:                                     # noqa: BLE001
    print("  thresholds+roi  : FAIL %s" % e)
    st1 = None

for name in ("a_std_dev", "b_std_dev", "l_std_dev", "pixel_count"):
    for tag, stx in (("roi", st0), ("seed", st1)):
        if stx is None:
            continue
        try:
            v = getattr(stx, name)()
            print("  %-14s[%s] = %s" % (name, tag, v))
        except Exception as e:                             # noqa: BLE001
            print("  %-14s[%s] : MISSING (%s)" % (name, tag, e))

# ---------------------------------------------------------------- 2) 真实学习
print("\n--- 2) learn_from_center（对着当前画面）")
t0 = time.time()
nm, th, note = T.learn_from_center(img, W, H)
dt = (time.time() - t0) * 1000
print("  结果: name=%s  th=%s  %.0fms" % (nm, th, dt))
print("  说明: %s" % note)

if th:
    seed = T.COLOR_SEED[nm] if nm in T.COLOR_SEED else None
    if seed:
        inside = (th[2] >= seed[2] and th[3] <= seed[3]
                  and th[4] >= seed[4] and th[5] <= seed[5])
        print("  在色系种子内: %s (seed=%s)" % (inside, seed))
    print("  亮度全开: %s" % (th[0] == 0 and th[1] == 100))

    try:
        blobs = img.find_blobs([th], pixels_threshold=100)
        worst = 0
        for b in blobs or []:
            area = b[2] * b[3]
            if area > worst:
                worst = area
        print("  回找: %d 个框，最大占全幅 %.1f%%" %
              (len(blobs or []), 100.0 * worst / (W * H)))
    except Exception as e:                                 # noqa: BLE001
        print("  回找: FAIL %s" % e)

# ---------------------------------------------------------------- 3) 耗时
print("\n--- 3) 耗时（学一次的总代价）")
t0 = time.time()
for _ in range(5):
    T.learn_from_center(img, W, H)
print("  learn 平均 %.1fms" % ((time.time() - t0) * 1000 / 5.0))
print("\n完成")
