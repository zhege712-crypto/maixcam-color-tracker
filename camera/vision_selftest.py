#!/usr/bin/env python3
"""
环/块判定 —— 真机自检
=====================

在 **MaixCAM 上**用真实 MaixPy 图像对象跑一遍 `scan_objects()`，验证：

    · find_blobs 的跳点采样参数在这版 MaixPy 上真的支持
    · get_statistics(roi=list) 的调用方式对
    · blob 的像素数接口（pixels()/count()）拿得到
    · 环（中间是空的）和块（中间是实的）在真实图像数据上分得开
    · 出厂的色阈值确实能抓到对应颜色

不需要搭场地：测试图由 `pc/make_ring_test.py` 生成后 scp 上来即可。

用法：
    python3 vision_selftest.py [图片路径]
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import maixcam_color_tracker as core          # noqa: E402

DEFAULT_PATH = "/root/ring_test.png"

# 和 pc/make_ring_test.py 里的 EXPECT 必须一致
EXPECT = [
    ("red", "ring", 160, 240),
    ("red", "block", 400, 200),
    ("blue", "ring", 480, 380),
    ("green", "ring", 560, 110),
]
TOL_PX = 8        # 中心容差


def load_image(path):
    from maix import image
    last = None
    for maker in (lambda: image.load(path), lambda: image.Image(path)):
        try:
            return maker(), None
        except Exception as e:                # noqa: BLE001
            last = e
    return None, last


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else DEFAULT_PATH

    print("=" * 56)
    print("环/块判定 真机自检")
    print("=" * 56)

    if not os.path.exists(path):
        print("[FAIL] 找不到测试图：%s" % path)
        print("       先在电脑上跑 python pc/make_ring_test.py，再 scp 上来")
        return 1

    img, err = load_image(path)
    if img is None:
        print("[FAIL] 图片加载失败：%s" % err)
        return 1

    try:
        w, h = img.width(), img.height()
    except Exception:                          # noqa: BLE001
        w, h = img.shape[1], img.shape[0]
    print("[INFO] 已载入 %s  %dx%d" % (path, w, h))

    objs = core.scan_objects(img, w, h)
    print("[INFO] 检测到 %d 个候选目标：" % len(objs))
    for o in objs:
        print("   %-5s %-5s 框(%3d,%3d %3dx%d) 中心(%6.1f,%6.1f) "
              "w_n=%3d 填充=%.2f  %s"
              % (o["color"], o["kind"], o["x"], o["y"], o["w"], o["h"],
                 o["cx"], o["cy"], o["w_n"],
                 o["fill"] if o["fill"] is not None else -1,
                 o.get("probe", "")))

    # ---------------- 逐项比对 ----------------
    fails = 0
    used = set()
    print("-" * 56)
    for color, kind, ex, ey in EXPECT:
        hit = None
        for i, o in enumerate(objs):
            if i in used:
                continue
            if (o["color"] == color and o["kind"] == kind
                    and abs(o["cx"] - ex) <= TOL_PX
                    and abs(o["cy"] - ey) <= TOL_PX):
                hit = i
                break
        if hit is None:
            print("[FAIL] 期望 %s %s 在 (%d,%d) —— 没找到" % (color, kind, ex, ey))
            fails += 1
        else:
            used.add(hit)
            print("[ OK ] %s %s 在 (%d,%d)" % (color, kind, ex, ey))

    if len(objs) != len(EXPECT):
        print("[FAIL] 目标数量不对：期望 %d，实际 %d"
              "（多了通常是阈值误命中，少了通常是阈值太窄或环被拆开）"
              % (len(EXPECT), len(objs)))
        fails += 1
    else:
        print("[ OK ] 目标数量 %d 与期望一致" % len(objs))

    # ---------------- 状态机回放：到位 -> 可抓取 -> NEXT -> 可投放 ----------------
    print("-" * 56)
    print("[INFO] 状态机回放（用同一张图，动作点设成目标当前位置）")
    fails += mission_replay(objs, w, h)

    print("-" * 56)
    if fails:
        print("[FAIL] 自检未通过：%d 项不符合预期" % fails)
        return 1
    print("[ OK ] 自检通过：环/块在真实图像上判定正确，状态机流程正常")
    return 0


def feed(m, objs, n=3):
    """连续喂 n 帧（跨过防抖），返回最后一帧的决策。"""
    d = None
    for _ in range(n):
        d = m.update(objs)
    return d


def mission_replay(objs, w, h):
    """用真实检测结果跑一遍「对准物料 → 可抓取 → NEXT → 对准目标区 → 可放置」。

    把动作点设成目标**当前所在的归一化坐标**，等价于"机器人已经把目标
    摆到动作点上了"，这样在固定图片上也能验完整的阶段流转。
    """
    fails = 0
    blk = rng = None
    for o in objs:
        if o["color"] != "red":
            continue
        if o["kind"] == "block" and blk is None:
            blk = o
        if o["kind"] == "ring" and rng is None:
            rng = o
    if blk is None or rng is None:
        print("[FAIL] 图片里缺少红色物料或红色色标，无法回放")
        return 1

    def make(phase="src"):
        m = core.Mission(order=("red",),
                         grab_point=(blk["x_n"], blk["y_n"]),
                         drop_point=(rng["x_n"], rng["y_n"]),
                         tol_n=8, grab_w_n=10, drop_w_n=10, hold_frames=2)
        if phase == "dst":
            m.phase = "dst"
        return m

    # ---- 1. 物料已在抓取动作点 -> 可抓取 ----
    m = make()
    d = feed(m, objs)
    if d["state"] == core.STATE_GRAB and d["color"] == "red" and d["target"]:
        print("[ OK ] 物料在动作点上 -> state=可抓取（进度 %s）" % d["progress"])
    else:
        print("[FAIL] 物料在动作点上却没报可抓取：state=%s"
              % core.STATE_TEXT.get(d["state"]))
        fails += 1

    # ---- 2. 主控"抓完了" -> 切到找同色目标区 ----
    m.advance()
    if m.phase == "dst" and m.color == "red":
        print("[ OK ] 收到 NEXT 后进入对准目标区（颜色保持 red）")
    else:
        print("[FAIL] NEXT 后阶段不对：phase=%s color=%s" % (m.phase, m.color))
        fails += 1

    # ---- 3. 目标区不在放置动作点 -> 对准目标区 ----
    m2 = make("dst")
    m2.drop_point = (128, 128)          # 画面正中，而红环在左边
    d = feed(m2, objs)
    if d["state"] == core.STATE_DEST and d["target"] is not None:
        print("[ OK ] 目标区不在动作点上 -> state=对准目标区")
    else:
        print("[FAIL] 应报对准目标区，实际 state=%s"
              % core.STATE_TEXT.get(d["state"]))
        fails += 1

    # ---- 4. 目标区在放置动作点 -> 可放置 ----
    m3 = make("dst")
    d = feed(m3, objs)
    if d["state"] == core.STATE_DROP:
        print("[ OK ] 目标区在动作点上 -> state=可放置")
    else:
        print("[FAIL] 目标区在动作点上却没报可放置：state=%s"
              % core.STATE_TEXT.get(d["state"]))
        fails += 1

    # ---- 5. 放完后进度推进 ----
    m3.advance()
    if m3.batch is not None or m3.is_done:
        print("[ OK ] 收到 NEXT 后进度推进（%s，本批已完成 %s）"
              % (m3.progress, ",".join(sorted(m3.done)) or "无"))
    else:
        print("[FAIL] 放置后进度没有推进")
        fails += 1
    return fails


if __name__ == "__main__":
    sys.exit(main())
