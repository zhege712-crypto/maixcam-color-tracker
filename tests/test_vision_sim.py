"""
视觉链路仿真测试（不需要相机）
==============================

用 `tests/fake_camera.py` 造出"有环、有块、有地面"的假图像，把相机端真正的
`scan_objects()` → `Mission.update()` → `frame_from_object()` 整条链路跑一遍。

这样能在没有实物色环的情况下验证：
    · 环和块分得开（同一种颜色！）
    · 环上已经放了块的情况
    · 低色度物体不会被当成目标
    · 多条阈值范围重复命中同一个物体时会去重
    · 状态机拿到的是"真"检测结果而不是手写的字典

运行：
    python tests/test_vision_sim.py
"""

import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "pc"))
sys.path.insert(0, os.path.join(ROOT, "tests"))

import protocol as P          # noqa: E402
import fake_camera as FC      # noqa: E402


def load_module(name, rel_path):
    path = os.path.join(ROOT, rel_path)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CAM = load_module("maixcam_color_tracker", os.path.join("camera", "maixcam_color_tracker.py"))

# 仿真用的 LAB 阈值范围（覆盖 fake_camera 里那三种颜色）
SIM_THRESHOLDS = {
    "red":   [[60, 180, 40, 95, 5, 60]],
    "green": [[60, 200, -95, -25, 0, 60]],
    "blue":  [[60, 180, -60, 5, -95, -25]],
}

W, H = 320, 240

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


def setup():
    """把相机端的阈值换成仿真阈值（只改这个测试进程里的模块状态）。"""
    CAM.THRESHOLDS = {k: [list(r) for r in v] for k, v in SIM_THRESHOLDS.items()}
    CAM.CONFIG["pixels_threshold"] = 300


def image(L, A, B):
    return FC.FakeImage(L, A, B)


def kinds(objs):
    return sorted(o["kind"] for o in objs)


def color_kinds(objs):
    return sorted((o["color"], o["kind"]) for o in objs)


# ---------------------------------------------------------------- 用例

def test_block_and_ring():
    section("同色环和块要能分开")
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 40, 95, 50, 50, FC.lab_red())          # 实心块
    FC.draw_ring(L, A, B, 230, 120, 40, 24, FC.lab_red())        # 空心环
    sim = image(L, A, B)
    objs = CAM.scan_objects(sim, W, H)

    check(len(objs) == 2, "应该只检测到 2 个目标，得到 %d（%s）",
          len(objs), color_kinds(objs))
    check(kinds(objs) == ["block", "ring"], "必须一个块一个环，得到 %s", kinds(objs))
    check(all(o["color"] == "red" for o in objs), "两个都应是红色")

    blk = [o for o in objs if o["kind"] == "block"][0]
    rng = [o for o in objs if o["kind"] == "ring"][0]
    check(abs(blk["cx"] - 65) <= 2, "块的横向中心应≈65，得到 %.1f", blk["cx"])
    check(abs(rng["cx"] - 230) <= 2, "环的横向中心应≈230，得到 %.1f", rng["cx"])

    # 性能相关的硬性要求：形状判定不该再动 get_statistics（一次 1~4ms）
    check(sim.stats_calls == 0,
          "scan_objects 不该调用 get_statistics，实际调了 %d 次",
          sim.stats_calls)


def test_ring_only():
    section("只有环")
    L, A, B = FC.blank(W, H)
    FC.draw_ring(L, A, B, 160, 120, 45, 27, FC.lab_blue())
    objs = CAM.scan_objects(image(L, A, B), W, H)
    check(len(objs) == 1, "应只检测到 1 个目标，得到 %d", len(objs))
    check(objs and objs[0]["kind"] == "ring", "蓝环必须判成环，得到 %s",
          objs and objs[0]["kind"])
    check(objs and objs[0]["color"] == "blue", "颜色应为蓝")


def test_disc_is_block():
    section("实心圆片是块")
    L, A, B = FC.blank(W, H)
    FC.draw_disc(L, A, B, 160, 120, 30, FC.lab_green())
    objs = CAM.scan_objects(image(L, A, B), W, H)
    check(len(objs) == 1, "应检测到 1 个目标，得到 %d", len(objs))
    check(objs and objs[0]["kind"] == "block", "实心圆片必须判成块，得到 %s",
          objs and objs[0]["kind"])
    check(objs and objs[0]["color"] == "green", "颜色应为绿")


def test_occupied_ring():
    section("环上已经放了块")
    L, A, B = FC.blank(W, H)
    FC.draw_ring(L, A, B, 160, 120, 42, 24, FC.lab_red())
    FC.draw_disc(L, A, B, 160, 120, 26, FC.lab_red())   # 块把洞填住了
    objs = CAM.scan_objects(image(L, A, B), W, H)
    check(len(objs) == 1, "被填住的环应只算 1 个目标，得到 %d", len(objs))
    check(objs and objs[0]["kind"] == "block",
          "洞被填住后应判成块（= 这个环已经用掉了），得到 %s",
          objs and objs[0]["kind"])


def test_gray_ignored():
    section("低色度物体不算目标")
    L, A, B = FC.blank(W, H)
    FC.draw_disc(L, A, B, 160, 120, 35, (140, 2, 3))     # 灰色圆片
    objs = CAM.scan_objects(image(L, A, B), W, H)
    check(len(objs) == 0, "灰色物体不该被任何颜色的阈值命中，得到 %d", len(objs))


def test_multi_range_dedup():
    section("多段阈值重复命中同物体要去重")
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 120, 90, 60, 60, FC.lab_red())
    # 故意给红色两段互相重叠的范围，模拟"块和环材料不同"的配置
    CAM.THRESHOLDS = {"red": [[60, 180, 40, 95, 5, 60],
                              [100, 160, 50, 90, 10, 50]],
                      "green": SIM_THRESHOLDS["green"],
                      "blue": SIM_THRESHOLDS["blue"]}
    objs = CAM.scan_objects(image(L, A, B), W, H)
    check(len(objs) == 1, "两段阈值命中同一个物体必须去重成 1 个，得到 %d", len(objs))
    setup()


def test_mission_on_real_scan():
    section("状态机 + 真实扫描结果")
    m = CAM.Mission(order=("red", "green", "blue"),
                    grab_point=(128, 128), drop_point=(128, 128),
                    tol_n=12, grab_w_n=35, drop_w_n=55, hold_frames=2)

    # 1) 物料在远处
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 250, 170, 50, 50, FC.lab_red())
    objs = CAM.scan_objects(image(L, A, B), W, H)
    for _ in range(3):
        d = m.update(objs)
    check(d["state"] == CAM.STATE_MATERIAL, "远处应按对准物料，得到 %s",
          CAM.STATE_TEXT.get(d["state"]))
    check(d["color"] == "red", "应锁定红色")

    # 2) 物料移到正中且变大 -> 可抓取
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 130, 90, 60, 60, FC.lab_red())
    objs = CAM.scan_objects(image(L, A, B), W, H)
    for _ in range(3):
        d = m.update(objs)
    check(d["state"] == CAM.STATE_GRAB, "正中且够大应可抓取，得到 %s",
          CAM.STATE_TEXT.get(d["state"]))
    frame, _ = CAM.frame_from_object(7, d["target"], d["state"], d["color"])
    check(frame[7] == CAM.STATE_GRAB, "上行帧的 state 字段应是可抓取")
    check(frame[12] == sum(frame[4:12]) & 0xFF, "上行帧校验和错误")

    m.advance()

    # 3) 目标区在视野里 -> 对准目标区
    L, A, B = FC.blank(W, H)
    FC.draw_ring(L, A, B, 90, 120, 45, 27, FC.lab_red())
    objs = CAM.scan_objects(image(L, A, B), W, H)
    for _ in range(3):
        d = m.update(objs)
    check(d["state"] == CAM.STATE_DEST, "应按对准目标区，得到 %s",
          CAM.STATE_TEXT.get(d["state"]))

    # 4) 目标区居中且够大 -> 可放置
    L, A, B = FC.blank(W, H)
    FC.draw_ring(L, A, B, 160, 120, 60, 36, FC.lab_red())
    objs = CAM.scan_objects(image(L, A, B), W, H)
    for _ in range(3):
        d = m.update(objs)
    check(d["state"] == CAM.STATE_DROP, "目标区居中且够大应可放置，得到 %s",
          CAM.STATE_TEXT.get(d["state"]))

    m.advance()
    check(m.progress == "A2/3" and m.color == "green",
          "放完后应进入第二个物料，得到 %s %s", m.progress, m.color)


def test_distance_metric():
    section("w_n 是距离的代理（越大越近）")
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 140, 100, 40, 40, FC.lab_red())
    near = CAM.scan_objects(image(L, A, B), W, H)[0]

    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 150, 110, 20, 20, FC.lab_red())
    far = CAM.scan_objects(image(L, A, B), W, H)[0]

    check(near["w_n"] > far["w_n"], "大目标 w_n 应更大：近 %d / 远 %d",
          near["w_n"], far["w_n"])
    check(near["x_n"] == 128 and far["x_n"] == 128, "两者中心都应在画面正中")


def maix_image(L, A, B):
    """给 learn_from_center 用：把 L 从 0..255 压到真机的 0..100 刻度。

    fake_camera 为了配合 SIM_THRESHOLDS 用 0..255 的 L，而真机 MaixPy 的 L
    就是 0..100（COLOR_SEED 和"亮度全开 [0,100]"都照真机写的）。
    A/B 两轴两边都是 -128..127，不需要换算。
    """
    import numpy as np
    return FC.FakeImage((L.astype(np.int16) * 100) // 255, A, B)


def test_learn_from_center():
    section("LEARN：只学画面正中那个物体，不许把背景学进去")
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 140, 100, 40, 40, FC.lab_red())     # 只占全幅 2%
    sim = maix_image(L, A, B)
    nm, th, note = CAM.learn_from_center(sim, W, H)

    check(nm == "red", "应学成 red，得到 %s (%s)", nm, note)
    check(th is not None, "应给出阈值")
    if th:
        check(th[0] == 0 and th[1] == 100, "亮度必须全开，得到 L%d-%d", th[0], th[1])
        seed = CAM.COLOR_SEED["red"]
        check(th[2] >= seed[2] and th[3] <= seed[3]
              and th[4] >= seed[4] and th[5] <= seed[5],
              "学到的窗口必须落在红色系种子内，得到 A%d-%d B%d-%d",
              th[2], th[3], th[4], th[5])
        # 旧实现（对中心 200x160 一大块直接算 mean±kσ）在这里会算出
        # 跨 100 多宽的 A 窗口，把浅灰背景一起框进去。
        check(th[3] - th[2] <= 45, "A 窗口过宽（背景被学进去了）：%d", th[3] - th[2])
        check(th[5] - th[4] <= 45, "B 窗口过宽（背景被学进去了）：%d", th[5] - th[4])

        # 学到的阈值必须"只框住那个块"，不能匹配满幅背景
        blobs = sim.find_blobs([th], pixels_threshold=100)
        worst = max((b[2] * b[3] for b in blobs), default=0)
        check(worst < W * H * 0.4, "学到阈值后最大框占全幅 %.0f%%，太宽",
              100.0 * worst / (W * H))


def test_learn_prefers_center():
    section("LEARN：边上有个更大的，也要学正中间那个")
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 20, 10, 110, 110, FC.lab_red())    # 又大又偏
    FC.draw_rect(L, A, B, 145, 105, 30, 30, FC.lab_blue())   # 小而正中
    nm, th, note = CAM.learn_from_center(maix_image(L, A, B), W, H)
    check(nm == "blue", "应学正中间的蓝色，得到 %s (%s)", nm, note)


def test_learn_rejects():
    section("LEARN：学不到东西要明说，不能硬塞一个坏阈值")
    L, A, B = FC.blank(W, H)
    nm, th, note = CAM.learn_from_center(maix_image(L, A, B), W, H)
    check(nm is None and th is None, "空白画面应失败")
    check("NO OBJECT" in note, "应提示中心没东西，得到 %s", note)

    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 140, 100, 60, 60, (200, -10, -20))   # 色度只有 22
    nm, th, note = CAM.learn_from_center(maix_image(L, A, B), W, H)
    check(nm is None, "淡色应被色度门挡掉，得到 %s (%s)", nm, note)
    check("PALE" in note, "应提示太淡，得到 %s", note)


def test_measure_center_target():
    section("G-SZ/D-SZ：在动作点量 grab_w_n，量完必须判得了「到位」")
    L, A, B = FC.blank(W, H)
    FC.draw_rect(L, A, B, 140, 100, 40, 40, FC.lab_red())
    sim = image(L, A, B)
    wn, note = CAM.measure_center_target(sim, W, H)
    check(wn is not None, "应量出宽度，得到 %s (%s)", wn, note)
    check(wn == 30, "40px/320*255*0.95 应≈30，得到 %s", wn)

    objs = CAM.scan_objects(sim, W, H)
    o = objs[0]
    check(o["w_n"] >= wn, "量出的门槛必须能让目标判为到位：%d vs %s",
          o["w_n"], wn)
    check(CAM.is_aligned(o, (128, 128), 40, wn), "标定后应判为已到位")
    # 这条是现场"卡在对准物料"的回归：占位值 90 在这个尺度下永远不成立
    check(not CAM.is_aligned(o, (128, 128), 40, 90),
          "占位值 90 不该成立（否则这个测试没意义）")

    L, A, B = FC.blank(W, H)
    wn2, note2 = CAM.measure_center_target(image(L, A, B), W, H)
    check(wn2 is None and "NO TARGET" in note2, "空白画面应报没目标，得到 %s",
          note2)


def main():
    print("=" * 60)
    print("视觉链路仿真测试（无硬件）")
    print("=" * 60)
    setup()
    test_block_and_ring()
    test_ring_only()
    test_disc_is_block()
    test_occupied_ring()
    test_gray_ignored()
    test_multi_range_dedup()
    test_mission_on_real_scan()
    test_distance_metric()
    test_learn_from_center()
    test_learn_prefers_center()
    test_learn_rejects()
    test_measure_center_target()

    print("\n" + "=" * 60)
    print("结果：通过 %d 项，失败 %d 项" % (_PASS, _FAIL))
    print("=" * 60)
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
