"""
v3.0 搬运任务逻辑测试
=====================

覆盖"形状判定 + 目标选择 + 两批任务状态机 + 任务码 + 触摸界面 + 上行帧"
这些**不依赖硬件**的部分。相机端程序里的这些函数都是纯逻辑，
在电脑上直接加载就能测，不需要摄像头。

流程依据《附件1-智能搬运比赛规则》：
    第一批：原料区 3 个物料 --按任务码顺序--> 粗加工区对应颜色区域
    第二批：粗加工区 3 个物料 --同样顺序--> 暂存区对应颜色区域

运行：
    python tests/test_mission.py
"""

import importlib.util
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "pc"))

import protocol as P  # noqa: E402


def load_module(name, rel_path):
    path = os.path.join(ROOT, rel_path)
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CAM = load_module("maixcam_color_tracker", os.path.join("camera", "maixcam_color_tracker.py"))

W, H = 640, 480

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


def obj(color, cx=320, cy=240, size=200, kind="block", img_w=W, img_h=H):
    """按中心点造一个目标字典。"""
    return CAM.make_object(kind, color,
                           (int(cx - size / 2), int(cy - size / 2), size, size),
                           img_w, img_h, fill=0.8)


# ---------------------------------------------------------------- 形状判定

def test_hole_judgement():
    section("洞/环带比对（RGB）")
    red = (178, 34, 34)
    white = (235, 235, 230)

    check(CAM.rgb_dist(red, red) == 0.0, "同色距离应为 0")
    check(CAM.rgb_dist(red, white) > 200, "红和白应该离得很远")

    check(CAM.hole_is_background_rgb([white] * 5, red) is True,
          "洞里全是背景色时应判成环")
    check(CAM.hole_is_background_rgb([red] * 5, red) is False,
          "洞和环带同色时应判成实心")
    check(CAM.hole_is_background_rgb([red, red, red, white, white], red) is False,
          "多数点同色就应判实心")
    check(CAM.hole_is_background_rgb([red, red, white, white, white], red) is True,
          "多数点不同色就应判环")
    check(CAM.hole_is_background_rgb([red, red, (255, 250, 245), red, red], red) is False,
          "一个高光点不该把实心块判成环")
    check(CAM.hole_is_background_rgb([], red) is None, "没采到样应返回 None")
    check(CAM.hole_is_background_rgb([red], None) is None, "没有环带色应返回 None")


def test_decide_shape():
    section("形状判定")
    check(CAM.decide_shape(True, 0.95) == "ring", "洞是空的就是环，与填充率无关")
    check(CAM.decide_shape(False, 0.30) == "block", "洞是同色的就是块，与填充率无关")
    check(CAM.decide_shape(None, 0.40) == "ring", "填充率 0.40 明确是环")
    check(CAM.decide_shape(None, 0.90) == "block", "填充率 0.90 明确是块")
    check(CAM.decide_shape(None, 0.75) == "block",
          "模糊区间默认判块：误判成环会永远抓不到")
    check(CAM.decide_shape(None, None) == "block", "什么都不知道时默认判块")


def test_make_object():
    section("目标字典与归一化")
    o = obj("red", 320, 240, 128)
    check(o["x_n"] == 128, "画面正中 x_n 应为 128，得到 %d", o["x_n"])
    check(o["y_n"] == 128, "画面正中 y_n 应为 128，得到 %d", o["y_n"])
    check(o["w_n"] == CAM.norm255(128, W), "w_n 错误")
    check(o["h_n"] == CAM.norm255(128, W),
          "h_n 也必须除以画面宽度（和协议定义一致）")

    o2 = obj("red", 160, 120, 64, img_w=320, img_h=240)
    check(o["w_n"] == o2["w_n"], "同一比例尺寸在不同分辨率下 w_n 应相同")


def test_dedup():
    section("重复目标去重")
    a = obj("red", 100, 100, 100)
    b = obj("red", 104, 102, 96)
    c = obj("blue", 500, 400, 120)
    kept = CAM.dedup_objects([a, b, c])
    check(len(kept) == 2, "重叠的应合并成 2 个，得到 %d", len(kept))
    check(any(o["w"] == 100 for o in kept), "去重应保留面积大的那个")
    check(CAM.rect_iou((0, 0, 10, 10), (100, 100, 10, 10)) == 0.0, "不相交 IoU 应为 0")


# ---------------------------------------------------------------- 任务码

def test_task_code():
    section("任务码解析")
    check(CAM.parse_task_code("123") == ("red", "green", "blue"), "123 解析错误")
    check(CAM.parse_task_code("321") == ("blue", "green", "red"), "321 解析错误")
    check(CAM.parse_task_code("112") == ("red", "red", "green"),
          "112（允许重复）解析错误")
    check(CAM.parse_task_code("1") == ("red",), "单个数字应可用")
    check(CAM.parse_task_code("124") is None, "含 4 必须判非法")
    check(CAM.parse_task_code("") is None, "空串必须判非法")
    check(CAM.parse_task_code(None) is None, "None 必须判非法")
    check(CAM.parse_task_code("1 2") is None, "空格必须判非法")


# ---------------------------------------------------------------- 目标选择

def test_pick_largest():
    section("放置目标选择：最大的同色块")
    small = obj("red", 100, 100, 80)
    big = obj("red", 400, 300, 240)
    other = obj("blue", 320, 240, 200)

    t = CAM.pick_largest([small, big, other], "red")
    check(t is not None and t["w"] == 240,
          "应选面积最大的同色块（不按形状挑：色环可能只有 2 像素宽）")
    check(CAM.pick_largest([other], "red") is None, "没有该颜色应返回 None")

    t = CAM.pick_largest([small, big], "red", hold_point=(big["x_n"], big["y_n"]),
                         hold_ignore_n=40)
    check(t is not None and t["w"] == 80,
          "落在 hold_point 附近的目标必须被排除，得到 %s", t and t["w"])
    t = CAM.pick_largest([big], "red", hold_point=(big["x_n"], big["y_n"]),
                         hold_ignore_n=40)
    check(t is None, "只剩自己抓着的物料时应返回 None（该去别处找）")


def test_pick_nearest():
    section("抓取目标选择：离动作点最近")
    a = obj("red", 100, 240, 80)
    b = obj("red", 560, 240, 80)
    t = CAM.pick_nearest([a, b], "red", (128, 128))
    check(t is not None and t["cx"] < 320, "应挑离动作点近的那个")
    check(CAM.pick_nearest([a], "green", (128, 128)) is None, "颜色过滤失效")


def test_aligned():
    section("到位判定")
    p = (128, 128)
    o = obj("red", 320, 240, 200)
    check(CAM.is_aligned(o, p, 12, 0), "正中的目标应判为到位")
    check(not CAM.is_aligned(o, p, 12, 255), "w_n 门限太高时不该判到位")
    o2 = obj("red", 460, 240, 200)
    check(not CAM.is_aligned(o2, p, 12, 0), "偏出容差的目标不该判到位")
    check(CAM.is_aligned(o2, p, 80, 0), "放宽容差后应判到位")


# ---------------------------------------------------------------- 状态机

def mission(order="123", **kw):
    kw.setdefault("grab_w_n", 70)
    kw.setdefault("drop_w_n", 70)
    kw.setdefault("hold_frames", 2)
    return CAM.Mission(order=CAM.parse_task_code(order),
                       grab_point=(128, 128), drop_point=(128, 128),
                       tol_n=12, **kw)


def feed(m, objs, n=3):
    d = None
    for _ in range(n):
        d = m.update(objs)
    return d


def test_mission_order():
    section("状态机：按任务码顺序")
    m = mission("321")
    check(m.order == ["blue", "green", "red"], "顺序应为 321，得到 %s", m.order)
    check(m.color == "blue", "第一次应该搬蓝色")
    d = feed(m, [obj("red"), obj("green"), obj("blue")])
    check(d["color"] == "blue",
          "画面里三种颜色都有时，必须按任务码先拿蓝色，得到 %s", d["color"])


def test_mission_one_cycle():
    section("状态机：一个物料的两段")
    m = mission("123")

    d = feed(m, [])
    check(d["state"] == CAM.STATE_SEARCH, "空画面应报搜索")

    far = [obj("red", 560, 400, 80)]
    d = feed(m, far)
    check(d["state"] == CAM.STATE_MATERIAL, "物料在远处应报对准物料，得到 %s",
          CAM.STATE_TEXT.get(d["state"]))
    check(d["color"] == "red", "应锁定红色")

    near = [obj("red", 320, 240, 200)]
    d = feed(m, near)
    check(d["state"] == CAM.STATE_GRAB, "物料居中且够大应报可抓取")
    check(d["progress"] == "A1/3", "进度应为 A1/3，得到 %s", d["progress"])

    m.advance()
    check(m.phase == "dst", "抓完后应进入去放置阶段")
    check(m.color == "red", "颜色不能丢")

    d = feed(m, [])
    check(d["state"] == CAM.STATE_SEARCH, "看不见目标区时应报搜索")

    d = feed(m, [obj("red", 200, 120, 90)])
    check(d["state"] == CAM.STATE_DEST, "目标区在远处应报对准目标区，得到 %s",
          CAM.STATE_TEXT.get(d["state"]))

    d = feed(m, [obj("red", 320, 240, 200)])
    check(d["state"] == CAM.STATE_DROP, "目标区居中且够大应报可放置")

    m.advance()
    check(m.progress == "A2/3", "应进入第二个物料，得到 %s", m.progress)
    check(m.color == "green", "第二个应是绿色")
    check(m.phase == "src", "应回到去抓阶段")


def test_mission_two_batches():
    section("状态机：两批共 6 次搬运")
    m = mission("123")
    seq = []
    for _ in range(6):
        seq.append((m.progress, m.color))
        d = feed(m, [obj(m.color, 320, 240, 200)])
        check(d["state"] == CAM.STATE_GRAB, "%s 应报可抓取" % (m.color,))
        m.advance()
        d = feed(m, [obj(m.color, 320, 240, 200)])
        check(d["state"] == CAM.STATE_DROP, "%s 应报可放置" % (m.color,))
        m.advance()

    check(m.is_done, "6 次搬运后应全部完成")
    check(seq[0] == ("A1/3", "red") and seq[1] == ("A2/3", "green")
          and seq[2] == ("A3/3", "blue"), "第一批顺序错误：%s", seq[:3])
    check(seq[3] == ("B1/3", "red") and seq[4] == ("B2/3", "green")
          and seq[5] == ("B3/3", "blue"), "第二批顺序错误：%s", seq[3:])
    check(m.action_count == 12, "6 次搬运共 12 次动作，得到 %d", m.action_count)

    d = feed(m, [obj("red", 320, 240, 200)])
    check(d["state"] == CAM.STATE_DONE, "完成后必须报 DONE")
    check(d["target"] is None, "完成后不该再给目标")


def test_mission_hysteresis():
    section("状态机：迟滞（防抖动）")
    m = mission("123")
    far = [obj("red", 560, 400, 80)]
    near = [obj("red", 320, 240, 200)]
    feed(m, far, 3)
    check(m.state == CAM.STATE_MATERIAL, "应先进对准物料")

    m.update(near)
    check(m.state == CAM.STATE_MATERIAL, "单帧到位不该立刻切换")
    m.update(near)
    check(m.state == CAM.STATE_GRAB, "连续两帧到位才切到可抓取")


def test_mission_abort():
    section("状态机：放弃")
    m = mission("123")
    feed(m, [obj("red", 320, 240, 200)])
    m.advance()
    check(m.phase == "dst", "应已进入放置阶段")
    m.abort()
    check(m.phase == "src", "放弃后应回到去抓阶段")
    check(m.color == "red", "放弃不该丢颜色/进度")


def test_mission_hold_point():
    section("状态机：不会对自己抓着的物料对准")
    m = mission("123", hold_point=(40, 128), hold_ignore_n=40)
    payload = obj("red", 100, 240, 120)
    d = feed(m, [payload])
    check(d["state"] == CAM.STATE_SEARCH,
          "只有自己抓着的物料时应报搜索（否则会原地打转），得到 %s",
          CAM.STATE_TEXT.get(d["state"]))

    target = obj("red", 320, 240, 200)
    d = feed(m, [payload, target])
    check(d["target"] is not None and d["target"]["cx"] == target["cx"],
          "应绕过车上物料，对准真正的目标")


# ---------------------------------------------------------------- 触摸界面

def test_field_ui():
    section("现场触摸界面（布局/命中/任务码输入）")
    ui = CAM.FieldUI(552, 368)
    check(len(ui.rects) == 10, "应有 10 个按钮，得到 %d", len(ui.rects))

    for name, (x, y, w, h) in ui.rects.items():
        check(ui.hit(x + w // 2, y + h // 2) == name,
              "按钮 %s 的中心没命中自己", name)

    names = list(ui.rects)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            ax, ay, aw, ah = ui.rects[names[i]]
            bx, by, bw, bh = ui.rects[names[j]]
            overlap = (ax < bx + bw and bx < ax + aw
                       and ay < by + bh and by < ay + ah)
            check(not overlap, "按钮 %s 和 %s 重叠了", names[i], names[j])

    for name, (x, y, w, h) in ui.rects.items():
        check(x >= 0 and y >= 0 and x + w <= 552 and y + h <= 368,
              "按钮 %s 超出屏幕", name)

    top = min(y for (_, y, _, _) in ui.rects.values())
    check(top >= 60, "按钮上方至少要留 60px 画状态栏，实际 %d", top)
    check(ui.hit(2, 2) is None, "状态栏区域不该命中按钮")

    check(not ui.push_digit("1"), "第 1 位不该算完成")
    check(not ui.push_digit("3"), "第 2 位不该算完成")
    check(ui.push_digit("2"), "第 3 位应算完成")
    check(ui.code == "132", "任务码应为 132，得到 %s", ui.code)
    check(ui.code_done, "三位应标记为完成")
    ui.push_digit("1")
    check(ui.code == "132", "第 4 位应被忽略")
    check(not ui.push_digit("9"), "非法数字应返回 False")
    ui.clear()
    check(ui.code == "" and not ui.code_done, "清除失败")

    small = CAM.FieldUI(320, 240)
    for name, (x, y, w, h) in small.rects.items():
        check(x >= 0 and y >= 0 and x + w <= 320 and y + h <= 240,
              "小屏下按钮 %s 超出屏幕", name)
        check(small.hit(x + w // 2, y + h // 2) == name, "小屏下 %s 命中失败", name)


# ---------------------------------------------------------------- 配置

def test_config_persistence():
    section("配置持久化（现场标定要能掉电不丢）")
    import json
    import tempfile

    d = tempfile.mkdtemp()
    path = os.path.join(d, "cfg.json")

    ok = CAM.save_config(path, "mission", 30, None, None,
                         {"lock_ae": True, "mission": True, "ui": True}, None,
                         {"mission": True, "grab_point": [100, 140],
                          "drop_point": [138, 120], "grab_w_n": 95,
                          "drop_w_n": 120, "task_code": "312",
                          "hold_point": [40, 200]})
    check(ok, "保存配置失败")
    saved = CAM.load_config(path)
    check(saved.get("mission") is True, "mission 开关应被持久化")
    check(saved.get("grab_point") == [100, 140], "抓取动作点应被持久化")
    check(saved.get("drop_point") == [138, 120], "放置动作点应被持久化")
    check(saved.get("grab_w_n") == 95 and saved.get("drop_w_n") == 120,
          "门限应被持久化")
    check(saved.get("task_code") == "312", "任务码应被持久化")
    check(saved.get("hold_point") == [40, 200], "车上物料位置应被持久化")
    check(saved.get("ui") is True, "触摸界面开关应被持久化（否则开机进不去）")
    check("color" not in saved, "搬运模式不该覆盖之前保存的追踪色")

    with open(path, "w", encoding="utf-8") as f:
        json.dump({"mission": True, "grab_point": [999, -5], "drop_point": "x",
                   "grab_w_n": 300, "drop_w_n": "abc", "task_code": "124",
                   "hold_point": [1]}, f)
    bad = CAM.load_config(path)
    check("grab_point" not in bad, "越界的动作点必须被拒绝")
    check("drop_point" not in bad, "类型不对的动作点必须被拒绝")
    check("task_code" not in bad, "非法任务码必须被拒绝")
    check("hold_point" not in bad, "长度不对的位置必须被拒绝")

    check(CAM.load_config(os.path.join(d, "nope.json")) == {},
          "配置文件不存在时应返回空字典")

    # Windows 记事本/Set-Content 存出来的 UTF-8 带 BOM，必须能读
    bom_path = os.path.join(d, "bom.json")
    with open(bom_path, "wb") as f:
        f.write(b"\xef\xbb\xbf" + b'{"mission": true, "task_code": "213"}')
    got = CAM.load_config(bom_path)
    check(got.get("mission") is True and got.get("task_code") == "213",
          "带 BOM 的配置文件必须能读（记事本保存的 JSON 就带 BOM）: %s", got)


def test_thresholds_override():
    section("现场调阈值：配置文件覆盖（不用重新上传程序）")
    import json
    import tempfile

    d = tempfile.mkdtemp()
    path = os.path.join(d, "cfg.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"thresholds": {
            "red": [[30, 80, 45, 95, 10, 60]],
            "green": [[25, 85, -110, -20, -10, 70]],
            "yellow": [[0, 1, 2, 3, 4, 5]],
            "blue": [[25, 85, -30]],
        }}, f)
    got = CAM.load_config(path).get("thresholds")
    check(got is not None, "合法阈值应被读入")
    check(got.get("red") == [[30, 80, 45, 95, 10, 60]], "红色阈值读错")
    check(got.get("green") == [[25, 85, -110, -20, -10, 70]], "绿色阈值读错")
    check("yellow" not in got, "不存在的颜色必须被拒绝")
    check("blue" not in got, "长度不对的阈值必须被拒绝")


# ---------------------------------------------------------------- 上行帧

def test_frame_from_object():
    section("v3.0 上行帧")
    o = obj("green", 320, 240, 128, kind="ring")
    frame, info = CAM.frame_from_object(0x2A, o, CAM.STATE_DROP)
    check(len(frame) == 17, "帧长必须还是 17，得到 %d", len(frame))
    check(frame[4] == 0x2A and frame[5] == 1, "seq/exists 错误")
    check(frame[6] == P.COLOR_GREEN, "color 应为绿色")
    check(frame[7] == CAM.STATE_DROP, "state 字段应等于 STATE_DROP")
    check(frame[8] == 128 and frame[9] == 128, "坐标应是画面正中")
    check(frame[12] == sum(frame[4:12]) & 0xFF,
          "校验和错误（必须只覆盖偏移 4-11）")

    frame, info = CAM.frame_from_object(0x2B, None, CAM.STATE_DEST, "red")
    check(frame[5] == 0, "无目标时 exists 必须为 0")
    check(frame[8:12] == b"\x00\x00\x00\x00", "无目标时坐标必须全 0（不是 128）")
    check(frame[7] == CAM.STATE_DEST, "无目标时 state 仍应上报")
    check(frame[6] == P.COLOR_RED, "无目标时仍可上报正在等的颜色")
    check(info is None, "无目标时 info 应为 None")


def test_state_constants_match_pc():
    section("两端状态常量一致")
    for name in ("SEARCH", "MATERIAL", "GRAB", "DEST", "DROP", "DONE"):
        a = getattr(CAM, "STATE_" + name)
        b = getattr(P, "STATE_" + name)
        check(a == b, "STATE_%s 两端不一致：相机 %d / 上位机 %d", name, a, b)
    check(len(CAM.STATE_TEXT) == 6, "相机端状态文字表应覆盖 6 个状态")
    check(len(P.STATE_NAMES) == 6, "上位机状态文字表应覆盖 6 个状态")
    check(CAM.STATE_BLOCK == CAM.STATE_MATERIAL, "旧名 STATE_BLOCK 应兼容")
    check(CAM.STATE_RING == CAM.STATE_DEST, "旧名 STATE_RING 应兼容")


# ---------------------------------------------------------------- 主函数

def main():
    print("=" * 60)
    print("v3.0 搬运任务逻辑测试")
    print("=" * 60)
    test_hole_judgement()
    test_decide_shape()
    test_make_object()
    test_dedup()
    test_task_code()
    test_pick_largest()
    test_pick_nearest()
    test_aligned()
    test_mission_order()
    test_mission_one_cycle()
    test_mission_two_batches()
    test_mission_hysteresis()
    test_mission_abort()
    test_mission_hold_point()
    test_field_ui()
    test_config_persistence()
    test_thresholds_override()
    test_frame_from_object()
    test_state_constants_match_pc()

    print("\n" + "=" * 60)
    print("结果：通过 %d 项，失败 %d 项" % (_PASS, _FAIL))
    print("=" * 60)
    return 0 if _FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
