#!/usr/bin/env python3
"""
显示 / 触摸坐标探针
===================

做现场触摸界面之前必须确认两件事：

    1. 屏幕的分辨率是多少（和摄像头 640x480 不一样）
    2. 把画面按屏幕尺寸缩放到屏幕上显示后，触摸返回的坐标是不是
       就是屏幕坐标（这样才能 1:1 命中按钮，不用做坐标换算）

用法：
    python3 probe_ui.py            # 画 5 秒测试图案，期间可以点屏幕
"""

import sys


def main():
    from maix import camera, display, image, time, touchscreen  # noqa: F401

    print("=" * 56)
    print("显示 / 触摸坐标探针")
    print("=" * 56)

    disp = display.Display()
    for name in ("width", "height"):
        f = getattr(disp, name, None)
        print("[INFO] disp.%s() = %s" % (name, f() if callable(f) else f))

    ts = touchscreen.TouchScreen()

    cam = camera.Camera(640, 480)
    img = cam.read()
    print("[INFO] 摄像头帧：%dx%d" % (img.width(), img.height()))

    # 关键：把画面缩放到屏幕尺寸
    dw = disp.width() if callable(getattr(disp, "width", None)) else 552
    dh = disp.height() if callable(getattr(disp, "height", None)) else 368
    print("[INFO] 目标画布：%dx%d" % (dw, dh))

    canvas = None
    try:
        canvas = img.resize(dw, dh)
        print("[INFO] img.resize(%d,%d) 成功 -> %dx%d"
              % (dw, dh, canvas.width(), canvas.height()))
    except Exception as e:                        # noqa: BLE001
        print("[FAIL] resize 失败：%s" % e)
        return 1

    # 画几个定位用的方块，看它们在屏幕上是否对得上
    canvas.draw_rect(0, 0, dw // 4, dh // 4, image.COLOR_RED, 2)
    canvas.draw_rect(dw - dw // 4, 0, dw // 4, dh // 4, image.COLOR_GREEN, 2)
    canvas.draw_rect(0, dh - dh // 4, dw // 4, dh // 4, image.COLOR_BLUE, 2)
    canvas.draw_rect(dw - dw // 4, dh - dh // 4, dw // 4, dh // 4,
                     image.COLOR_YELLOW, 2)
    canvas.draw_string(10, dh // 2, "TOUCH TEST %dx%d" % (dw, dh),
                       image.COLOR_WHITE, 2)
    disp.show(canvas)

    print("[INFO] 屏幕上应该是：左上红 右上绿 左下蓝 右下黄")
    print("[INFO] 请在 5 秒内依次点这四个角，观察下面的坐标")
    print("[INFO] 如果点左上角却打出接近 (dw,0) 的坐标，说明坐标系是反的/有偏移")

    t0 = time.time()
    last = None
    while time.time() - t0 < 5.0:
        d = ts.read()
        if d and d[2]:                            # pressed
            if last != (d[0], d[1]):
                print("[TOUCH] x=%4d y=%4d   (屏幕 %dx%d)" % (d[0], d[1], dw, dh))
            last = (d[0], d[1])
        else:
            last = None
        time.sleep_ms(30)

    print("[INFO] 结束")
    return 0


if __name__ == "__main__":
    sys.exit(main())
