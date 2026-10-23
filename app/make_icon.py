"""生成 color_tracker 应用的菜单图标（离屏生成，不需要设备）。"""

import os

from PIL import Image, ImageDraw

W = H = 128
BG = (24, 26, 32)
FRAME = (92, 98, 112)
GUIDE = (66, 72, 86)
BLOCK = (226, 62, 62)
DETECT = (54, 214, 128)

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)

# 画面矩形
d.rectangle([14, 22, 114, 106], outline=FRAME, width=3)
# 中心参考线（对应程序里画的画面中心十字）
d.line([64, 28, 64, 100], fill=GUIDE, width=2)
d.line([20, 64, 108, 64], fill=GUIDE, width=2)
# 目标色块
d.rectangle([76, 42, 102, 68], fill=BLOCK)
# 检测框
d.rectangle([71, 37, 107, 73], outline=DETECT, width=3)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                   "color_tracker", "app.png")
os.makedirs(os.path.dirname(out), exist_ok=True)
img.save(out)
print("written:", out, os.path.getsize(out), "bytes")
