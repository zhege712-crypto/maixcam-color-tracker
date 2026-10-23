"""
虚拟相机
========

在电脑上模拟 MaixPy 的 `image` 对象，用来把**整条视觉链路**（找色块 → 判断
环/块 → 选目标 → 组帧）在没有硬件的情况下跑通。

为什么值得做：环和块只差"中间空不空"，这个逻辑用真机调一次要一分钟（插拔、
重启、对准），在电脑上是毫秒级，而且可以精确构造"环被压住一半""两个实心块"
这种真机上很难复现的场景。

只实现程序真正用到的那几个接口：
    img.find_blobs(thresholds, ...)  -> 列表，元素支持 [0..3] 和 .pixels()
    img.get_statistics(roi=(x,y,w,h)) -> 有 a_mean()/b_mean()/l_mean() 等

图像内容是**直接在 LAB 空间**给的（L/A/B 三个数组），省掉 RGB→LAB 换算，
因为要测的是判定逻辑，不是色彩空间转换本身。
"""

import numpy as np


class FakeStats:
    def __init__(self, l, a, b):
        self._l, self._a, self._b = l, a, b

    def l_mean(self):
        return float(np.mean(self._l)) if self._l.size else 0.0

    def a_mean(self):
        return float(np.mean(self._a)) if self._a.size else 0.0

    def b_mean(self):
        return float(np.mean(self._b)) if self._b.size else 0.0

    def l_min(self):
        return int(np.min(self._l)) if self._l.size else 0

    def l_max(self):
        return int(np.max(self._l)) if self._l.size else 0

    def a_min(self):
        return int(np.min(self._a)) if self._a.size else 0

    def a_max(self):
        return int(np.max(self._a)) if self._a.size else 0

    def b_min(self):
        return int(np.min(self._b)) if self._b.size else 0

    def b_max(self):
        return int(np.max(self._b)) if self._b.size else 0

    def pixel_count(self):
        return int(self._l.size)

    def l_std_dev(self):
        return float(np.std(self._l)) if self._l.size else 0.0

    def a_std_dev(self):
        return float(np.std(self._a)) if self._a.size else 0.0

    def b_std_dev(self):
        return float(np.std(self._b)) if self._b.size else 0.0


class FakeBlob:
    """和 MaixPy 的 blob 一样支持 blob[0..3]，另外提供 .pixels()。"""

    def __init__(self, rect, pixels):
        self._rect = rect
        self._pixels = pixels

    def __getitem__(self, i):
        return self._rect[i]

    def __len__(self):
        return 4

    def rect(self):
        return self._rect

    def pixels(self):
        return self._pixels

    def count(self):
        return self._pixels

    def density(self):
        """和 MaixPy 一样：像素数 / 框面积。"""
        w, h = self._rect[2], self._rect[3]
        return self._pixels / float(w * h) if w > 0 and h > 0 else 0.0

    def extent(self):
        return self.density()

    def x(self):
        return self._rect[0]

    def y(self):
        return self._rect[1]

    def w(self):
        return self._rect[2]

    def h(self):
        return self._rect[3]


class FakeImage:
    def __init__(self, L, A, B):
        self.L = np.asarray(L, dtype=np.int16)
        self.A = np.asarray(A, dtype=np.int16)
        self.B = np.asarray(B, dtype=np.int16)
        self.h, self.w = self.L.shape
        self.stats_calls = 0      # get_statistics 被调用了多少次（用来验证"能不算就不算"）

    # ---- 程序用到的两个接口 ----

    def find_blobs(self, thresholds, pixels_threshold=150, area_threshold=150,
                   roi=None, x_stride=1, y_stride=1, merge=False, margin=0):
        """多段阈值取并集，做连通域标记。

        注意：这里**故意忽略 x_stride/x_stride**（模拟器不做跳点采样），
        因为要验的是判定逻辑；真实设备上的跳点采样只影响速度。
        """
        mask = np.zeros((self.h, self.w), dtype=bool)
        for t in thresholds:
            l0, l1, a0, a1, b0, b1 = t
            mask |= ((self.L >= l0) & (self.L <= l1)
                     & (self.A >= a0) & (self.A <= a1)
                     & (self.B >= b0) & (self.B <= b1))

        if roi:
            rx, ry, rw, rh = [int(v) for v in roi[:4]]
            sub = np.zeros_like(mask)
            x0, y0 = max(0, rx), max(0, ry)
            x1, y1 = min(self.w, rx + rw), min(self.h, ry + rh)
            if x1 > x0 and y1 > y0:
                sub[y0:y1, x0:x1] = mask[y0:y1, x0:x1]
            mask = sub

        return self._components(mask, max(pixels_threshold, area_threshold))

    def get_pixel(self, x, y):
        """和 MaixPy 一样返回 [打包成整数的 RGB]。

        这里没有真的 RGB，就用 (L, A+128, B+128) 当伪 RGB ——
        形状判定只关心"两个颜色是不是同一种"，LAB 差值和 RGB 差值
        在语义上等价，够用来验证逻辑。
        """
        x, y = int(x), int(y)
        if not (0 <= x < self.w and 0 <= y < self.h):
            return [0]
        r = int(np.clip(self.L[y, x], 0, 255))
        g = int(np.clip(self.A[y, x] + 128, 0, 255))
        b = int(np.clip(self.B[y, x] + 128, 0, 255))
        return [(r << 16) | (g << 8) | b]

    def get_statistics(self, roi=None, thresholds=None):
        self.stats_calls += 1
        if roi:
            x, y, w, h = [int(v) for v in roi[:4]]
            x0, y0 = max(0, x), max(0, y)
            x1, y1 = min(self.w, x + w), min(self.h, y + h)
            if x1 <= x0 or y1 <= y0:          # 越界时给一个退化区域
                x0, y0, x1, y1 = 0, 0, 1, 1
        else:
            x0, y0, x1, y1 = 0, 0, self.w, self.h

        L = self.L[y0:y1, x0:x1]
        A = self.A[y0:y1, x0:x1]
        B = self.B[y0:y1, x0:x1]

        if thresholds:
            # 和真机一样：给了 thresholds 就**只统计落在阈值内的像素**。
            # 这条语义很关键 —— learn_from_center 靠它把背景排除在外。
            mask = np.zeros(L.shape, dtype=bool)
            for t in thresholds:
                l0, l1, a0, a1, b0, b1 = t
                mask |= ((L >= l0) & (L <= l1)
                         & (A >= a0) & (A <= a1)
                         & (B >= b0) & (B <= b1))
            if not mask.any():
                empty = np.zeros(0, np.int16)
                return FakeStats(empty, empty, empty)
            L, A, B = L[mask], A[mask], B[mask]

        return FakeStats(L.ravel(), A.ravel(), B.ravel())

    # ---- 内部：连通域 ----

    def _components(self, mask, min_pixels):
        seen = np.zeros_like(mask)
        out = []
        h, w = mask.shape
        for sy in range(h):
            row = mask[sy]
            for sx in range(w):
                if not row[sx] or seen[sy, sx]:
                    continue
                # 广度优先，用列表当栈
                stack = [(sy, sx)]
                seen[sy, sx] = True
                x0 = x1 = sx
                y0 = y1 = sy
                n = 0
                while stack:
                    cy, cx = stack.pop()
                    n += 1
                    if cx < x0:
                        x0 = cx
                    elif cx > x1:
                        x1 = cx
                    if cy < y0:
                        y0 = cy
                    elif cy > y1:
                        y1 = cy
                    for ny, nx in ((cy - 1, cx), (cy + 1, cx),
                                   (cy, cx - 1), (cy, cx + 1)):
                        if 0 <= ny < h and 0 <= nx < w \
                                and mask[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            stack.append((ny, nx))
                if n >= min_pixels:
                    out.append(FakeBlob((x0, y0, x1 - x0 + 1, y1 - y0 + 1), n))
        return out


# ---------------------------------------------------------------- 画场景

BG = (210, 0, 0)          # 浅灰白地面：高亮度、零色度


def blank(w=320, h=240, bg=BG):
    L = np.full((h, w), bg[0], dtype=np.int16)
    A = np.full((h, w), bg[1], dtype=np.int16)
    B = np.full((h, w), bg[2], dtype=np.int16)
    return L, A, B


def draw_disc(L, A, B, cx, cy, r, lab):
    yy, xx = np.ogrid[:L.shape[0], :L.shape[1]]
    m = (xx - cx) ** 2 + (yy - cy) ** 2 <= r * r
    L[m], A[m], B[m] = lab
    return L, A, B


def draw_ring(L, A, B, cx, cy, r_out, r_in, lab):
    yy, xx = np.ogrid[:L.shape[0], :L.shape[1]]
    d2 = (xx - cx) ** 2 + (yy - cy) ** 2
    m = (d2 <= r_out * r_out) & (d2 >= r_in * r_in)
    L[m], A[m], B[m] = lab
    return L, A, B


def draw_rect(L, A, B, x, y, w, h, lab):
    L[y:y + h, x:x + w], A[y:y + h, x:x + w], B[y:y + h, x:x + w] = lab
    return L, A, B


def lab_red():
    return (120, 62, 28)


def lab_green():
    return (140, -55, 25)


def lab_blue():
    return (110, -20, -58)
