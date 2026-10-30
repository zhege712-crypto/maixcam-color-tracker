"""
给手机用的 HTTP 小服务（相机端）
================================

为什么不用 WebSocket / 第三方库：MaixPy 里只有 Python 标准库，
`http.server` 够用，手机浏览器轮询 8~10Hz 的开销可以忽略。
整个文件约 100 行，不引入任何依赖，换台设备也能跑。

它提供两个接口（手机页面 pc/mobile/index.html 会用）：

    GET  /api/latest    -> 最近一帧的 JSON
    POST /api/cmd       -> {"c": 1|2|3} 选色，{"c": 127} PING
    GET  /              -> 手机页面（默认从 /root/mobile.html 读）

两种跑法：

1) **跟着主程序跑**（推荐，现场用这个）
       在相机上执行： python3 /root/maixcam_color_tracker.py --net 8000
   主程序会把每帧的识别结果喂进来，手机连同一个网就能看。

2) **单独跑（本机调试用，没有相机也能验证手机页面）**
       python3 net_server.py --demo
   它自己造数据，用来确认手机浏览器能不能打开、布局对不对。

用法：
    python3 net_server.py --demo --port 8000
    python3 net_server.py --port 8000 --page /root/mobile.html
"""

import argparse
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

STATE = {
    "frame": {"ok": True, "exists": 0, "color": 0, "color_name": "?",
              "is_ring": False, "angle": 0.0,
              "cx": 0, "cy": 0, "w": 0, "h": 0, "fps": 0.0, "t": 0.0},
    "page": "/root/mobile.html",
    "on_cmd": None,          # 主程序注册：收到选色指令时调用
}
_LOCK = threading.Lock()


def set_frame(f):
    """主程序每帧调用一次，把识别结果交给手机端。"""
    with _LOCK:
        STATE["frame"] = f


def _read_page():
    path = STATE["page"]
    try:
        with open(path, "r", encoding="utf-8") as fh:
            return fh.read().encode("utf-8")
    except OSError as e:
        html = ("<h1>找不到手机页面</h1><p>%s</p>"
                "<p>把 pc/mobile/index.html 传到设备的 %s</p>"
                % (e, path))
        return html.encode("utf-8")


class Handler(BaseHTTPRequestHandler):
    # 默认的日志会把每一帧请求都打出来，串口日志会被刷爆
    def log_message(self, fmt, *args):
        pass

    def _send(self, code, body, ctype="application/json"):
        self.send_response(code)
        self.send_header("Content-Type", ctype + "; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.startswith("/api/latest"):
            with _LOCK:
                body = json.dumps(STATE["frame"]).encode("utf-8")
            self._send(200, body)
            return
        if self.path in ("/", "/index.html"):
            self._send(200, _read_page(), "text/html")
            return
        self._send(404, b'{"ok":false}', "application/json")

    def do_POST(self):
        if not self.path.startswith("/api/cmd"):
            self._send(404, b'{"ok":false}')
            return
        try:
            n = int(self.headers.get("Content-Length") or 0)
            data = json.loads(self.rfile.read(n) or b"{}")
            c = int(data.get("c", 0))
        except Exception:                                    # noqa: BLE001
            self._send(400, b'{"ok":false,"err":"bad json"}')
            return
        cb = STATE["on_cmd"]
        ok = False
        if cb is not None:
            try:
                ok = bool(cb(c))
            except Exception:                                # noqa: BLE001
                ok = False
        self._send(200, json.dumps({"ok": ok, "c": c}).encode("utf-8"))


def start(port=8000, page=None, on_cmd=None, host="0.0.0.0", quiet=False):
    """起一个后台线程跑 HTTP 服务，返回 server 对象（可 .shutdown()）。"""
    if page:
        STATE["page"] = page
    if on_cmd is not None:
        STATE["on_cmd"] = on_cmd
    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    if not quiet:
        print("[NET] 手机页面: http://<相机IP>:%d" % port)
    return srv


# ---------------------------------------------------------------- 演示模式

def _demo_loop(stop_after=None):
    """造一个在画面里绕圈的目标，偶尔换颜色、偶尔无目标。"""
    t0 = time.time()
    while True:
        t = time.time() - t0
        if stop_after and t > stop_after:
            break
        colors = [(1, "red"), (2, "green"), (3, "blue")]
        color, cname = colors[int(t / 8) % 3]
        ang = t * 1.2
        exists = 1
        if int(t) % 9 == 8:          # 每 9 秒里有 1 秒"没目标"
            exists = 0
        cx = 320 + 210 * (0.5 + 0.5 * (t % 8) / 8.0) * (1 if int(t / 8) % 2 else -1)
        cy = 240 + 130 * __import__("math").sin(ang)
        w = 60 + 30 * (0.5 + 0.5 * __import__("math").sin(t))
        f = {"ok": True, "exists": exists, "color": color, "color_name": cname,
             "is_ring": bool(int(t / 3) % 2), "angle": (t * 37) % 360,
             "cx": int(cx), "cy": int(cy), "w": int(w), "h": int(w * 0.8),
             "fps": 15.0, "t": time.time()}
        if not exists:
            f.update({"cx": 0, "cy": 0, "w": 0, "h": 0})
        set_frame(f)
        time.sleep(0.07)


def main():
    ap = argparse.ArgumentParser(description="相机端 HTTP 小服务（给手机用）")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--page", default=None, help="手机页面文件路径")
    ap.add_argument("--demo", action="store_true", help="自己造数据，不需要相机")
    ap.add_argument("--duration", type=float, default=0)
    args = ap.parse_args()

    if args.page is None:
        here = os.path.dirname(os.path.abspath(__file__))
        args.page = os.path.join(os.path.dirname(here), "pc", "mobile", "index.html")
        if not os.path.exists(args.page):
            args.page = "/root/mobile.html"

    start(args.port, args.page, on_cmd=lambda c: print("[CMD] %s" % c))
    if args.demo:
        _demo_loop(args.duration or None)
    else:
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
