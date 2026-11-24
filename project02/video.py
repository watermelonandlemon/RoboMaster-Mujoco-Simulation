# -*- coding: utf-8 -*-
"""FPV 视频；可选 --show-gt 叠加同帧绿灯真值，不改变原始 RGB 输出。"""
import argparse
from collections import OrderedDict
import math
import base64
import binascii
import struct
import time
import tkinter as tk
import zlib

from bridge_ipc import SensorSubscriber, TOPIC_IMAGE, TOPIC_VISION_GT


def rgb_to_png_base64(rgb_bytes, width, height):
    """将 RGB24 编码为 Tkinter 可显示的 PNG Base64。"""
    expected = width * height * 3
    if len(rgb_bytes) != expected:
        raise ValueError(f"RGB 数据长度错误：期望 {expected}，实际 {len(rgb_bytes)}")
    stride = width * 3
    raw = b"".join(
        b"\x00" + rgb_bytes[row * stride:(row + 1) * stride]
        for row in range(height)
    )

    def chunk(kind, data):
        body = kind + data
        return (struct.pack(">I", len(data)) + body +
                struct.pack(">I", binascii.crc32(body) & 0xffffffff))

    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, level=1))
        + chunk(b"IEND", b"")
    )
    return base64.b64encode(png).decode("ascii")


class FrameMatchedVision:
    """UDP允许乱序/丢包：只使用(frame_id,time_s)完全相同的真值。"""
    def __init__(self, max_entries=32):
        self.max_entries = max(1, int(max_entries))
        self.cache = OrderedDict()
        self.image_key = None

    @staticmethod
    def key(event):
        return int(event["frame_id"]), float(event["time_s"])

    def set_image(self, event):
        self.image_key = self.key(event)
        return self.current()

    def add_truth(self, event):
        key = self.key(event)
        self.cache[key] = event["data"]
        self.cache.move_to_end(key)
        while len(self.cache) > self.max_entries:
            self.cache.popitem(last=False)
        return self.current()

    def current(self):
        return self.cache.get(self.image_key)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--show-gt", action="store_true", help="订阅第四话题，在视频上叠加同帧中心十字")
    args = parser.parse_args()
    root = tk.Tk()
    root.title("RoboMaster · 飞镖视频订阅")
    root.configure(bg="#111417")
    root.geometry("700x600")
    status = tk.StringVar(value="等待 MuJoCo 视频输出...")
    rate_text = tk.StringVar(value="视频：-- FPS")
    gt_status = tk.StringVar(value="")
    tk.Label(root, textvariable=status, anchor="w", bg="#111417",
             fg="#e6eef7", font=("Microsoft YaHei UI", 10),
             padx=12, pady=8).pack(fill="x")
    tk.Label(root, textvariable=rate_text, anchor="w", bg="#111417",
             fg="#9aa7b4", font=("Consolas", 10),
             padx=12, pady=4).pack(fill="x")
    canvas = tk.Canvas(root, width=640, height=480, bg="#202831", highlightthickness=0)
    canvas.pack(expand=True, padx=10, pady=10)
    image_item = canvas.create_image(0, 0, anchor="nw")
    if args.show_gt:
        tk.Label(root, textvariable=gt_status, anchor="w", bg="#111417", fg="#9aa7b4",
                 font=("Microsoft YaHei UI", 9), padx=12, pady=4).pack(fill="x")
    topics = (TOPIC_IMAGE, TOPIC_VISION_GT) if args.show_gt else (TOPIC_IMAGE,)
    subscriber = SensorSubscriber(topics=topics)
    print(f"[UDP订阅] {', '.join(topics)} <- 127.0.0.1:18980")
    matched = FrameMatchedVision()
    alive, count = True, 0
    last_rate = time.monotonic()

    def close():
        nonlocal alive
        alive = False
        root.destroy()

    def draw_truth():
        canvas.delete("gt")
        if not args.show_gt:
            return
        data = matched.current()
        if data is None:
            gt_status.set("本图尚无同帧真值；不使用其他帧坐标代替。")
            return
        if not data.get("valid", False) or not data.get("image_available", False):
            gt_status.set("本帧真值无效/无匹配图像：" + data.get("error", ""))
            return
        visible = []
        for key, target in data.get("targets", {}).items():
            if not target.get("visible"):
                continue
            u, v = target.get("u"), target.get("v")
            if u is None or v is None or not math.isfinite(u) or not math.isfinite(v):
                continue
            # 像素中心坐标(0,0)在Canvas图像左上边缘内0.5px处。
            x, y = u + 0.5, v + 0.5
            color = "#3dff7d"
            canvas.create_line(x-7, y, x+7, y, fill=color, width=1, tags="gt")
            canvas.create_line(x, y-7, x, y+7, fill=color, width=1, tags="gt")
            canvas.create_text(x+9, y-9, anchor="w", text=f"base ({u:.2f}, {v:.2f})",
                               fill=color, font=("Consolas", 10), tags="gt")
            visible.append(key)
        gt_status.set("已匹配同帧：" + (", ".join(visible) if visible else "基地绿灯中心不可见"))

    root.protocol("WM_DELETE_WINDOW", close)
    try:
        while alive:
            try:
                root.update()
            except tk.TclError:
                break
            if not alive:
                break
            event = subscriber.recv(timeout=0.02)
            if event is not None:
                if event["topic"] == TOPIC_IMAGE:
                    width, height = event["width"], event["height"]
                    photo = tk.PhotoImage(master=root, data=rgb_to_png_base64(
                        event["image_rgb"], width, height), format="png")
                    canvas.configure(width=width, height=height)
                    canvas.itemconfigure(image_item, image=photo)
                    canvas.image = photo
                    matched.set_image(event)
                    status.set(f"视频已连接 · {width}×{height} · 仿真时间 {event['time_s']:.4f}s"
                               f" · 帧 #{event['frame_id']}")
                    count += 1
                    draw_truth()
                elif event["topic"] == TOPIC_VISION_GT:
                    matched.add_truth(event)
                    draw_truth()
            now = time.monotonic()
            if now - last_rate >= 1.0:
                rate_text.set(f"视频：{count / (now - last_rate):.1f} FPS")
                count, last_rate = 0, now
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()
        try:
            root.destroy()
        except tk.TclError:
            pass


if __name__ == "__main__":
    main()
