# -*- coding: utf-8 -*-
"""仅订阅飞镖 FPV 视频：python video.py"""
import base64
import binascii
import struct
import time
import tkinter as tk
import zlib

from bridge_ipc import SensorSubscriber, TOPIC_IMAGE


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


def main():
    root = tk.Tk()
    root.title("RoboMaster · 飞镖视频订阅")
    root.configure(bg="#111417")
    root.geometry("700x570")
    status = tk.StringVar(value="等待 MuJoCo 视频输出...")
    rate_text = tk.StringVar(value="视频：-- FPS")
    tk.Label(root, textvariable=status, anchor="w", bg="#111417",
             fg="#e6eef7", font=("Microsoft YaHei UI", 10),
             padx=12, pady=8).pack(fill="x")
    tk.Label(root, textvariable=rate_text, anchor="w", bg="#111417",
             fg="#9aa7b4", font=("Consolas", 10),
             padx=12, pady=4).pack(fill="x")
    image_label = tk.Label(root, text="等待图像", bg="#202831", fg="#a8b7c6")
    image_label.pack(expand=True, padx=10, pady=10)

    subscriber = SensorSubscriber(topics=(TOPIC_IMAGE,))
    print(f"[UDP订阅] 仅订阅 {TOPIC_IMAGE} <- 127.0.0.1:18980")
    alive = True
    count = 0
    last_rate = time.monotonic()

    def close():
        nonlocal alive
        alive = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    try:
        while alive:
            try:
                root.update()
            except tk.TclError:
                break
            event = subscriber.recv(timeout=0.02)
            if event is not None:
                width, height = event["width"], event["height"]
                png_data = rgb_to_png_base64(event["image_rgb"], width, height)
                photo = tk.PhotoImage(master=root, data=png_data, format="png")
                image_label.configure(image=photo, text="")
                image_label.image = photo
                status.set(
                    f"视频已连接 · {width}×{height} · 仿真时间 {event['time_s']:.4f} s"
                )
                count += 1
            now = time.monotonic()
            if now - last_rate >= 1.0:
                rate_text.set(f"视频：{count / (now - last_rate):.1f} FPS")
                count = 0
                last_rate = now
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
