# -*- coding: utf-8 -*-
"""仅订阅绿灯像素真值：python vision.py；纯JSON输出：python vision.py --console。"""
import argparse
import json
import time

from bridge_ipc import SensorSubscriber, TOPIC_VISION_GT

BG = "#111417"
FG = "#e6eef7"
MUTED = "#9aa7b4"
REASONS = {
    "visible": "中心可见", "offscreen": "中心在视野外", "behind_camera": "位于相机后方",
    "clipped": "超出近/远裁剪范围", "occluded": "中心视线被遮挡", "ray_miss": "中心射线未命中，状态未知",
}


def console():
    """标准输出每行一个完整JSON；便于重定向或作为算法输入示例。"""
    with SensorSubscriber(topics=(TOPIC_VISION_GT,)) as subscriber:
        try:
            while True:
                event = subscriber.recv(timeout=0.2)
                if event is not None:
                    print(json.dumps(event, ensure_ascii=False, allow_nan=False), flush=True)
        except KeyboardInterrupt:
            pass


def show_panel():
    import tkinter as tk
    root = tk.Tk()
    root.title("RoboMaster · 绿灯像素真值")
    root.geometry("560x420")
    root.minsize(500, 380)
    root.configure(bg=BG)
    status = tk.StringVar(value="等待 dart/vision_gt ...")
    rate = tk.StringVar(value="接收频率：-- Hz")
    tk.Label(root, text="视觉真值 · 绿灯中心像素坐标", bg=BG, fg=FG,
             font=("Microsoft YaHei UI", 16, "bold"), anchor="w",
             padx=18, pady=12).pack(fill="x")
    tk.Label(root, textvariable=status, bg=BG, fg=MUTED, anchor="w",
             font=("Consolas", 10), padx=18).pack(fill="x")
    cards = tk.Frame(root, bg=BG)
    cards.pack(fill="both", expand=True, padx=12, pady=12)
    values = {}
    for col, (key, title) in enumerate((("base", "对面基地绿灯"),)):
        cards.grid_columnconfigure(col, weight=1, uniform="cards")
        card = tk.Frame(cards, bg="#1b222a", highlightthickness=1, highlightbackground="#303b46")
        card.grid(row=0, column=col, sticky="nsew", padx=6)
        card.grid_columnconfigure(0, weight=1)
        card.grid_columnconfigure(1, weight=1)
        tk.Label(card, text=title, bg="#1b222a", fg=FG,
                 font=("Microsoft YaHei UI", 13, "bold")).grid(row=0, column=0,
                 columnspan=2, sticky="w", padx=14, pady=(12, 6))
        values[key] = {name: tk.StringVar(value="--") for name in
                       ("u", "v", "in_frame", "occluded", "visible", "reason")}
        for i, axis in enumerate(("u", "v")):
            tk.Label(card, text=f"{axis} / pixel", bg="#1b222a", fg=MUTED,
                     font=("Consolas", 11)).grid(row=1, column=i, sticky="w", padx=14)
            tk.Label(card, textvariable=values[key][axis], bg="#1b222a", fg="#35d879",
                     font=("Consolas", 27, "bold")).grid(row=2, column=i,
                     sticky="w", padx=14, pady=(4, 10))
        for row, (name, label) in enumerate((("in_frame", "中心在画面内"),
                    ("occluded", "中心被遮挡"), ("visible", "坐标有效/中心可见")), start=3):
            tk.Label(card, text=label, bg="#1b222a", fg=MUTED,
                     font=("Microsoft YaHei UI", 10)).grid(row=row, column=0,
                     sticky="w", padx=14, pady=4)
            tk.Label(card, textvariable=values[key][name], bg="#1b222a", fg=FG,
                     font=("Consolas", 11)).grid(row=row, column=1, sticky="w", padx=14)
        tk.Label(card, textvariable=values[key]["reason"], bg="#1b222a", fg=MUTED,
                 font=("Microsoft YaHei UI", 10)).grid(row=6, column=0, columnspan=2,
                 sticky="w", padx=14, pady=(8, 12))
    tk.Label(root, text="左上像素中心为 (0, 0)，u 向右、v 向下；只输出对面基地绿灯中心。遮挡判断针对灯面中心。",
             bg=BG, fg=MUTED, anchor="w", padx=18, pady=4,
             font=("Microsoft YaHei UI", 9)).pack(fill="x")
    tk.Label(root, textvariable=rate, bg=BG, fg=MUTED, anchor="e", padx=18,
             pady=8, font=("Consolas", 10)).pack(fill="x")
    alive = True

    def close():
        nonlocal alive
        alive = False
        root.destroy()

    def clear():
        for fields in values.values():
            for variable in fields.values():
                variable.set("--")

    root.protocol("WM_DELETE_WINDOW", close)
    count, last_rate, last_event = 0, time.monotonic(), time.monotonic()
    stale = False
    with SensorSubscriber(topics=(TOPIC_VISION_GT,)) as subscriber:
        try:
            while alive:
                try:
                    root.update()
                except tk.TclError:
                    break
                if not alive:
                    break
                event = subscriber.recv(timeout=0.02)
                now = time.monotonic()
                if event is not None:
                    data = event["data"]
                    count += 1
                    last_event, stale = now, False
                    if not data.get("valid", False):
                        clear()
                        status.set("视觉真值无效：" + data.get("error", "未知错误"))
                    else:
                        paired = "同周期已渲染图像" if data.get("image_available") else "仅数字输出/本帧无图像"
                        status.set(f"{data['width']}×{data['height']} · t={event['time_s']:.4f}s"
                                   f" · 帧 #{event['frame_id']} · {paired}")
                        for key, fields in values.items():
                            target = data.get("targets", {}).get(key, {})
                            for axis in ("u", "v"):
                                value = target.get(axis)
                                fields[axis].set("--" if value is None else f"{value:.3f}")
                            for name in ("in_frame", "occluded", "visible"):
                                value = target.get(name)
                                fields[name].set("--" if value is None else str(bool(value)))
                            reason = target.get("reason", "")
                            fields["reason"].set(REASONS.get(reason, reason or "--"))
                if now - last_event > 1.5 and not stale:
                    clear()
                    status.set("超过 1.5 秒未收到新数据；请检查仿真程序与接口。")
                    stale = True
                if now - last_rate >= 1.0:
                    rate.set(f"接收频率：{count / (now - last_rate):.1f} Hz")
                    count, last_rate = 0, now
        except KeyboardInterrupt:
            pass
        finally:
            try:
                root.destroy()
            except tk.TclError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--console", action="store_true", help="不打开窗口，每行打印一个JSON数据包")
    args = parser.parse_args()
    console() if args.console else show_panel()


if __name__ == "__main__":
    main()
