# -*- coding: utf-8 -*-
"""仅订阅 IMU 数字数据：python IMU.py。"""
import time
import tkinter as tk

from simulator_link import SensorSubscriber, TOPIC_IMU


BG = "#111417"
FG = "#e6eef7"
MUTED = "#9aa7b4"
ACCENT = "#78b8e8"


def main():
    root = tk.Tk()
    root.title("RoboMaster · 飞镖 IMU")
    root.geometry("560x390")
    root.configure(bg=BG)

    time_var = tk.StringVar(value="t = -- s")
    valid_var = tk.StringVar(value="IMU 状态：等待数据")
    values = {
        "ax": tk.StringVar(value="--"), "ay": tk.StringVar(value="--"),
        "az": tk.StringVar(value="--"), "wx": tk.StringVar(value="--"),
        "wy": tk.StringVar(value="--"), "wz": tk.StringVar(value="--"),
    }
    rate_var = tk.StringVar(value="接收频率：-- Hz")

    tk.Label(root, text="IMU 实时监测", bg=BG, fg=FG,
             font=("Microsoft YaHei UI", 17, "bold"),
             anchor="w", padx=20, pady=15).pack(fill="x")
    tk.Label(root, textvariable=time_var, bg=BG, fg=FG,
             font=("Consolas", 14), anchor="e", padx=20).pack(fill="x")

    panel = tk.Frame(root, bg="#1b222a", highlightthickness=1,
                     highlightbackground="#303b46")
    panel.pack(fill="both", expand=True, padx=16, pady=14)
    panel.grid_columnconfigure(1, weight=1)
    panel.grid_columnconfigure(3, weight=1)

    def section(row, col, title, unit, keys):
        tk.Label(panel, text=title, bg="#1b222a", fg=FG,
                 font=("Microsoft YaHei UI", 12, "bold")).grid(
                     row=row, column=col, columnspan=2, sticky="w",
                     padx=16, pady=(18, 10))
        tk.Label(panel, text=unit, bg="#1b222a", fg=MUTED,
                 font=("Microsoft YaHei UI", 9)).grid(
                     row=row, column=col+1, sticky="e", padx=16, pady=(18, 10))
        for i, (key, label) in enumerate(keys, start=1):
            tk.Label(panel, text=label, bg="#1b222a", fg=MUTED,
                     font=("Microsoft YaHei UI", 11)).grid(
                         row=row+i, column=col, sticky="w", padx=18, pady=9)
            tk.Label(panel, textvariable=values[key], bg="#1b222a", fg=FG,
                     font=("Consolas", 14), anchor="e").grid(
                         row=row+i, column=col+1, sticky="ew", padx=18, pady=9)

    section(0, 0, "线加速度", "m/s²",
            (("ax", "Ax"), ("ay", "Ay"), ("az", "Az")))
    section(0, 2, "角速度", "rad/s",
            (("wx", "ωx"), ("wy", "ωy"), ("wz", "ωz")))

    footer = tk.Frame(root, bg=BG)
    footer.pack(fill="x", padx=18, pady=(0, 12))
    tk.Label(footer, textvariable=valid_var, bg=BG, fg=ACCENT,
             font=("Microsoft YaHei UI", 10)).pack(side="left")
    tk.Label(footer, textvariable=rate_var, bg=BG, fg=MUTED,
             font=("Consolas", 10)).pack(side="right")

    subscriber = SensorSubscriber(topics=(TOPIC_IMU,))
    print(f"[UDP订阅] 仅订阅 {TOPIC_IMU} <- 127.0.0.1:18980")
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
                data = event.get("data", {})
                time_var.set(f"t = {float(event['time_s']):.4f} s   /   帧 #{event['frame_id']}")
                acc = data.get("acceleration_body_mps2", [None] * 3)
                gyro = data.get("angular_velocity_body_rad_s", [None] * 3)
                for key, val in zip(("ax", "ay", "az"), acc):
                    values[key].set("--" if val is None else f"{float(val):+.4f}")
                for key, val in zip(("wx", "wy", "wz"), gyro):
                    values[key].set("--" if val is None else f"{float(val):+.4f}")
                valid_var.set(
                    "IMU 状态：有效" if data.get("acceleration_valid")
                    else "IMU 状态：初始化 / 加速度暂不可用"
                )
                count += 1
            now = time.monotonic()
            if now - last_rate >= 1.0:
                rate_var.set(f"接收频率：{count / (now-last_rate):.1f} Hz")
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
