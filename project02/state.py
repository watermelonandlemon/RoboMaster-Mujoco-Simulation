# -*- coding: utf-8 -*-
"""仅订阅 MuJoCo 完整状态真值（上帝视角）：python state.py。"""
import time
import tkinter as tk

from simulator_link import SensorSubscriber, TOPIC_STATE


BG = "#111417"
FG = "#e6eef7"
MUTED = "#9aa7b4"


def main():
    root = tk.Tk()
    root.title("RoboMaster · 飞镖 Full State（上帝视角）")
    root.geometry("760x690")
    root.configure(bg=BG)

    time_var = tk.StringVar(value="t = -- s")
    rate_var = tk.StringVar(value="接收频率：-- Hz")
    fields = {
        key: tk.StringVar(value="--") for key in (
            "px", "py", "pz", "vx", "vy", "vz",
            "yaw", "pitch", "roll", "qw", "qx", "qy", "qz",
            "wx", "wy", "wz", "active", "passive", "total"
        )
    }

    tk.Label(root, text="飞镖实时状态 · MuJoCo Ground Truth",
             bg=BG, fg=FG, font=("Microsoft YaHei UI", 16, "bold"),
             anchor="w", padx=20, pady=14).pack(fill="x")
    tk.Label(root, textvariable=time_var, bg=BG, fg=FG,
             font=("Consolas", 13), anchor="e", padx=20).pack(fill="x")

    panel = tk.Frame(root, bg="#1b222a", highlightthickness=1,
                     highlightbackground="#303b46")
    panel.pack(fill="both", expand=True, padx=16, pady=12)
    panel.grid_columnconfigure(1, weight=1)
    panel.grid_columnconfigure(3, weight=1)

    def add_group(row, col, title, unit, items):
        tk.Label(panel, text=title, bg="#1b222a", fg=FG,
                 font=("Microsoft YaHei UI", 11, "bold")).grid(
                     row=row, column=col, columnspan=2, sticky="w",
                     padx=14, pady=(13, 5))
        tk.Label(panel, text=unit, bg="#1b222a", fg=MUTED,
                 font=("Microsoft YaHei UI", 9)).grid(
                     row=row, column=col+1, sticky="e", padx=14, pady=(13, 5))
        for i, (key, label) in enumerate(items, start=1):
            tk.Label(panel, text=label, bg="#1b222a", fg=MUTED,
                     font=("Microsoft YaHei UI", 10)).grid(
                         row=row+i, column=col, sticky="w", padx=16, pady=5)
            tk.Label(panel, textvariable=fields[key], bg="#1b222a", fg=FG,
                     font=("Consolas", 12), anchor="e").grid(
                         row=row+i, column=col+1, sticky="ew", padx=16, pady=5)

    add_group(0, 0, "质心位置", "m",
              (("px", "X"), ("py", "Y"), ("pz", "Z")))
    add_group(0, 2, "世界线速度", "m/s",
              (("vx", "Vx"), ("vy", "Vy"), ("vz", "Vz")))
    add_group(4, 0, "姿态角", "°",
              (("yaw", "Yaw"), ("pitch", "Pitch"), ("roll", "Roll")))
    add_group(4, 2, "机体角速度", "°/s",
              (("wx", "ωx"), ("wy", "ωy"), ("wz", "ωz")))
    add_group(8, 0, "姿态四元数", "wxyz",
              (("qw", "qw"), ("qx", "qx"), ("qy", "qy"), ("qz", "qz")))

    force_frame = tk.Frame(root, bg="#1b222a", highlightthickness=1,
                           highlightbackground="#303b46")
    force_frame.pack(fill="x", padx=16, pady=(0, 10))
    for col, (key, label) in enumerate((
        ("active", "主动六维力/力矩"),
        ("passive", "被动六维力/力矩"),
        ("total", "合计六维力/力矩"),
    )):
        tk.Label(force_frame, text=label, bg="#1b222a", fg=MUTED,
                 font=("Microsoft YaHei UI", 9)).grid(
                     row=0, column=col, padx=12, pady=(9, 3), sticky="w")
        tk.Label(force_frame, textvariable=fields[key], bg="#1b222a", fg=FG,
                 font=("Consolas", 9), wraplength=225, justify="left").grid(
                     row=1, column=col, padx=12, pady=(3, 10), sticky="w")

    tk.Label(root, textvariable=rate_var, bg=BG, fg=MUTED,
             font=("Consolas", 10), anchor="e", padx=18, pady=8).pack(fill="x")

    subscriber = SensorSubscriber(topics=(TOPIC_STATE,))
    print(f"[UDP订阅] 仅订阅 {TOPIC_STATE} <- 127.0.0.1:18980")
    alive = True
    count = 0
    last_rate = time.monotonic()

    def fmt(v, digits=4):
        return "--" if v is None else f"{float(v):+.{digits}f}"

    def set_vec(keys, values, digits=4):
        values = values or []
        for key, val in zip(keys, values):
            fields[key].set(fmt(val, digits))

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
                set_vec(("px", "py", "pz"), data.get("position_world_m"))
                set_vec(("vx", "vy", "vz"), data.get("linear_velocity_world_mps"))
                set_vec(("yaw", "pitch", "roll"), data.get("orientation_yaw_pitch_roll_deg"), 2)
                set_vec(("qw", "qx", "qy", "qz"), data.get("orientation_quaternion_wxyz"), 5)
                set_vec(("wx", "wy", "wz"), data.get("angular_velocity_body_deg_s"), 2)
                for key, source in (("active", "active_force_body"),
                                    ("passive", "passive_force_body"),
                                    ("total", "total_force_body")):
                    vector = data.get(source) or []
                    fields[key].set("  ".join(fmt(v, 3) for v in vector) if vector else "--")
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
