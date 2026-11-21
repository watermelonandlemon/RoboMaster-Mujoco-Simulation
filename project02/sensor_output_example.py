# -*- coding: utf-8 -*-
"""飞镖传感器 UDP 订阅示例，支持同时运行多个进程。

python sensor_output_example.py                   # 数值 + 画面
python sensor_output_example.py --mode state      # 只接收数值，不传视频
python sensor_output_example.py --mode image      # 只接收视频
python sensor_output_example.py --mode state --fields acceleration_body_mps2 angular_velocity_body_rad_s
"""
import argparse
import time
import tkinter as tk

from bridge_ipc import SensorSubscriber, TOPIC_STATE, TOPIC_IMAGE


def fmt_vector(values):
    if values is None:
        return "（未订阅）"
    return "  ".join(f"{float(v):+.3f}" for v in values)


def main():
    parser = argparse.ArgumentParser(description="飞镖 UDP 传感器订阅程序")
    parser.add_argument("--mode", choices=("both", "state", "image"), default="both",
                        help="both=全部，state=仅数字，image=仅图像")
    parser.add_argument("--fields", nargs="*", default=None,
                        help="仅 state 模式有效：可筛选数字字段，例如 acceleration_body_mps2")
    args = parser.parse_args()

    topics = {
        "both": (TOPIC_STATE, TOPIC_IMAGE),
        "state": (TOPIC_STATE,),
        "image": (TOPIC_IMAGE,),
    }[args.mode]

    root = tk.Tk()
    root.title("RoboMaster · 飞镖传感订阅 / UDP · " + args.mode)
    root.geometry("840x790" if TOPIC_IMAGE in topics else "710x320")
    root.configure(bg="#111417")

    font = ("Consolas", 11)
    info = tk.StringVar(value=f"订阅 {', '.join(topics)}；等待 MuJoCo 发布...")
    tval = tk.StringVar(value="仿真时间：-- s")
    aval = tk.StringVar(value="a_xyz [m/s²]：--")
    wval = tk.StringVar(value="ω_xyz [rad/s]：--")
    rates = tk.StringVar(value="状态 -- Hz  /  视频 -- FPS  /  目标 30 Hz")

    for variable in (info, tval, aval, wval, rates):
        tk.Label(root, textvariable=variable, anchor="w", bg="#111417", fg="#e6eef7",
                 font=font, padx=15, pady=8).pack(fill="x")

    image_label = None
    if TOPIC_IMAGE in topics:
        image_label = tk.Label(root, text="等待图像订阅数据", bg="#202831", fg="#a8b7c6")
        image_label.pack(pady=8)
    else:
        tk.Label(root, text="仅订阅数字话题：没有视频渲染、压缩或传输",
                 bg="#111417", fg="#95cabb", font=font).pack(pady=10)
    tk.Label(root, text="可多开本程序；各进程独立选择订阅话题。关闭此窗口不会影响 MuJoCo。",
             bg="#111417", fg="#9aa7b4", font=("Microsoft YaHei UI", 9)).pack(pady=5)

    subscriber = SensorSubscriber(topics=topics, fields=args.fields)
    print(f"[UDP订阅] {', '.join(topics)} <- 127.0.0.1:18980")
    if args.fields:
        print(f"[UDP订阅] 数字字段过滤：{args.fields}")

    alive = True

    def close_window():
        nonlocal alive
        alive = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close_window)
    state_count = image_count = 0
    last_rate_time = time.monotonic()
    last_received = last_rate_time

    try:
        while alive:
            try:
                root.update()
            except tk.TclError:
                break

            event = subscriber.recv(timeout=0.02)
            if event is not None:
                last_received = time.monotonic()
                topic = event["topic"]
                t = float(event["time_s"])
                tval.set(f"仿真时间：t = {t:.4f} s  /  同步帧 #{event['frame_id']}")
                if topic == TOPIC_STATE:
                    state_count += 1
                    values = event.get("data", {})
                    aval.set("a_xyz [m/s²]：" + fmt_vector(values.get("acceleration_body_mps2")))
                    wval.set("ω_xyz [rad/s]：" + fmt_vector(values.get("angular_velocity_body_rad_s")))
                    info.set("已订阅数字话题 dart/state（独立 UDP 数据包）")
                elif topic == TOPIC_IMAGE:
                    image_count += 1
                    w, h = event["width"], event["height"]
                    ppm = f"P6\n{w} {h}\n255\n".encode("ascii") + event["image_rgb"]
                    img = tk.PhotoImage(data=ppm, format="PPM")
                    image_label.configure(image=img, text="", width=w, height=h)
                    image_label.image = img
                    info.set(f"已订阅图像话题 dart/image · {w}×{h} · RGB24 + zlib")

            now = time.monotonic()
            if now - last_rate_time >= 1.0:
                interval = now - last_rate_time
                state_hz = state_count / interval if TOPIC_STATE in topics else None
                image_fps = image_count / interval if TOPIC_IMAGE in topics else None
                s = f"{state_hz:.1f} Hz" if state_hz is not None else "未订阅"
                v = f"{image_fps:.1f} FPS" if image_fps is not None else "未订阅"
                rates.set(f"数字：{s}   /   图像：{v}   /   目标 30 Hz")
                state_count = image_count = 0
                last_rate_time = now
            if now - last_received > 2.5:
                info.set("等待 MuJoCo UDP 传感发布服务 127.0.0.1:18980...")
    except KeyboardInterrupt:
        pass
    finally:
        subscriber.close()
        if alive:
            try:
                root.destroy()
            except tk.TclError:
                pass


if __name__ == "__main__":
    main()
