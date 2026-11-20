# -*- coding: utf-8 -*-
"""传感端独立进程：python sensor_output_example.py。

自动连接 main.py 里的本机 TCP 服务；每帧收到同步的图像、加速度、角速度。
传感数据是在同一仿真状态上采样的；图像以 RGB24 传输。
无需安装 OpenCV/Pillow，使用 Python 自带 Tkinter 显示真实 MuJoCo FPV。
"""
import select
import socket
import tkinter as tk
import time

from bridge_ipc import HOST, SENSOR_PORT, read_sensor_frame


def on_sensor_packet(packet, image_rgb):
    """你的视觉/制导算法可以直接在这里处理接收到的真实数据。

    packet['time_s']                    仿真时刻，s
    packet['acceleration_body_mps2']    (ax, ay, az)，包含重力引起的运动加速度
    packet['angular_velocity_body_rad_s'] (wx, wy, wz)，机体系角速度
    image_rgb                           RGB24 bytes，长宽见 packet
    """
    return None


def fmt_triple(values, decimal=3):
    return "  ".join(f"{float(v):+.{decimal}f}" for v in values)


def main():
    root = tk.Tk()
    root.title("RoboMaster · 飞镖 FPV + 三轴传感 / 30 Hz")
    root.geometry("560x480")
    root.configure(bg="#111417")
    title = tk.StringVar(value="连接中……")
    tval = tk.StringVar(value="仿真时间：-- s")
    aval = tk.StringVar(value="加速度：-- m/s²")
    wval = tk.StringVar(value="角速度：-- rad/s")
    fps = tk.StringVar(value="接收帧率：-- FPS")
    font = ("Consolas", 11)
    for var in (title, tval, aval, wval, fps):
        tk.Label(root, textvariable=var, anchor="w", bg="#111417", fg="#e6eef7",
                 font=font, padx=15, pady=7).pack(fill="x")
    image_label = tk.Label(root, text="尚未收到 FPV 图像", bg="#202831", fg="#a8b7c6",
                           width=45, height=15, compound="center")
    image_label.pack(pady=8)
    tk.Label(root, text="关闭本窗口只会断开传感输出，仿真不受影响。",
             bg="#111417", fg="#9aa7b4", font=("Microsoft YaHei UI", 9)).pack()
    alive = True

    def close():
        nonlocal alive
        alive = False
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    recv_counter = 0
    last_fps_time = time.monotonic()
    last_print_time = -1.0
    try:
        while alive:
            try:
                root.update()
            except tk.TclError:
                break
            try:
                sock = socket.create_connection((HOST, SENSOR_PORT), timeout=0.4)
            except OSError:
                title.set(f"等待仿真服务  {HOST}:{SENSOR_PORT}")
                time.sleep(0.15)
                continue
            title.set("已连接，正在接收真实飞镖图像及传感数据")
            print(f"[传感接收] 已连接 {HOST}:{SENSOR_PORT}")
            try:
                with sock:
                    sock.settimeout(2.0)
                    recv_counter = 0
                    last_fps_time = time.monotonic()
                    while alive:
                        try:
                            root.update()
                        except tk.TclError:
                            alive = False
                            break
                        # 避免接收等待时 Tk 图形窗口失去响应
                        ready, _, _ = select.select([sock], [], [], 0.02)
                        if not ready:
                            continue
                        packet, image_rgb = read_sensor_frame(sock)
                        on_sensor_packet(packet, image_rgb)
                        t = float(packet["time_s"])
                        title.set(f"同步帧 #{packet['frame_id']}  |  机体系 +X 为机头")
                        tval.set(f"仿真时间：t = {t:.4f} s")
                        aval.set("a_xyz [m/s²]：" + fmt_triple(packet["acceleration_body_mps2"]))
                        wval.set("ω_xyz [rad/s]：" + fmt_triple(packet["angular_velocity_body_rad_s"]))
                        if image_rgb is not None:
                            w, h = packet["image_width"], packet["image_height"]
                            ppm = f"P6\n{w} {h}\n255\n".encode("ascii") + image_rgb
                            img = tk.PhotoImage(data=ppm, format="PPM")
                            image_label.config(image=img, text="", width=w, height=h)
                            image_label.image = img
                        else:
                            image_label.config(image="", text="离屏渲染暂不可用，请检查仿真端日志", width=45, height=15)
                            image_label.image = None
                        recv_counter += 1
                        now = time.monotonic()
                        if now - last_fps_time >= 1.0:
                            real_fps = recv_counter / (now - last_fps_time)
                            fps.set(f"接收帧率：{real_fps:.1f} FPS   /   目标 30 FPS")
                            print(f"[传感接收] 实测 {real_fps:.1f} FPS / 目标 30 FPS")
                            recv_counter = 0
                            last_fps_time = now
                        if now - last_print_time >= 1.0:
                            print(f"[传感接收] t={t:.3f}s, a={packet['acceleration_body_mps2']}, "
                                  f"ω={packet['angular_velocity_body_rad_s']}, "
                                  f"image={packet['image_width']}x{packet['image_height']}")
                            last_print_time = now
            except (OSError, ConnectionError, ValueError) as error:
                if alive:
                    print(f"[传感接收] 暂时断线，将自动重连：{error}")
                    title.set("已断开，自动重连中……")
    except KeyboardInterrupt:
        pass
    finally:
        if alive:
            try:
                root.destroy()
            except tk.TclError:
                pass


if __name__ == "__main__":
    main()
