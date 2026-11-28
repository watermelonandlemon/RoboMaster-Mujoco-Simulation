# -*- coding: utf-8 -*-
"""滑模制导：仅使用 dart/state 的基地靶板碰撞盒中心和飞镖世界状态。
运行 python active.py；终端模式 python active.py --no-plot。
"""
import argparse
from collections import deque
import math
import time
try:
    import tkinter as tk
except ImportError:
    tk = None

from simulator_link import ForcePublisher, SensorSubscriber, TOPIC_STATE, FrameTracker
from record import RunRecorder
from guidance import DEFAULT_PARAMS, compute_guidance_acceleration, world_acceleration_to_flight_body_force, TargetMotionEstimator

LABEL = '滑模制导'
DART_MASS_KG = 0.3
MAX_DATA_AGE_S = 0.25
ZERO_WRENCH = (0.0,) * 6

def vector(data, key, n):
    values = data.get(key)
    if not isinstance(values, (list, tuple)) or len(values) != n or not all(
        type(x) in (int, float) and math.isfinite(x) for x in values
    ):
        raise ValueError(f"dart/state 缺失或无效字段：{key}")
    return tuple(map(float, values))


class StateController:
    """一次计算只取一帧状态，返回六维指令、曲线数据及诊断。"""
    def __init__(self):
        self.reset()

    def reset(self):
        self.motion = TargetMotionEstimator()

    def sample(self, t, data):
        if not isinstance(data, dict):
            raise ValueError("dart/state 的 data 不是字典")
        target = vector(data, "target_position_world_m", 3)
        dart = vector(data, "position_world_m", 3)
        vel = vector(data, "linear_velocity_world_mps", 3)
        quat = vector(data, "orientation_quaternion_wxyz", 4)
        if max(map(abs, quat)) < 1e-9:
            raise ValueError("姿态四元数无效")
        relative = tuple(ti - pi for ti, pi in zip(target, dart))
        # 固定靶速度为零；移动靶沿世界 Y 运动。保留原包的差分滤波。
        # 暂停/初始化时同一仿真时间可能反复发布，不重复更新滤波器。
        if self.motion.last_time is not None and (t < self.motion.last_time or
                (t == self.motion.last_time and target != self.motion.last_position)):
            self.motion.reset()
        if t != self.motion.last_time:
            self.motion.update(target, t)
        a, diagnostics = compute_guidance_acceleration(dart, vel, target, self.motion.velocity,
                                             self.motion.acceleration)
        force = world_acceleration_to_flight_body_force(a, quat, mass_kg=DART_MASS_KG)
        if not all(math.isfinite(value) for value in force):
            raise ValueError("非有限主动力")
        diagnostics.update(relative_position_world_m=relative,
                           guidance_acceleration_world_mps2=a)
        diagnostics.update(target_motion_source="estimated_from_state_y",
                           target_velocity_est_world_mps=self.motion.velocity,
                           target_acceleration_est_world_mps2=self.motion.acceleration)
        return (*force, 0.0, 0.0, 0.0), relative, vel, diagnostics


class GuidanceMonitor:
    """实时显示目标相对位置与飞镖世界系速度；不参与制导计算。"""

    def __init__(self, history_s=10.0):
        self.history_s = float(history_s)
        self.samples = deque(maxlen=600)
        self.alive = True
        self.last_draw = 0.0

        self.root = tk.Tk()
        self.root.title(f"RoboMaster · {LABEL} · 靶板中心真值")
        self.root.geometry("1100x680")
        self.root.configure(bg="#101722")
        self.root.protocol("WM_DELETE_WINDOW", self.close)

        header = tk.Frame(self.root, bg="#101722")
        header.pack(fill="x", padx=16, pady=(12, 4))
        tk.Label(
            header, text="GUIDANCE TELEMETRY", bg="#101722", fg="#8bd5ff",
            font=("Consolas", 11, "bold")
        ).pack(side="left")
        self.status = tk.StringVar(value="等待 dart/state 真值")
        tk.Label(
            header, textvariable=self.status, bg="#101722", fg="#cbd5e1",
            font=("Microsoft YaHei UI", 10)
        ).pack(side="right")

        self.readout = tk.StringVar(
            value="目标相对位置：X -- m   Y -- m   Z -- m       "
                  "世界速度：Vx -- m/s   Vy -- m/s   Vz -- m/s"
        )
        tk.Label(
            self.root, textvariable=self.readout, bg="#182435", fg="#e8f1fb",
            font=("Consolas", 11), anchor="w", padx=12, pady=10
        ).pack(fill="x", padx=16, pady=(6, 8))

        self.canvas = tk.Canvas(
            self.root, width=1060, height=550, bg="#111c2a",
            highlightthickness=1, highlightbackground="#34465d"
        )
        self.canvas.pack(fill="both", expand=True, padx=16, pady=(0, 14))
        self._draw()

    def close(self):
        self.alive = False
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def add_sample(self, sim_time, relative_position, velocity_world):
        pos = tuple(relative_position) if relative_position is not None else None
        vel = tuple(float(x) for x in velocity_world)
        if self.samples and sim_time < self.samples[-1][0]:
            self.samples.clear()  # 仿真重置：不要把上一发的轨迹连到新一发。
        if self.samples and sim_time == self.samples[-1][0]:
            self.samples.pop()    # 暂停时同一仿真时刻只保留最新一次显示。
        self.samples.append((float(sim_time), pos, vel))
        if pos is not None:
            self.readout.set(
                "目标相对位置："
                f"X {pos[0]:+.3f} m   Y {pos[1]:+.3f} m   Z {pos[2]:+.3f} m       "
                f"世界速度：Vx {vel[0]:+.3f} m/s   "
                f"Vy {vel[1]:+.3f} m/s   Vz {vel[2]:+.3f} m/s"
            )
        else:
            self.readout.set(
                "目标相对位置：不可用（状态字段无效）       "
                f"世界速度：Vx {vel[0]:+.3f} m/s   "
                f"Vy {vel[1]:+.3f} m/s   Vz {vel[2]:+.3f} m/s"
            )

    @staticmethod
    def _limits(values):
        finite = [v for v in values if v is not None and math.isfinite(v)]
        if not finite:
            return -1.0, 1.0
        lo, hi = min(finite), max(finite)
        if hi - lo < 1e-6:
            pad = max(0.5, abs(hi) * 0.15)
        else:
            pad = (hi - lo) * 0.12
        return lo - pad, hi + pad

    def _draw(self):
        c = self.canvas
        c.delete("all")
        width = max(700, c.winfo_width())
        height = max(400, c.winfo_height())
        left, right, top, bottom = 78, width - 82, 38, height - 58
        plot_w, plot_h = right - left, bottom - top

        now = self.samples[-1][0] if self.samples else 0.0
        visible = [s for s in self.samples if s[0] >= now - self.history_s]
        if not visible:
            visible = list(self.samples)

        positions = [[s[1][i] if s[1] is not None else None for s in visible]
                     for i in range(3)]
        velocities = [[s[2][i] for s in visible] for i in range(3)]
        pmin, pmax = self._limits([v for series in positions for v in series])
        vmin, vmax = self._limits([v for series in velocities for v in series])

        # 背景网格与左右双纵轴
        for i in range(6):
            y = top + plot_h * i / 5
            c.create_line(left, y, right, y, fill="#26384c")
            pv = pmax - (pmax - pmin) * i / 5
            vv = vmax - (vmax - vmin) * i / 5
            c.create_text(left - 12, y, text=f"{pv:.2f}", fill="#a8c7e8",
                          anchor="e", font=("Consolas", 9))
            c.create_text(right + 12, y, text=f"{vv:.2f}", fill="#f2c18d",
                          anchor="w", font=("Consolas", 9))
        for i in range(6):
            x = left + plot_w * i / 5
            c.create_line(x, top, x, bottom, fill="#1d2c3d")
            t = max(0.0, now - self.history_s) + max(1e-6, min(now, self.history_s)) * i / 5
            c.create_text(x, bottom + 20, text=f"{t:.1f}", fill="#9fb0c3",
                          font=("Consolas", 9))

        c.create_line(left, top, left, bottom, fill="#8bb8e8", width=2)
        c.create_line(right, top, right, bottom, fill="#e9b77e", width=2)
        c.create_line(left, bottom, right, bottom, fill="#9fb0c3", width=1)
        c.create_text(left, 13, text="目标相对位置 / m", fill="#8bb8e8",
                      anchor="w", font=("Microsoft YaHei UI", 10, "bold"))
        c.create_text(right, 13, text="飞镖世界速度 / m/s", fill="#e9b77e",
                      anchor="e", font=("Microsoft YaHei UI", 10, "bold"))
        c.create_text((left + right) / 2, height - 12, text="仿真时间 / s",
                      fill="#cbd5e1", font=("Microsoft YaHei UI", 9))

        if visible:
            t0 = max(0.0, now - self.history_s)
            tspan = max(now - t0, 1e-6)
            colors = ("#ff6b6b", "#55d6a5", "#64a9ff")
            labels = ("X / Vx", "Y / Vy", "Z / Vz")
            for axis in range(3):
                # 每段连续数据只创建一条折线，而不是为每对采样点创建 Tk 对象。
                for series, lo, hi, style in ((positions[axis], pmin, pmax, {}),
                                               (velocities[axis], vmin, vmax, {"dash": (5, 2)})):
                    points = []
                    for sample, value in zip(visible, series):
                        if value is None:
                            if len(points) >= 4:
                                c.create_line(*points, fill=colors[axis], width=2, **style)
                            points.clear()
                            continue
                        points.extend((left + (sample[0] - t0) / tspan * plot_w,
                                       bottom - (value - lo) / (hi - lo) * plot_h))
                    if len(points) >= 4:
                        c.create_line(*points, fill=colors[axis], width=2, **style)

            # 图例：实线为位置，虚线为速度；同一轴使用同色。
            lx = left + 12
            for i, (label, color) in enumerate(zip(labels, colors)):
                x = lx + i * 135
                c.create_line(x, top + 16, x + 28, top + 16,
                              fill=color, width=2)
                c.create_text(x + 35, top + 16, text=label,
                              fill="#e5edf7", anchor="w",
                              font=("Consolas", 9))
            c.create_line(lx + 420, top + 16, lx + 448, top + 16,
                          fill="#cbd5e1", width=2, dash=(5, 2))
            c.create_text(lx + 455, top + 16, text="虚线=速度，实线=位置",
                          fill="#cbd5e1", anchor="w",
                          font=("Microsoft YaHei UI", 8))

        if not visible:
            c.create_text((left + right) / 2, (top + bottom) / 2,
                          text="等待数据……", fill="#cbd5e1",
                          font=("Microsoft YaHei UI", 16))

    def update(self):
        if not self.alive:
            return
        try:
            self.root.update_idletasks()
            self.root.update()
            if not self.alive:
                return
            now = time.monotonic()
            if now - self.last_draw >= 0.05:
                self._draw()
                self.last_draw = now
        except tk.TclError:
            self.alive = False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-plot", action="store_true", help="不显示曲线，减少 GUI 对接收时序的影响")
    args = parser.parse_args()
    if not args.no_plot and tk is None:
        parser.error("当前 Python 无 Tk，请加 --no-plot 或安装 Tk")
    monitor = None if args.no_plot else GuidanceMonitor()
    controller, tracker = StateController(), FrameTracker()
    recorder = RunRecorder("active", LABEL, {
        "source": "RoboMaster-Mujoco-Simulation v2.6/project01",
        "algorithm": LABEL, "target": "dart_target_collision_1 geometry center",
        "state_topic": TOPIC_STATE, "mass_kg_reference": DART_MASS_KG,
        "gravity_world_mps2_reference": [0.0, 0.0, -9.81],
        "inertia_stl_kg_m2_reference": [0.001, 0.001, 0.0001],
        "model_reference_note": "来自 v2.6 model.xml，非运行时 UDP 实测；由 Project01 独立求解",
        "guidance_params": DEFAULT_PARAMS, "active_torque": [0.0, 0.0, 0.0],
        "gravity_in_command": False, "max_data_age_s": MAX_DATA_AGE_S,
    })
    last_zero = last_print = 0.0
    last_status = None
    timed_out = False

    def status(text):
        nonlocal last_status
        if text != last_status:
            if monitor is not None and monitor.alive:
                monitor.status.set(text)
            print(f"[{LABEL}] {text}", flush=True)
            last_status = text

    def send_zero(publisher, reason, event=None, received=None, timing=None, error=""):
        publisher.send(ZERO_WRENCH)
        recorder.write(event, reason, received, timing, command=ZERO_WRENCH,
                       publisher=publisher, error=error)

    try:
        with SensorSubscriber(topics=(TOPIC_STATE,)) as subscriber, ForcePublisher("active") as publisher:
            try:
                send_zero(publisher, "startup_zero")
                print(f"[{LABEL}] 只订阅 dart/state；目标=基地碰撞盒中心；主动输入 -> {publisher.address}", flush=True)
                print(f"[CSV] {recorder.directory}", flush=True)
                while True:
                    event = subscriber.recv(timeout=0.02)
                    now = time.monotonic()
                    if event is not None:
                        timing = {}
                        try:
                            frame_status, timing, accepted = tracker.accept(event, now)
                            if not accepted:
                                recorder.write(event, frame_status, now, timing)
                            else:
                                if frame_status in ("reset", "restart"):
                                    controller.reset()
                                    recorder.new_run()
                                    if monitor is not None and monitor.alive:
                                        monitor.samples.clear()
                                if timed_out:
                                    controller.reset()
                                    timed_out = False
                                t = float(event["time_s"])
                                wrench, relative, vel, diagnostic = controller.sample(t, event.get("data"))
                                publisher.send(wrench)
                                recorder.write(event, frame_status, now, timing, diagnostic, wrench, publisher)
                                status("靶板真值 · dart/state")
                                if monitor is not None and monitor.alive:
                                    monitor.add_sample(t, relative, vel)
                                if now - last_print >= 1.0:
                                    print(f"[t={t:.3f}s] ΔXYZ=({relative[0]:+.3f}, {relative[1]:+.3f}, "
                                          f"{relative[2]:+.3f}) m | 距离={math.hypot(*relative):.3f} m | "
                                          f"主动力={math.hypot(*wrench[:3]):.3f} N", flush=True)
                                    last_print = now
                        except (ValueError, TypeError, OverflowError, KeyError) as exc:
                            controller.reset()
                            send_zero(publisher, "invalid_state_zero", event, now, timing, str(exc))
                            last_zero = now
                            status(f"{exc}；主动输入=0")
                    if tracker.received is None or now - tracker.received > MAX_DATA_AGE_S:
                        if not timed_out:
                            controller.reset()
                            timed_out = True
                        if now - last_zero >= 0.08:
                            send_zero(publisher, "timeout_zero")
                            last_zero = now
                        status("等待 dart/state" if tracker.received is None else "dart/state 超时；主动输入=0")
                    if monitor is not None and monitor.alive:
                        monitor.update()
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    send_zero(publisher, "exit_zero")
                except OSError as exc:
                    print(f"[Active] 退出清零发送失败：{exc}", flush=True)
    finally:
        recorder.close()
        if monitor is not None and monitor.alive:
            monitor.close()
        print(f"[CSV] 记录结束：{recorder.directory}", flush=True)


if __name__ == "__main__":
    main()
