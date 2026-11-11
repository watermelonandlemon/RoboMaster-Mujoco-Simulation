# -*- coding: utf-8 -*-
import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

import mujoco


# ======================== 默认控制参数 ========================

DEFAULT_YAW = 0.0          # 偏航角（°）
DEFAULT_PITCH = 0.0        # 俯仰角（°）
DEFAULT_ROLL = 0.0         # 滚转角（°）

DEFAULT_SPEED = 15.0       # 初始速度（m/s）
MIN_SPEED = 0.0
MAX_SPEED = 40.0
SPEED_RESOLUTION = 0.1


# ======================== 姿态与速度计算 ========================

def axis_quat(axis, angle_deg):
    """生成绕指定轴旋转的四元数；输入角度单位为度（°）。"""
    half = math.radians(angle_deg) / 2
    c = math.cos(half)
    s = math.sin(half)

    if axis == "x":
        return (c, s, 0, 0)
    if axis == "y":
        return (c, 0, s, 0)
    return (c, 0, 0, s)


def quat_mul(a, b):
    """四元数相乘。"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b

    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )


def set_dart_attitude(model, yaw=0.0, pitch=0.0, roll=0.0):
    """
    设置飞镖姿态，三个输入角度的单位均为度（°）。

    yaw:   偏航角，正值向 +Y 转
    pitch: 俯仰角，正值抬头
    roll:  滚转角，绕飞镖自身长轴旋转

    STL 原始长轴沿 Z，最后通过 Y 轴旋转 90°，
    使飞镖机头朝向世界 +X。
    """
    q = quat_mul(
        quat_mul(
            quat_mul(
                axis_quat("z", yaw),
                axis_quat("y", -pitch)
            ),
            axis_quat("x", roll)
        ),
        axis_quat("y", 90)
    )

    model.body("dart_preview").quat[:] = q
    return q


def launch_velocity(speed, pitch_deg, yaw_deg):
    """
    根据初速度大小、俯仰角和偏航角计算世界坐标系下的速度分量。

    speed:    初速度大小（m/s）
    pitch_deg:俯仰角（°），正值向上
    yaw_deg:  偏航角（°），正值向 +Y
    """
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    return [
        speed * math.cos(pitch) * math.cos(yaw),
        speed * math.cos(pitch) * math.sin(yaw),
        speed * math.sin(pitch),
    ]


# ======================== 摄像机 ========================

def camera_init(
    viewer,
    lookat=(14.0, 7.5, 1.0),
    position=(-10.0, 7.5, -3.0)
):
    """
    初始化自由摄像头。

    lookat: 摄像头观察目标
    position: 摄像头初始位置

    初始化后仍然可以用鼠标自由操作。
    """
    dx = position[0] - lookat[0]
    dy = position[1] - lookat[1]
    dz = position[2] - lookat[2]

    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    azimuth = math.degrees(math.atan2(dy, dx))
    elevation = math.degrees(math.atan2(dz, math.hypot(dx, dy)))

    with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = lookat
        viewer.cam.distance = distance
        viewer.cam.azimuth = azimuth
        viewer.cam.elevation = elevation

    viewer.sync()


# ======================== 仿真控制面板 ========================

class DartSimulationApp:
    """飞镖仿真控制面板及 MuJoCo 仿真循环。"""

    def __init__(self, model, data, viewer, initial_qpos, qpos_adr, qvel_adr):
        self.model = model
        self.data = data
        self.viewer = viewer
        self.initial_qpos = initial_qpos.copy()
        self.qpos_adr = qpos_adr
        self.qvel_adr = qvel_adr

        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()
        self.closing = False

        self.root = tk.Tk()
        self.root.title("RoboMaster 飞镖发射参数控制面板")
        self.root.geometry("600x610")
        self.root.minsize(560, 570)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # 启动时应用默认参数，但保持暂停。
        self._apply_initial_state()
        self.root.after(8, self._simulation_tick)

    def _build_ui(self):
        style = ttk.Style()
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        main_frame = ttk.Frame(self.root, padding=12)
        main_frame.pack(fill="both", expand=True)

        ttk.Label(
            main_frame,
            text="Dart Launch Control / 飞镖发射控制",
            font=("Microsoft YaHei UI", 15, "bold"),
        ).pack(anchor="w", pady=(0, 5))

        ttk.Label(
            main_frame,
            text="设置初始角度和速度，点击“应用参数并重置”，再点击“运行”。",
            wraplength=550,
        ).pack(anchor="w", pady=(0, 10))

        # ---------- 角度设置 ----------
        angle_frame = ttk.LabelFrame(
            main_frame,
            text="1. 发射角度 Attitude（单位：°）",
            padding=10,
        )
        angle_frame.pack(fill="x", pady=5)

        self.yaw_var = tk.StringVar(value=str(DEFAULT_YAW))
        self.pitch_var = tk.StringVar(value=str(DEFAULT_PITCH))
        self.roll_var = tk.StringVar(value=str(DEFAULT_ROLL))

        angle_items = (
            ("偏航角 YAW（°）", self.yaw_var),
            ("俯仰角 PITCH（°）", self.pitch_var),
            ("滚转角 ROLL（°）", self.roll_var),
        )

        for column, (label_text, variable) in enumerate(angle_items):
            angle_frame.columnconfigure(column, weight=1)
            ttk.Label(angle_frame, text=label_text).grid(
                row=0, column=column, sticky="w", padx=5, pady=(0, 5)
            )
            ttk.Entry(
                angle_frame,
                textvariable=variable,
                width=14,
                justify="center",
            ).grid(
                row=1, column=column, sticky="ew", padx=5, pady=(0, 5)
            )

        ttk.Label(
            angle_frame,
            text="正俯仰角表示抬头；偏航角和俯仰角决定初速度方向；滚转角只改变飞镖绕自身长轴的姿态。",
            wraplength=530,
        ).grid(row=2, column=0, columnspan=3, sticky="w", padx=5, pady=(4, 0))

        # ---------- 初始速度 ----------
        speed_frame = ttk.LabelFrame(
            main_frame,
            text="2. 初始速度 Velocity",
            padding=10,
        )
        speed_frame.pack(fill="x", pady=5)

        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED)
        self.speed_value_text = tk.StringVar(value=f"{DEFAULT_SPEED:.3f}")
        self.velocity_text = tk.StringVar(value="")
        self.angle_summary_text = tk.StringVar(value="")

        speed_top = ttk.Frame(speed_frame)
        speed_top.pack(fill="x")

        ttk.Label(speed_top, text="初速度大小（m/s）").pack(side="left")
        ttk.Label(
            speed_top,
            textvariable=self.speed_value_text,
            width=12,
            anchor="e",
        ).pack(side="right")

        self.speed_scale = tk.Scale(
            speed_frame,
            variable=self.speed_var,
            from_=MIN_SPEED,
            to=MAX_SPEED,
            orient="horizontal",
            resolution=SPEED_RESOLUTION,
            showvalue=False,
            length=520,
            highlightthickness=0,
        )
        self.speed_scale.pack(fill="x")

        ttk.Label(
            speed_frame,
            textvariable=self.angle_summary_text,
            foreground="#444444",
        ).pack(anchor="w", pady=(4, 2))

        ttk.Label(
            speed_frame,
            textvariable=self.velocity_text,
            foreground="#1769aa",
        ).pack(anchor="w")

        # 参数变化时实时更新速度分量预览；实际仿真状态需点击“应用参数并重置”。
        self.speed_var.trace_add("write", self._update_velocity_display)
        self.yaw_var.trace_add("write", self._update_velocity_display)
        self.pitch_var.trace_add("write", self._update_velocity_display)
        self.roll_var.trace_add("write", self._update_velocity_display)
        self._update_velocity_display()

        # ---------- 按钮 ----------
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill="x", pady=12)

        self.apply_button = ttk.Button(
            button_frame,
            text="应用参数并重置",
            command=self._apply_initial_state,
        )
        self.apply_button.pack(side="left", fill="x", expand=True, padx=(0, 4))

        self.run_button = ttk.Button(
            button_frame,
            text="运行 Run",
            command=self._toggle_run,
        )
        self.run_button.pack(side="left", fill="x", expand=True, padx=4)

        self.reset_button = ttk.Button(
            button_frame,
            text="重置 Reset",
            command=self._apply_initial_state,
        )
        self.reset_button.pack(side="left", fill="x", expand=True, padx=(4, 0))

        self.status_text = tk.StringVar(value="状态：模型已加载，等待设置发射参数。")
        ttk.Label(
            main_frame,
            textvariable=self.status_text,
            wraplength=550,
        ).pack(anchor="w", pady=(0, 6))

        ttk.Label(
            main_frame,
            text="操作流程：设置角度和速度 → 应用参数并重置 → 运行。关闭任一窗口即可结束。",
            wraplength=550,
        ).pack(anchor="w")

    def _read_parameters(self):
        """读取并检查面板参数，角度统一按度（°）解释。"""
        speed = float(self.speed_var.get())
        yaw = float(self.yaw_var.get())
        pitch = float(self.pitch_var.get())
        roll = float(self.roll_var.get())

        if not all(math.isfinite(value) for value in (speed, yaw, pitch, roll)):
            raise ValueError("参数必须是有限数值。")

        if not MIN_SPEED <= speed <= MAX_SPEED:
            raise ValueError(f"初速度必须在 {MIN_SPEED}～{MAX_SPEED} m/s 范围内。")

        return speed, yaw, pitch, roll

    def _update_velocity_display(self, *_):
        """根据当前面板参数实时显示角度和速度分量。"""
        try:
            speed, yaw, pitch, roll = self._read_parameters()
            vx, vy, vz = launch_velocity(speed, pitch, yaw)

            self.speed_value_text.set(f"{speed:.3f}")
            self.angle_summary_text.set(
                f"YAW = {yaw:.2f}°    PITCH = {pitch:.2f}°    ROLL = {roll:.2f}°"
            )
            self.velocity_text.set(
                f"Vx = {vx:.3f}    Vy = {vy:.3f}    Vz = {vz:.3f} m/s"
            )
        except (tk.TclError, ValueError):
            self.speed_value_text.set("--")
            self.angle_summary_text.set("角度：请输入有效数值（单位：°）")
            self.velocity_text.set("速度分量：请输入有效参数")

    def _apply_initial_state(self):
        """根据面板参数更新飞镖姿态和初速度，并重置仿真。"""
        try:
            speed, yaw, pitch, roll = self._read_parameters()
        except (tk.TclError, ValueError) as error:
            messagebox.showerror("参数错误", str(error), parent=self.root)
            return

        quat = set_dart_attitude(
            self.model,
            yaw=yaw,
            pitch=pitch,
            roll=roll,
        )
        velocity = launch_velocity(speed, pitch, yaw)

        # 更新 freejoint 的初始四元数，也同步模型默认状态，保证 MuJoCo 重置一致。
        self.initial_qpos[self.qpos_adr + 3:self.qpos_adr + 7] = quat
        self.model.qpos0[self.qpos_adr + 3:self.qpos_adr + 7] = quat

        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()

        with self.viewer.lock():
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[:] = self.initial_qpos
            self.data.qvel[:] = 0.0

            # freejoint 的前三个速度分量为世界坐标系线速度。
            self.data.qvel[self.qvel_adr:self.qvel_adr + 3] = velocity

            mujoco.mj_forward(self.model, self.data)

        self.viewer.sync()
        self.run_button.configure(text="运行 Run")
        self.status_text.set(
            f"状态：参数已应用。速度={speed:.2f} m/s；"
            f"YAW={yaw:.2f}°，PITCH={pitch:.2f}°，ROLL={roll:.2f}°。"
        )

    def _toggle_run(self):
        self.running = not self.running
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()

        self.run_button.configure(
            text="暂停 Pause" if self.running else "运行 Run"
        )
        self.status_text.set(
            "状态：仿真运行中。" if self.running else "状态：仿真已暂停。"
        )

    def _simulation_tick(self):
        """在 Tkinter 事件循环中驱动 MuJoCo 仿真。"""
        if self.closing:
            return

        if not self.viewer.is_running():
            self._on_close()
            return

        now = time.perf_counter()
        elapsed = min(max(now - self.last_wall_time, 0.0), 0.1)
        self.last_wall_time = now

        if self.running:
            self.accumulator += elapsed
            timestep = float(self.model.opt.timestep)
            steps = min(int(self.accumulator / timestep), 50)

            if steps > 0:
                with self.viewer.lock():
                    for _ in range(steps):
                        old_time = self.data.time
                        old_pos = self.data.qpos[
                            self.qpos_adr:self.qpos_adr + 3
                        ].copy()

                        mujoco.mj_step(self.model, self.data)

                        # 如果仿真时间突然回到 0，说明发生了自动重置。
                        if self.data.time < old_time:
                            print("\n========== 检测到 MuJoCo 自动重置 ==========")
                            print(f"重置发生前时间：{old_time:.3f} s")
                            print(f"重置前位置：{old_pos}")
                            print(f"重置后时间：{self.data.time:.3f} s")
                            print(
                                "重置后位置："
                                f"{self.data.qpos[self.qpos_adr:self.qpos_adr + 3]}"
                            )
                            print("============================================\n")

                            self.running = False
                            self.accumulator = 0.0
                            self.run_button.configure(text="运行 Run")
                            self.status_text.set(
                                "状态：检测到 MuJoCo 自动重置，仿真已暂停。"
                            )
                            break

                if self.running:
                    self.accumulator = max(
                        0.0,
                        self.accumulator - steps * timestep,
                    )

        self.viewer.sync()
        self.root.after(8, self._simulation_tick)

    def _on_close(self):
        if self.closing:
            return

        self.closing = True
        try:
            if self.viewer.is_running():
                self.viewer.close()
        finally:
            self.root.destroy()

    def run(self):
        """启动控制面板事件循环。"""
        self.root.mainloop()
