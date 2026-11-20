# -*- coding: utf-8 -*-
import math
import time
import tkinter as tk
from tkinter import messagebox, ttk

import mujoco
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# ======================== 默认控制参数 ========================

DEFAULT_YAW = 0.0          # 偏航角（°）
DEFAULT_PITCH = 0.0        # 俯仰角（°）
DEFAULT_ROLL = 0.0         # 滚转角（°）

DEFAULT_SPEED = 15.0       # 初始速度（m/s）
MIN_SPEED = 0.0
MAX_SPEED = 40.0
SPEED_RESOLUTION = 0.1

# 实时曲线：按仿真时间采样，按实际时间限制 GUI 重绘频率。
DISTANCE_SAMPLE_INTERVAL = 0.02  # 仿真时间 20ms / 点
DISTANCE_PLOT_INTERVAL = 0.10    # 实际时间 100ms / 次图形刷新

# 靶中心使用 model.xml 中静态碰撞箱的几何中心（不是世界坐标原点）。
DISTANCE_TARGETS = (
    ("base", "基地", "dart_target_collision_1"),
    ("outpost", "前哨站", "dart_target_collision_2"),
)


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

        # xipos 是飞镖的世界坐标系质心；geom_xpos 是靶碰撞箱的世界坐标系几何中心。
        self.dart_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "dart_preview"
        )
        if self.dart_body_id < 0:
            raise RuntimeError("模型中找不到飞镖刚体 dart_preview")

        self.target_geom_ids = {}
        for target_key, _, geom_name in DISTANCE_TARGETS:
            geom_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_GEOM, geom_name
            )
            if geom_id < 0:
                raise RuntimeError(f"模型中找不到目标碰撞体 {geom_name}")
            self.target_geom_ids[target_key] = geom_id

        self.distance_history = {}
        self.last_distance_sample_time = float("-inf")
        self.last_distance_plot_time = float("-inf")

        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()
        self.closing = False

        self.root = tk.Tk()
        self.root.title("RoboMaster 飞镖发射与距离监测")
        self.root.geometry("1230x700")
        self.root.minsize(1030, 620)
        self.selected_target = tk.StringVar(master=self.root, value="base")

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

        # 左侧保留原有的发射控制，右侧显示实时距离曲线。
        outer = ttk.Frame(self.root, padding=12)
        outer.pack(fill="both", expand=True)

        main_frame = ttk.Frame(outer, width=455)
        main_frame.pack(side="left", fill="y", padx=(0, 12))
        main_frame.pack_propagate(False)

        chart_frame = ttk.LabelFrame(
            outer, text="3. 与目标中心的距离（实时绘图）", padding=8
        )
        chart_frame.pack(side="left", fill="both", expand=True)

        ttk.Label(
            main_frame,
            text="Dart Launch Control / 飞镖发射控制",
            font=("Microsoft YaHei UI", 15, "bold"),
        ).pack(anchor="w", pady=(0, 5))

        ttk.Label(
            main_frame,
            text="设置初始角度和速度，点击“应用参数并重置”，再点击“运行”。",
            wraplength=430,
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
            wraplength=405,
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
            length=410,
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
            wraplength=430,
        ).pack(anchor="w", pady=(0, 6))

        ttk.Label(
            main_frame,
            text="操作流程：设置角度和速度 → 应用参数并重置 → 运行。关闭任一窗口即可结束。",
            wraplength=430,
        ).pack(anchor="w")

        self._build_distance_chart(chart_frame)

    # ======================== 距离采样与实时曲线 ========================

    def _build_distance_chart(self, parent):
        """嵌入右侧的时间序列图；同一时刻只展示一个被选目标。"""
        target_row = ttk.Frame(parent)
        target_row.pack(fill="x", pady=(2, 6))
        ttk.Label(target_row, text="监测目标：").pack(side="left")
        for target_key, target_label, _ in DISTANCE_TARGETS:
            ttk.Radiobutton(
                target_row,
                text=target_label,
                value=target_key,
                variable=self.selected_target,
                command=self._on_target_changed,
            ).pack(side="left", padx=(2, 12))

        ttk.Label(
            parent,
            text="Δ = 目标中心 − 飞镖质心；ΔX / ΔY / ΔZ 为有符号坐标差，D 为三维直线距离。",
            foreground="#555555",
            wraplength=680,
        ).pack(anchor="w", pady=(0, 5))

        values_row = ttk.Frame(parent)
        values_row.pack(fill="x", pady=(0, 6))
        self.distance_texts = {}
        for column, (key, label, color) in enumerate((
            ("dx", "ΔX", "#2166ac"),
            ("dy", "ΔY", "#d95f02"),
            ("dz", "ΔZ", "#1b9e77"),
            ("distance", "D", "#7b3294"),
        )):
            values_row.columnconfigure(column, weight=1)
            cell = ttk.Frame(values_row)
            cell.grid(row=0, column=column, sticky="ew", padx=(0, 5))
            ttk.Label(cell, text=label, foreground=color).pack(anchor="center")
            self.distance_texts[key] = tk.StringVar(master=self.root, value="-- m")
            ttk.Label(
                cell, textvariable=self.distance_texts[key],
                font=("Consolas", 11, "bold"),
            ).pack(anchor="center")

        # FigureCanvasTkAgg 把 Matplotlib 曲线直接放进 Tkinter 面板。
        self.distance_figure = Figure(figsize=(7, 5), dpi=100)
        self.distance_axes = self.distance_figure.add_subplot(111)
        self.distance_figure.subplots_adjust(
            left=0.11, right=0.97, top=0.97, bottom=0.12
        )
        self.distance_axes.set_xlabel("Simulation time (s)")
        self.distance_axes.set_ylabel("Difference / distance (m)")
        self.distance_axes.set_xlim(0.0, 3.0)
        self.distance_axes.set_ylim(-10.0, 30.0)
        self.distance_axes.grid(True, linestyle="--", alpha=0.30)
        self.distance_axes.axhline(0.0, color="#777777", linewidth=0.8)

        self.distance_lines = {}
        for key, label, color in (
            ("dx", "ΔX", "#2166ac"),
            ("dy", "ΔY", "#d95f02"),
            ("dz", "ΔZ", "#1b9e77"),
            ("distance", "D = |Δr|", "#7b3294"),
        ):
            (line,) = self.distance_axes.plot(
                [], [], color=color, linewidth=1.9, label=label
            )
            self.distance_lines[key] = line
        self.distance_axes.legend(loc="upper right", ncol=2, framealpha=0.9)

        self.distance_canvas = FigureCanvasTkAgg(
            self.distance_figure, master=parent
        )
        self.distance_canvas.get_tk_widget().pack(fill="both", expand=True)
        self.distance_canvas.draw()

        self.plot_time_text = tk.StringVar(master=self.root, value="仿真时间：0.000 s")
        ttk.Label(
            parent, textvariable=self.plot_time_text, foreground="#444444"
        ).pack(anchor="e", pady=(5, 0))

    def _reset_distance_history(self):
        """新一次发射从 t=0 重新绘图，同时缓存两个靶的历史数据。"""
        self.distance_history = {
            key: {"time": [], "dx": [], "dy": [], "dz": [], "distance": []}
            for key, _, _ in DISTANCE_TARGETS
        }
        self.last_distance_sample_time = float("-inf")
        self._sample_distances(force=True)
        self._update_distance_chart(force=True)

    def _sample_distances(self, force=False):
        """按仿真时间采样，不在每一个 0.2ms 物理步都更新 GUI。"""
        sim_time = float(self.data.time)
        if (not force and
                sim_time - self.last_distance_sample_time < DISTANCE_SAMPLE_INTERVAL - 1e-9):
            return

        self.last_distance_sample_time = sim_time
        dart_center = self.data.xipos[self.dart_body_id]

        for target_key, geom_id in self.target_geom_ids.items():
            target_center = self.data.geom_xpos[geom_id]
            dx = float(target_center[0] - dart_center[0])
            dy = float(target_center[1] - dart_center[1])
            dz = float(target_center[2] - dart_center[2])
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)

            history = self.distance_history[target_key]
            history["time"].append(sim_time)
            history["dx"].append(dx)
            history["dy"].append(dy)
            history["dz"].append(dz)
            history["distance"].append(distance)

    def _on_target_changed(self):
        """目标切换时可直接显示当前目标从发射时刻起的全部曲线。"""
        self._update_distance_chart(force=True)

    def _update_distance_chart(self, force=False):
        """按墙上时钟限速刷新图形，保证物理步进不会被频繁绘图拖慢。"""
        now = time.perf_counter()
        if (not force and
                now - self.last_distance_plot_time < DISTANCE_PLOT_INTERVAL):
            return
        self.last_distance_plot_time = now

        target_key = self.selected_target.get()
        history = self.distance_history.get(target_key)
        if not history or not history["time"]:
            return

        for key, line in self.distance_lines.items():
            line.set_data(history["time"], history[key])
            self.distance_texts[key].set(f"{history[key][-1]:+.3f} m" if key != "distance"
                                         else f"{history[key][-1]:.3f} m")

        t_max = history["time"][-1]
        self.plot_time_text.set(f"仿真时间：{t_max:.3f} s")
        self.distance_axes.set_xlim(0.0, max(3.0, t_max * 1.05))

        # ΔX/ΔY/ΔZ 可能为负；根据全部曲线的极值自适应设置纵轴。
        all_values = [
            v for key in ("dx", "dy", "dz", "distance")
            for v in history[key]
        ]
        bottom = min(0.0, min(all_values))
        top = max(0.0, max(all_values))
        margin = max(0.5, (top - bottom) * 0.08)
        self.distance_axes.set_ylim(bottom - margin, top + margin)
        self.distance_canvas.draw_idle()

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

        # 每次应用参数或重置，都同步清空并重建实时距离曲线。
        self._reset_distance_history()
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
            steps = min(int(self.accumulator / timestep), 100)

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
                        # mj_step 后 qpos 已更新；重新计算位姿以取得当前时刻的 xipos。
                        mujoco.mj_kinematics(self.model, self.data)
                        # 在 viewer 锁内读取状态，两个目标分别保存历史数据。
                        self._sample_distances()

                if self.running:
                    self.accumulator = max(
                        0.0,
                        self.accumulator - steps * timestep,
                    )

        # GUI 绘图频率独立于 MuJoCo 物理时间步长（不会等仿真结束才绘图）。
        if self.running:
            self._update_distance_chart()
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
