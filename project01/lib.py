# -*- coding: utf-8 -*-
import csv
import math
import random
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import mujoco
from dart_interfaces import DartInterfaces
from bridge_ipc import LocalDartBridge, INTERFACE_PERIOD_S


# ======================== 面板初始参数 ========================

DEFAULT_YAW = -15.2          # 偏航角（°）
DEFAULT_PITCH = 27.0        # 俯仰角（°）
DEFAULT_ROLL = 0.0         # 滚转角（°）

DEFAULT_SPEED = 18.0       # 初始速度（m/s）
MIN_SPEED = 5.0
MAX_SPEED = 30.0
SPEED_RESOLUTION = 0.1

# 实时状态：监控画面按实际时间刷新，历史数据按仿真时间抽样。
STATE_MONITOR_REFRESH_INTERVAL_S = 0.10  # 最多每秒更新 10 次界面
STATE_HISTORY_SAMPLE_INTERVAL_S = 0.05   # 每 0.05 秒仿真时间记录一条

# 对面基地靶（位于场地 X≈25.6m）沿 Y 轴平移，单位为 mm。
BASE_TARGET_BODY_NAME = "base_right_target"
BASE_TARGET_MIN_OFFSET_MM = -280
BASE_TARGET_MAX_OFFSET_MM = 280

# 镖靶目标模式。所有时间均为仿真时间（data.time），与 GUI 刷新频率无关。
TARGET_MODE_FIXED = "fixed"
TARGET_MODE_RANDOM_FIXED = "random_fixed"
TARGET_MODE_RANDOM_MOVING = "random_moving"
TARGET_MODE_TERMINAL_MOVING = "terminal_moving"
TARGET_MOVING_MODES = (TARGET_MODE_RANDOM_MOVING, TARGET_MODE_TERMINAL_MOVING)
TARGET_MIN_SPEED_MPS = 0.4
TARGET_MAX_SPEED_MPS = 1.0
TARGET_DIRECTION_INTERVAL_S = 0.4


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


def dart_quat_to_angles(quaternion):
    """从 MuJoCo 自由关节的四元数恢复与发射面板一致的 YAW/PITCH/ROLL。

    发射时的约定为 Q = Rz(yaw) @ Ry(-pitch) @ Rx(roll) @ Ry(90°)。
    右乘 Ry(-90°) 移除 STL 长轴沿 Z 的固定校正后，再按 ZYX 顺序解算。
    单位为度；欧拉角仅用于展示，实际姿态仍使用 MuJoCo 四元数。
    """
    q = quat_mul(quaternion, axis_quat("y", -90.0))
    w, x, y, z = (float(v) for v in q)
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n == 0:
        raise ValueError("飞镖姿态四元数为零")
    w, x, y, z = (v / n for v in (w, x, y, z))

    yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    # ZYX 分解中的 Y 轴正向旋转与面板的正俯仰相反。
    y_sin = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = -math.asin(y_sin)
    roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    return (math.degrees(yaw), math.degrees(pitch), math.degrees(roll))


def dart_body_angular_velocity(omega_local):
    """将 MuJoCo STL 本体角速度换算到以 +X 为机头的飞行机体系。

    STL 的机头沿本地 +Z，发射姿态额外右乘 Ry(90°)，因此
    [omega_forward_X, omega_Y, omega_Z] = [omega_STL_Z, omega_STL_Y, -omega_STL_X]。
    返回 rad/s；UI 再转为 °/s。三个值是机体系角速度，不等同于欧拉角导数。
    """
    return (float(omega_local[2]), float(omega_local[1]), -float(omega_local[0]))


# ======================== 摄像机 ========================


def camera_init(
    viewer,
    lookat=(25.58828, 7.5, 1.32034),
    position=(14.0, 7.5, 4.5)
):
    """
    从场地中央上方，看向对面基地镖靶。
    """
    dx = lookat[0] - position[0]
    dy = lookat[1] - position[1]
    dz = lookat[2] - position[2]

    distance = math.sqrt(dx**2 + dy**2 + dz**2)
    azimuth = math.degrees(math.atan2(dy, dx))
    elevation = math.degrees(
        math.atan2(dz, math.hypot(dx, dy))
    )

    with viewer.lock():
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        viewer.cam.lookat[:] = lookat
        viewer.cam.distance = distance
        viewer.cam.azimuth = azimuth
        viewer.cam.elevation = elevation

    viewer.sync()


# ======================== 镖靶目标运动轨迹 ========================

class TargetMotionPlan:
    """单次仿真的随机轨迹；速度仅在创建时抽取一次。

    位置单位为 mm，速度单位为 m/s；仿真时间必须单调递增。
    对 0.4 秒换向时刻进行分段积分，对边界进行反射，因此不会越过 ±280mm。
    """

    def __init__(self, mode, rng):
        if mode not in TARGET_MOVING_MODES:
            raise ValueError(f"不支持的自动镖靶模式：{mode}")
        self.mode = mode
        self.rng = rng
        self.offset_mm = rng.uniform(BASE_TARGET_MIN_OFFSET_MM, BASE_TARGET_MAX_OFFSET_MM)
        self.speed_mps = rng.uniform(TARGET_MIN_SPEED_MPS, TARGET_MAX_SPEED_MPS)
        self.direction = rng.choice((-1, 1))
        self.start_time = 0.0 if mode == TARGET_MODE_RANDOM_MOVING else 1.2
        self.stop_time = 0.6 if mode == TARGET_MODE_RANDOM_MOVING else 1.8
        self.next_change_time = self.start_time + TARGET_DIRECTION_INTERVAL_S
        self.last_time = 0.0

    def _move_for(self, duration):
        """移动 duration 秒；碰到边界按镜面反射的方式立即反向。"""
        if duration <= 0:
            return
        candidate = self.offset_mm + self.direction * self.speed_mps * 1000.0 * duration
        low, high = BASE_TARGET_MIN_OFFSET_MM, BASE_TARGET_MAX_OFFSET_MM
        while candidate > high or candidate < low:
            if candidate > high:
                candidate = 2.0 * high - candidate
                self.direction = -1
            elif candidate < low:
                candidate = 2.0 * low - candidate
                self.direction = 1
        self.offset_mm = max(low, min(high, candidate))
        # 恰好到达边界时，下一小步也必须向内运动。
        if abs(self.offset_mm - high) < 1e-9 and self.direction > 0:
            self.direction = -1
        elif abs(self.offset_mm - low) < 1e-9 and self.direction < 0:
            self.direction = 1

    def advance_to(self, sim_time):
        """推进至指定仿真时刻，返回靶心相对初始位置的真实位移（mm）。"""
        sim_time = float(sim_time)
        if sim_time + 1e-10 < self.last_time:
            raise ValueError("仿真时间回退后应重新创建 TargetMotionPlan")

        left = max(self.last_time, self.start_time)
        right = min(sim_time, self.stop_time)
        while right - left > 1e-10:
            end = min(right, self.next_change_time)
            if end > left:
                self._move_for(end - left)
                left = end
            # 每次独立掷硬币，50% 概率维持原方向。
            if left >= self.next_change_time - 1e-10:
                if left < self.stop_time - 1e-10:
                    self.direction = self.rng.choice((self.direction, -self.direction))
                self.next_change_time += TARGET_DIRECTION_INTERVAL_S

        self.last_time = max(self.last_time, sim_time)
        return self.offset_mm




class _DarkSlider(tk.Canvas):
    """深色扁平滑条：使用原来的 Tk 变量和回调，保留键盘微调。"""

    def __init__(self, parent, variable, minimum, maximum, resolution,
                 command=None, reverse=False, **kwargs):
        self.variable = variable
        self.minimum = float(minimum)
        self.maximum = float(maximum)
        self.resolution = float(resolution)
        self.command = command
        self.reverse = bool(reverse)
        super().__init__(
            parent, height=30, background="#1B222B", bd=0,
            highlightthickness=0, cursor="hand2", takefocus=1, **kwargs
        )
        self.bind("<Configure>", self._redraw)
        self.bind("<Button-1>", self._on_pointer)
        self.bind("<B1-Motion>", self._on_pointer)
        self.bind("<Left>", lambda _e: self._step(-1))
        self.bind("<Right>", lambda _e: self._step(1))
        self.bind("<Home>", lambda _e: self._set_from_user(
            self.maximum if self.reverse else self.minimum
        ))
        self.bind("<End>", lambda _e: self._set_from_user(
            self.minimum if self.reverse else self.maximum
        ))
        self.variable.trace_add("write", self._on_variable_changed)

    def _on_variable_changed(self, *_):
        self._redraw()

    def _set_from_user(self, value):
        value = max(self.minimum, min(self.maximum, float(value)))
        value = self.minimum + round((value - self.minimum) / self.resolution) * self.resolution
        value = max(self.minimum, min(self.maximum, value))
        if abs(value - float(self.variable.get())) < self.resolution * 1e-4:
            return
        self.variable.set(value)
        if self.command is not None:
            self.command(str(value))

    def _on_pointer(self, event):
        self.focus_set()
        width = max(40, self.winfo_width())
        start, end = 12.0, width - 12.0
        proportion = max(0.0, min(1.0, (event.x - start) / (end - start)))
        if self.reverse:
            proportion = 1.0 - proportion
        self._set_from_user(self.minimum + proportion * (self.maximum - self.minimum))

    def _step(self, direction):
        # 左右方向按滑块的视觉移动方向解释。
        step = -direction if self.reverse else direction
        self._set_from_user(float(self.variable.get()) + step * self.resolution)

    def _redraw(self, _event=None):
        self.delete("all")
        width = max(40, self.winfo_width())
        start, end, cy = 12.0, width - 12.0, 15.0
        value = max(self.minimum, min(self.maximum, float(self.variable.get())))
        fraction = (value - self.minimum) / (self.maximum - self.minimum)
        if self.reverse:
            fraction = 1.0 - fraction
        x = start + fraction * (end - start)
        self.create_line(start, cy, end, cy, fill="#35414E", width=6,
                         capstyle=tk.ROUND)
        self.create_line(start, cy, x, cy, fill="#64B9EF", width=6,
                         capstyle=tk.ROUND)
        self.create_oval(x-8, cy-8, x+8, cy+8, fill="#E5F4FC",
                         outline="#65B9F0", width=2)


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

        # xipos 是飞镖的世界坐标系质心（不是靶心距离）。
        self.dart_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "dart_preview"
        )
        if self.dart_body_id < 0:
            raise RuntimeError("模型中找不到飞镖刚体 dart_preview")

        # 镖靶在 XML 中已有 mocap body，直接更新 mocap_pos 即可让外观和碰撞体同步移动。
        self.base_target_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, BASE_TARGET_BODY_NAME
        )
        if self.base_target_body_id < 0:
            raise RuntimeError(f"模型中找不到活动镖靶 {BASE_TARGET_BODY_NAME}")
        self.base_target_mocap_id = int(model.body_mocapid[self.base_target_body_id])
        if self.base_target_mocap_id < 0:
            raise RuntimeError(f"{BASE_TARGET_BODY_NAME} 不是 mocap body，无法控制平移")
        self.base_target_mocap_origin = data.mocap_pos[self.base_target_mocap_id].copy()
        self.base_target_offset_mm = 0.0
        self._updating_target_controls = False
        self.target_rng = random.Random()
        self.target_motion = None

        # 本机进程间通信：三个独立 Python 文件可在各自的终端中启动/退出。
        # 传感走 TCP；主动与被动走各自的 UDP。没有连接时输入默认为零。
        self.bridge = LocalDartBridge()
        self.interfaces = DartInterfaces(
            model=model, data=data,
            body_id=self.dart_body_id,
            qpos_adr=qpos_adr, qvel_adr=qvel_adr,
            image_camera="dart_fpv", image_width=640, image_height=480,
            image_fps=30.0, publish_interval_s=INTERFACE_PERIOD_S,
            auto_capture_image=False,  # 无传感接收客户端时完全不渲染离屏 FPV
        )
        self._ipc_next_wall = time.perf_counter()
        self._ipc_frame_id = 0

        # 状态面板及历史数据；接口的六维力由独立模块管理。
        self.state_history = []
        self.last_state_sample_time = float("-inf")
        self.last_state_display_time = float("-inf")

        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()
        self.closing = False

        self.root = tk.Tk()
        self.root.title("RoboMaster")
        self.root.geometry("1230x760")
        self.root.minsize(1030, 680)
        self.target_mode_var = tk.StringVar(master=self.root, value=TARGET_MODE_FIXED)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # 启动时应用默认参数，但保持暂停。
        self._apply_initial_state()
        self.root.after(8, self._simulation_tick)

    def _build_ui(self):
        """与右侧监测区统一的暗色面板；只调整呈现，不更改仿真控制逻辑。"""
        background = "#111417"
        card_bg = "#1B222B"
        field_bg = "#10161D"
        border = "#38434E"
        foreground = "#F2F4F6"
        muted = "#A7B0BB"
        accent = "#70C3F4"
        btn_bg = "#27313B"
        btn_hover = "#344353"

        self.root.configure(bg=background)
        style = ttk.Style(self.root)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass

        outer = tk.Frame(self.root, bg=background)
        outer.pack(fill="both", expand=True, padx=12, pady=12)

        main_frame = tk.Frame(outer, bg=background, width=455)
        main_frame.pack(side="left", fill="y", padx=(0, 12))
        main_frame.pack_propagate(False)

        monitor_frame = tk.Frame(
            outer, bg=background, highlightbackground="#414951", highlightthickness=1
        )
        monitor_frame.pack(side="left", fill="both", expand=True)

        def label(parent, text=None, variable=None, color=muted, size=10,
                  weight="normal", **kwargs):
            options = dict(
                bg=parent.cget("bg"), fg=color,
                font=("Microsoft YaHei UI", size, weight),
                anchor="w", **kwargs
            )
            if variable is not None:
                options["textvariable"] = variable
            else:
                options["text"] = text
            return tk.Label(parent, **options)

        def section_card(title):
            border_frame = tk.Frame(main_frame, bg=border)
            border_frame.pack(fill="x", pady=(0, 7))
            surface = tk.Frame(border_frame, bg=card_bg)
            surface.pack(fill="both", expand=True, padx=1, pady=1)
            label(surface, title, color=foreground, size=11, weight="bold").pack(
                anchor="w", padx=14, pady=(9, 6)
            )
            content = tk.Frame(surface, bg=card_bg)
            content.pack(fill="x", padx=14, pady=(0, 9))
            return content

        def dark_entry(parent, variable, width=11, justify="center"):
            return tk.Entry(
                parent, textvariable=variable, width=width, justify=justify,
                bg=field_bg, fg=foreground, insertbackground=foreground,
                highlightthickness=1, highlightbackground=border,
                highlightcolor=accent, bd=5, relief="flat",
                font=("Consolas", 11)
            )

        def dark_button(parent, text, command, primary=False, compact=False):
            base = "#286A97" if primary else btn_bg
            active = "#347EAD" if primary else btn_hover
            button = tk.Button(
                parent, text=text, command=command,
                bg=base, fg=foreground,
                activebackground=active, activeforeground="#FFFFFF",
                relief="flat", bd=0, highlightthickness=0,
                font=("Microsoft YaHei UI", 10),
                padx=6 if compact else 10, pady=5 if compact else 9,
                cursor="hand2"
            )
            button.bind("<Enter>", lambda _e: button.configure(bg=active))
            button.bind("<Leave>", lambda _e: button.configure(bg=base))
            return button

        label(main_frame, "Control Board", color=foreground, size=17,
              weight="bold").pack(anchor="w", pady=(4, 10))

        # ---------- 1. 发射角度 ----------
        angle_frame = section_card("1. 发射角度 (°)")
        self.yaw_var = tk.StringVar(value=str(DEFAULT_YAW))
        self.pitch_var = tk.StringVar(value=str(DEFAULT_PITCH))
        self.roll_var = tk.StringVar(value=str(DEFAULT_ROLL))
        for column, (name, variable) in enumerate((
            ("偏航 YAW", self.yaw_var),
            ("俯仰 PITCH", self.pitch_var),
            ("滚转 ROLL", self.roll_var),
        )):
            angle_frame.grid_columnconfigure(column, weight=1, uniform="angle")
            cell = tk.Frame(angle_frame, bg=card_bg)
            cell.grid(row=0, column=column, sticky="ew", padx=(0, 8) if column < 2 else 0)
            label(cell, name, size=9).pack(fill="x", pady=(0, 6))
            dark_entry(cell, variable).pack(fill="x")

        # ---------- 2. 发射速度 ----------
        speed_frame = section_card("2. 发射速度 (m/s)")
        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED)
        self.speed_value_text = tk.StringVar(value=f"{DEFAULT_SPEED:.3f}")
        speed_top = tk.Frame(speed_frame, bg=card_bg)
        speed_top.pack(fill="x", pady=(0, 5))
        label(speed_top, "初速度大小").pack(side="left")
        label(speed_top, "m/s").pack(side="right", padx=(6, 0))
        self.speed_entry = dark_entry(speed_top, self.speed_value_text, width=10, justify="right")
        self.speed_entry.pack(side="right")
        self.speed_entry.bind("<Return>", self._on_speed_entry)
        self.speed_entry.bind("<FocusOut>", self._on_speed_entry)
        self.speed_scale = _DarkSlider(
            speed_frame, variable=self.speed_var, minimum=MIN_SPEED,
            maximum=MAX_SPEED, resolution=SPEED_RESOLUTION
        )
        self.speed_scale.pack(fill="x", pady=(0, 5))
        self.speed_var.trace_add("write", self._on_speed_slider_changed)

        # ---------- 3. 基地镖靶位置 ----------
        target_frame = section_card("3. 基地镖靶位置")
        self.target_offset_var = tk.DoubleVar(master=self.root, value=0.0)
        self.target_offset_entry_var = tk.StringVar(master=self.root, value="0")
        target_top = tk.Frame(target_frame, bg=card_bg)
        target_top.pack(fill="x", pady=(0, 5))
        label(target_top, "相对初始位置的偏移量").pack(side="left")
        label(target_top, "mm").pack(side="right", padx=(6, 0))
        self.target_offset_entry = dark_entry(
            target_top, self.target_offset_entry_var, width=9, justify="right"
        )
        self.target_offset_entry.pack(side="right")
        self.target_offset_entry.bind("<Return>", self._on_target_offset_entry)
        self.target_offset_entry.bind("<FocusOut>", self._on_target_offset_entry)
        self.target_offset_scale = _DarkSlider(
            target_frame, variable=self.target_offset_var,
            minimum=BASE_TARGET_MIN_OFFSET_MM,
            maximum=BASE_TARGET_MAX_OFFSET_MM, resolution=1,
            command=self._on_target_offset_slider, reverse=True,
        )
        self.target_offset_scale.pack(fill="x", pady=(0, 3))
        target_range = tk.Frame(target_frame, bg=card_bg)
        target_range.pack(fill="x")
        label(target_range, "+280 mm (左)", size=9).pack(side="left")
        label(target_range, "-280 mm (右)", size=9).pack(side="right")
        label(target_range, "0 (中)", size=9).pack(expand=True)

        # ---------- 目标模式 ----------
        mode_frame = section_card("目标模式")
        for mode_value, mode_name in (
            (TARGET_MODE_FIXED, "固定目标"),
            (TARGET_MODE_RANDOM_FIXED, "随机固定"),
            (TARGET_MODE_RANDOM_MOVING, "随机移动"),
            (TARGET_MODE_TERMINAL_MOVING, "末端移动"),
        ):
            mode_row = tk.Frame(mode_frame, bg=card_bg)
            mode_row.pack(fill="x", pady=1)
            tk.Radiobutton(
                mode_row, text=mode_name, variable=self.target_mode_var,
                value=mode_value, command=self._on_target_mode_changed,
                bg=card_bg, fg=foreground, selectcolor=field_bg,
                activebackground=card_bg, activeforeground=foreground,
                font=("Microsoft YaHei UI", 10), anchor="w", padx=0,
                bd=0, highlightthickness=0, cursor="hand2"
            ).pack(side="left")
            if mode_value == TARGET_MODE_RANDOM_FIXED:
                dark_button(
                    mode_row, "⟳", self._randomize_fixed_target, compact=True
                ).pack(side="left", padx=(12, 0))

        # ---------- 操作按钮 ----------
        button_frame = tk.Frame(main_frame, bg=background)
        button_frame.pack(fill="x", pady=(1, 0))
        for column in range(3):
            button_frame.grid_columnconfigure(column, weight=1, uniform="actions")
        self.apply_button = dark_button(button_frame, "应用参数并重置", self._apply_initial_state)
        self.apply_button.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.run_button = dark_button(button_frame, "运行 Run", self._toggle_run, primary=True)
        self.run_button.grid(row=0, column=1, sticky="ew", padx=(0, 5))
        self.reset_button = dark_button(button_frame, "重置 Reset", self._apply_initial_state)
        self.reset_button.grid(row=0, column=2, sticky="ew")

        # 逻辑仍用状态字符串，但不在界面上显示。
        self.status_text = tk.StringVar(value="状态：模型已加载，等待设置发射参数。")
        self._build_state_monitor(monitor_frame)

    # ======================== 镖靶平移与模式控制 ========================

    def _write_target_mocap_offset(self):
        """调用方必须持有 viewer.lock()；不改 X、Z，不叠加历史偏移。"""
        position = self.base_target_mocap_origin.copy()
        position[1] += self.base_target_offset_mm / 1000.0
        self.data.mocap_pos[self.base_target_mocap_id] = position

    def _sync_target_controls(self, force_entry=False):
        """同步滑条和输入框，显示 1mm 精度；保留底层浮点位移精度。"""
        display_mm = int(round(self.base_target_offset_mm))
        self._updating_target_controls = True
        try:
            if int(round(self.target_offset_var.get())) != display_mm:
                self.target_offset_var.set(display_mm)
            # 自动运动时，不覆盖用户正在输入中的数字。
            if force_entry or self.root.focus_get() != self.target_offset_entry:
                text = str(display_mm)
                if self.target_offset_entry_var.get() != text:
                    self.target_offset_entry_var.set(text)
        finally:
            self._updating_target_controls = False

    def _apply_target_offset_mm(self, offset_mm, manual=False):
        """手动设置可切换到随机固定；仅改变对面基地镖靶。"""
        offset_mm = int(offset_mm)
        if not BASE_TARGET_MIN_OFFSET_MM <= offset_mm <= BASE_TARGET_MAX_OFFSET_MM:
            raise ValueError(
                f"基地镖靶的横向位移必须在 "
                f"{BASE_TARGET_MIN_OFFSET_MM}～{BASE_TARGET_MAX_OFFSET_MM} mm 范围内。"
            )

        if manual:
            mode = self.target_mode_var.get()
            # 固定目标的非零人工位移 -> 随机固定；自动运动时人工接管也转为随机固定。
            if (mode == TARGET_MODE_FIXED and offset_mm != 0) or mode in TARGET_MOVING_MODES:
                self.target_mode_var.set(TARGET_MODE_RANDOM_FIXED)
                self.target_motion = None

        changed = abs(self.base_target_offset_mm - offset_mm) > 1e-9
        self.base_target_offset_mm = float(offset_mm)
        self._sync_target_controls(force_entry=True)
        if changed:
            with self.viewer.lock():
                self._write_target_mocap_offset()
                mujoco.mj_forward(self.model, self.data)
            # 这里只改变镖靶，飞镖的状态无需清空或重新采样。
            self.viewer.sync()

    def _on_target_offset_slider(self, value):
        """只有用户真正改变滑条值，才接管自动运动模式。"""
        if self._updating_target_controls:
            return
        chosen_mm = int(round(float(value)))
        # 忽略代码刷新滑条导致的延迟回调。
        if chosen_mm == int(round(self.base_target_offset_mm)):
            return
        self._apply_target_offset_mm(chosen_mm, manual=True)

    def _on_target_offset_entry(self, event=None):
        """按 Enter 或失去焦点时生效；非整数、越界输入恢复为当前值。"""
        raw = self.target_offset_entry_var.get().strip()
        try:
            value = float(raw)
            if not math.isfinite(value) or not value.is_integer():
                raise ValueError("请输入整数毫米数值。")
            self._apply_target_offset_mm(int(value), manual=True)
        except (ValueError, tk.TclError) as error:
            self._sync_target_controls(force_entry=True)
            if event is not None and getattr(event, "keysym", None) == "Return":
                messagebox.showerror("镖靶位置无效", str(error), parent=self.root)
        return "break"

    def _on_target_mode_changed(self):
        """根据单选模式调整初始靶位。自动模式需要从 t=0 开始新一局。"""
        mode = self.target_mode_var.get()
        self.target_motion = None
        if mode == TARGET_MODE_FIXED:
            self._apply_target_offset_mm(0)
        elif mode == TARGET_MODE_RANDOM_FIXED:
            # 保留当前手动设定的坐标，只有点击 ⟳ 才生成随机位置。
            pass
        elif mode in TARGET_MOVING_MODES:
            # 选择自动模式时从中心待机；开始运行的瞬间跳到随机位置。
            if self.running or self.data.time > 1e-9:
                self._apply_initial_state()
            else:
                self._apply_target_offset_mm(0)

    def _randomize_fixed_target(self):
        """只在点击随机按钮时生成随机固定位置，命中后不会再次随机。"""
        self.target_mode_var.set(TARGET_MODE_RANDOM_FIXED)
        self.target_motion = None
        new_offset = self.target_rng.randint(BASE_TARGET_MIN_OFFSET_MM, BASE_TARGET_MAX_OFFSET_MM)
        if new_offset == int(round(self.base_target_offset_mm)):
            # 避免点击后极小概率恰好抽到原值、视觉上看起来没变化。
            new_offset = BASE_TARGET_MIN_OFFSET_MM if new_offset != BASE_TARGET_MIN_OFFSET_MM else BASE_TARGET_MAX_OFFSET_MM
        self._apply_target_offset_mm(new_offset)

    def _prepare_target_motion_for_run(self):
        """一局只随机一次起始位置、方向和速度；暂停/继续不重新抽取。"""
        mode = self.target_mode_var.get()
        if (mode not in TARGET_MOVING_MODES or
                self.target_motion is not None or self.data.time > 1e-9):
            return
        self.target_motion = TargetMotionPlan(mode, self.target_rng)
        with self.viewer.lock():
            self.base_target_offset_mm = self.target_motion.offset_mm
            self._write_target_mocap_offset()
            mujoco.mj_forward(self.model, self.data)
        self._sync_target_controls(force_entry=True)
        self.viewer.sync()
        print(
            f"[镖靶] {mode}: 初始偏移="
            f"{self.target_motion.offset_mm:+.1f} mm, "
            f"速度={self.target_motion.speed_mps:.3f} m/s, "
            f"初始方向={'+' if self.target_motion.direction > 0 else '-'}Y"
        )

    # ======================== 只读状态监测 + 历史记录 ========================

    def _build_state_monitor(self, parent):
        """显示 12 个独立状态分量；不添加曲线，也不修改物理过程。"""
        background = "#111417"
        fg = "#F2F4F6"
        muted = "#9EA7B1"

        top = tk.Frame(parent, bg=background)
        top.pack(fill="x", padx=22, pady=(18, 4))
        tk.Label(
            top, text="4. 飞镖实时状态监测", bg=background, fg=fg,
            font=("Microsoft YaHei UI", 15, "bold")
        ).pack(side="left")
        tk.Button(
            top, text="导出历史 CSV", command=self._export_state_history,
            bg="#282F36", fg=fg, activebackground="#36414B",
            activeforeground=fg, relief="flat", padx=10, pady=4,
            font=("Microsoft YaHei UI", 9), cursor="hand2"
        ).pack(side="right")

        header = tk.Frame(parent, bg=background)
        header.pack(fill="x", padx=24, pady=(18, 18))
        tk.Label(
            header, text="仿真时间", bg=background, fg=muted,
            font=("Microsoft YaHei UI", 11)
        ).pack(side="left")
        self.state_time_text = tk.StringVar(master=self.root, value="t = 0.000 s")
        tk.Label(
            header, textvariable=self.state_time_text, bg=background, fg=fg,
            font=("Consolas", 16)
        ).pack(side="right")

        columns = tk.Frame(parent, bg=background)
        columns.pack(fill="both", expand=True, padx=22, pady=(4, 0))
        columns.grid_columnconfigure(0, weight=1, uniform="state_columns")
        columns.grid_columnconfigure(1, weight=1, uniform="state_columns")
        columns.grid_rowconfigure(0, weight=1)
        columns.grid_rowconfigure(1, weight=1)
        self.state_value_vars = {}

        def add_section(row, column, heading, unit, fields):
            section = tk.Frame(columns, bg=background)
            section.grid(row=row, column=column, sticky="nsew", padx=12, pady=(12, 22))
            section.grid_columnconfigure(1, weight=1)
            tk.Label(
                section, text=heading, bg=background, fg=fg,
                font=("Microsoft YaHei UI", 13)
            ).grid(row=0, column=0, sticky="w", pady=(0, 16))
            tk.Label(
                section, text=unit, bg=background, fg=muted,
                font=("Microsoft YaHei UI", 10)
            ).grid(row=0, column=1, sticky="e", pady=(0, 16))
            for index, (key, name) in enumerate(fields, start=1):
                tk.Label(
                    section, text=name, bg=background, fg=muted,
                    font=("Microsoft YaHei UI", 12)
                ).grid(row=index, column=0, sticky="w", pady=9)
                variable = tk.StringVar(master=self.root, value="--")
                self.state_value_vars[key] = variable
                tk.Label(
                    section, textvariable=variable, bg=background, fg=fg,
                    font=("Consolas", 14), anchor="e"
                ).grid(row=index, column=1, sticky="e", pady=9)

        add_section(0, 0, "质心位置", "m", (("x", "X"), ("y", "Y"), ("z", "Z")))
        add_section(0, 1, "线速度", "m/s", (("vx", "Vx"), ("vy", "Vy"), ("vz", "Vz")))
        add_section(1, 0, "姿态角", "°", (("yaw", "偏航"), ("pitch", "俯仰"), ("roll", "滚转")))
        add_section(1, 1, "机体角速度", "°/s", (("wx", "ωx"), ("wy", "ωy"), ("wz", "ωz")))

    def _read_dart_state_locked(self):
        """读取 MuJoCo 状态。调用方必须持有 viewer.lock()。"""
        qadr, vadr = self.qpos_adr, self.qvel_adr
        quaternion = self.data.qpos[qadr+3:qadr+7].copy()
        velocity = self.data.qvel[vadr:vadr+3].copy()
        omega_local = self.data.qvel[vadr+3:vadr+6].copy()
        center = self.data.xipos[self.dart_body_id].copy()
        yaw, pitch, roll = dart_quat_to_angles(quaternion)
        wx, wy, wz = dart_body_angular_velocity(omega_local)
        snapshot = {
            "time_s": float(self.data.time),
            "x_m": float(center[0]), "y_m": float(center[1]), "z_m": float(center[2]),
            "vx_mps": float(velocity[0]), "vy_mps": float(velocity[1]),
            "vz_mps": float(velocity[2]),
            "yaw_deg": yaw, "pitch_deg": pitch, "roll_deg": roll,
            "wx_deg_s": math.degrees(wx), "wy_deg_s": math.degrees(wy),
            "wz_deg_s": math.degrees(wz),
            # 原生四元数额外写入 CSV，避免以后由欧拉角重建时丢失信息。
            "qw": float(quaternion[0]), "qx": float(quaternion[1]),
            "qy": float(quaternion[2]), "qz": float(quaternion[3]),
        }
        return snapshot

    def _record_dart_state(self, snapshot):
        """历史数据按仿真时间采样；重置/时间倒退时重新开始记录。"""
        now = snapshot["time_s"]
        if self.state_history and now < self.state_history[-1]["time_s"] - 1e-9:
            self.state_history.clear()
            self.last_state_sample_time = float("-inf")
        if now - self.last_state_sample_time >= STATE_HISTORY_SAMPLE_INTERVAL_S - 1e-9:
            self.state_history.append(snapshot.copy())
            self.last_state_sample_time = now

    def _display_dart_state(self, snapshot, force=False):
        """只更新 Tkinter 文本，限速以避免拖慢 0.2ms 物理步进。"""
        now = time.perf_counter()
        if not force and now - self.last_state_display_time < STATE_MONITOR_REFRESH_INTERVAL_S:
            return
        self.last_state_display_time = now
        self.state_time_text.set(f"t = {snapshot['time_s']:.3f} s")
        mappings = {
            "x": ("x_m", ".3f"), "y": ("y_m", ".3f"), "z": ("z_m", ".3f"),
            "vx": ("vx_mps", "+.3f"), "vy": ("vy_mps", "+.3f"),
            "vz": ("vz_mps", "+.3f"),
            "yaw": ("yaw_deg", "+.2f"), "pitch": ("pitch_deg", "+.2f"),
            "roll": ("roll_deg", "+.2f"),
            "wx": ("wx_deg_s", "+.2f"), "wy": ("wy_deg_s", "+.2f"),
            "wz": ("wz_deg_s", "+.2f"),
        }
        for key, (source_key, fmt) in mappings.items():
            self.state_value_vars[key].set(format(snapshot[source_key], fmt))

    def _refresh_state_monitor(self, force=False, record=True):
        """从当前 MuJoCo 状态读取快照；仅在必要时更新 GUI。"""
        with self.viewer.lock():
            snapshot = self._read_dart_state_locked()
        if record:
            self._record_dart_state(snapshot)
        self._display_dart_state(snapshot, force=force)

    def _export_state_history(self):
        """将当前局保存的 12 个独立状态量及原生姿态四元数导出为 CSV。"""
        if not self.state_history:
            messagebox.showinfo("没有历史数据", "当前还没有可导出的仿真状态。", parent=self.root)
            return
        filepath = filedialog.asksaveasfilename(
            parent=self.root, title="导出飞镖状态历史", defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv")], initialfile="dart_state_history.csv"
        )
        if not filepath:
            return
        fieldnames = (
            "time_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
            "yaw_deg", "pitch_deg", "roll_deg", "wx_deg_s", "wy_deg_s",
            "wz_deg_s", "qw", "qx", "qy", "qz"
        )
        try:
            with open(filepath, "w", newline="", encoding="utf-8-sig") as output:
                writer = csv.DictWriter(output, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.state_history)
        except OSError as error:
            messagebox.showerror("导出失败", str(error), parent=self.root)

    def _on_speed_slider_changed(self, *_):
        """拖动滑条时，将数值同步到速度输入框。"""
        self.speed_value_text.set(f"{self.speed_var.get():.3f}")

    def _on_speed_entry(self, event=None):
        """按 Enter 或离开输入框时应用速度，按滑条精度 0.1 m/s 对齐。"""
        try:
            speed = float(self.speed_value_text.get().strip())
            if not math.isfinite(speed):
                raise ValueError("初速度必须是有限数值。")
            if not MIN_SPEED <= speed <= MAX_SPEED:
                raise ValueError(f"初速度必须在 {MIN_SPEED:g}～{MAX_SPEED:g} m/s 范围内。")

            speed = round(speed / SPEED_RESOLUTION) * SPEED_RESOLUTION
            self.speed_var.set(min(MAX_SPEED, max(MIN_SPEED, speed)))
            self.speed_value_text.set(f"{self.speed_var.get():.3f}")
        except (ValueError, tk.TclError) as error:
            self.speed_value_text.set(f"{self.speed_var.get():.3f}")
            if event is not None and getattr(event, "keysym", None) == "Return":
                messagebox.showerror("速度设置错误", str(error), parent=self.root)
        return "break"

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
        self.target_motion = None
        if self.target_mode_var.get() in (TARGET_MODE_FIXED,) + TARGET_MOVING_MODES:
            self.base_target_offset_mm = 0.0

        with self.viewer.lock():
            mujoco.mj_resetData(self.model, self.data)
            self.data.qpos[:] = self.initial_qpos
            self.data.qvel[:] = 0.0
            # MuJoCo reset 会恢复 mocap 的初始位置，这里重设用户当前选中的位移。
            self._write_target_mocap_offset()

            # freejoint 的前三个速度分量为世界坐标系线速度。
            self.data.qvel[self.qvel_adr:self.qvel_adr + 3] = velocity

            mujoco.mj_forward(self.model, self.data)
            # 清除上一局残余的六维力、传感采样和过期通信指令。
            self.interfaces.reset()
            self.bridge.clear_inputs()
            self._ipc_next_wall = time.perf_counter()

        # 重置后记录 t=0 的真实状态，不带入上一局历史数据。
        self.state_history.clear()
        self.last_state_sample_time = float("-inf")
        self.last_state_display_time = float("-inf")
        self._refresh_state_monitor(force=True)
        self._sync_target_controls(force_entry=True)
        self.viewer.sync()
        self.run_button.configure(text="运行 Run")
        self.status_text.set(
            f"状态：参数已应用。速度={speed:.2f} m/s；"
            f"YAW={yaw:.2f}°，PITCH={pitch:.2f}°，ROLL={roll:.2f}°；"
            f"基地靶横移={int(round(self.base_target_offset_mm)):+d} mm。"
        )

    def _toggle_run(self):
        if not self.running:
            self._prepare_target_motion_for_run()
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
                target_moved = False
                with self.viewer.lock():
                    for _ in range(steps):
                        old_time = self.data.time
                        old_pos = self.data.qpos[
                            self.qpos_adr:self.qpos_adr + 3
                        ].copy()

                        # 每一个 0.2ms 物理步前更新目标，保证碰撞与靶面位置同步。
                        if self.target_motion is not None:
                            offset_mm = self.target_motion.advance_to(
                                old_time + timestep
                            )
                            if abs(offset_mm - self.base_target_offset_mm) > 1e-9:
                                self.base_target_offset_mm = offset_mm
                                self._write_target_mocap_offset()
                                target_moved = True

                        velocity_before = self.data.qvel[
                            self.qvel_adr:self.qvel_adr + 3
                        ].copy()
                        # 同一物理时刻：主动 Wrench + 被动 Wrench，
                        # 转到世界系施加在飞镖质心，重力仍由 MuJoCo 自己计算。
                        self.interfaces.before_step()
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

                            # 自动重置后重新回到该模式的等待状态。
                            self.target_motion = None
                            if self.target_mode_var.get() in (TARGET_MODE_FIXED,) + TARGET_MOVING_MODES:
                                self.base_target_offset_mm = 0.0
                            target_moved = True
                            self._write_target_mocap_offset()
                            mujoco.mj_forward(self.model, self.data)
                            self.interfaces.reset()
                            self.bridge.clear_inputs()
                            self._ipc_next_wall = time.perf_counter()
                            self.running = False
                            self.accumulator = 0.0
                            self.run_button.configure(text="运行 Run")
                            self.status_text.set(
                                "状态：检测到 MuJoCo 自动重置，仿真已暂停。"
                            )
                            break
                        # 读取刚完成的物理步，输出机体系线加速度和角速度。
                        # 此处用的 dt 是实际仿真时间增量，而非 GUI 帧间隔。
                        self.interfaces.after_step(
                            velocity_before, float(self.data.time - old_time)
                        )

                    # mj_step 后 qpos 已更新，计算当前质心位置供状态监测使用。
                    mujoco.mj_kinematics(self.model, self.data)

                if self.running:
                    self.accumulator = max(
                        0.0,
                        self.accumulator - steps * timestep,
                    )
                # 只按 GUI tick 更新滑条/数字，不在每个物理步操作 Tk 控件。
                if target_moved:
                    self._sync_target_controls()

        # 每隔 1/30 秒真实时间执行一次通信周期：
        # 同周期锁存主动/被动最新值；传感图像和数值在同一个 data 状态下采样。
        # GUI 及场地渲染可能拖慢实际帧率，因此这是软实时的 30Hz 目标。
        self._exchange_ipc_tick()

        # 即使仿真暂停，也可观察通过 viewer 修改后的实时状态。
        # Tk 界面每 0.1 秒最多更新一次，历史记录按仿真时间抽样。
        self._refresh_state_monitor()
        self.viewer.sync()
        self.root.after(8, self._simulation_tick)

    def _exchange_ipc_tick(self):
        """30Hz 控制/传感采样周期；TCP 发送在线程中进行，不阻塞物理步。"""
        now = time.perf_counter()
        if now < self._ipc_next_wall:
            return
        self._ipc_next_wall += INTERFACE_PERIOD_S
        if self._ipc_next_wall < now:
            self._ipc_next_wall = now + INTERFACE_PERIOD_S

        active, passive = self.bridge.poll_inputs()
        # 30Hz 锁存并在后续 0.2ms 物理步中保持，两个输入在同一时刻逐分量相加。
        self.interfaces.set_active_input(*active)
        self.interfaces.set_passive_input(*passive)
        # 两个发送端使用同一个 30Hz 时钟源，同一 tick 的响应在后续周期锁存。
        self.bridge.broadcast_tick(self._ipc_frame_id, float(self.data.time))
        self._ipc_frame_id += 1

        if not self.bridge.sensor_connected:
            return

        with self.viewer.lock():
            # 捕获当前 MuJoCo data 的 dart_fpv 真实画面，而不是打印形状代替画面。
            # 必须与此时刻的加速度、角速度一起构建同一个传感消息。
            self.interfaces.capture_image_if_due(force=True)
            sample = self.interfaces.read_sensor()
            frame = sample.image_rgb
            meta = {
                "frame_id": self._ipc_frame_id - 1,
                "time_s": sample.time_s,
                "image_time_s": sample.image_time_s,
                "coordinate_frame": "dart_flight_body",
                "acceleration_body_mps2": list(sample.acceleration_xyz_mps2),
                "angular_velocity_body_rad_s": list(sample.angular_velocity_xyz_rad_s),
                "acceleration_valid": sample.acceleration_valid,
                # 已在该次状态采样之前的物理步中实际施加的合力/力矩：
                "active_force_body": list(self.interfaces.last_active.as_tuple()),
                "passive_force_body": list(self.interfaces.last_passive.as_tuple()),
                "total_force_body": list(self.interfaces.last_total.as_tuple()),
                # 以下两路是刚刚锁存、将在下一个物理步起生效的控制量：
                "next_active_force_body": list(active),
                "next_passive_force_body": list(passive),
                "camera_error": self.interfaces.camera_error,
            }

        # 真正的 socket 发送在独立线程内，队列始终只保留最近一帧。
        self.bridge.publish_sensor(meta, frame)

    def _on_close(self):
        if self.closing:
            return

        self.closing = True
        try:
            try:
                self.bridge.close()
            finally:
                try:
                    self.interfaces.close()
                finally:
                    if self.viewer.is_running():
                        self.viewer.close()
        finally:
            self.root.destroy()

    def run(self):
        """启动控制面板事件循环。"""
        self.root.mainloop()
