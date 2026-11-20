# -*- coding: utf-8 -*-
import math
import random
import time
import tkinter as tk
from tkinter import messagebox, ttk
import mujoco
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# ======================== 面板初始参数 ========================

DEFAULT_YAW = -15.2          # 偏航角（°）
DEFAULT_PITCH = 27.0        # 俯仰角（°）
DEFAULT_ROLL = 0.0         # 滚转角（°）

DEFAULT_SPEED = 18.0       # 初始速度（m/s）
MIN_SPEED = 5.0
MAX_SPEED = 30.0
SPEED_RESOLUTION = 0.1

# 实时曲线：按仿真时间采样，按实际时间限制 GUI 重绘频率。
DISTANCE_SAMPLE_INTERVAL = 0.05  # 仿真时间 50ms / 点
DISTANCE_PLOT_INTERVAL = 0.10    # 实际时间 100ms / 次图形刷新

# 距离使用碰撞箱的实时世界坐标，基地靶平移后会自动更新。
DISTANCE_TARGETS = (("base", "基地", "dart_target_collision_1"),("outpost", "前哨站", "dart_target_collision_2"),)

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

        self.distance_history = {}
        self.last_distance_sample_time = float("-inf")
        self.last_distance_plot_time = float("-inf")

        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()
        self.closing = False

        self.root = tk.Tk()
        self.root.title("RoboMaster")
        self.root.geometry("1230x760")
        self.root.minsize(1030, 680)
        self.selected_target = tk.StringVar(master=self.root, value="base")
        self.target_mode_var = tk.StringVar(master=self.root, value=TARGET_MODE_FIXED)

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
            outer, text="4. 与目标中心的距离", padding=8
        )
        chart_frame.pack(side="left", fill="both", expand=True)

        ttk.Label(
            main_frame,
            text="Control Board",
            font=("Microsoft YaHei UI", 15, "bold"),
        ).pack(anchor="w", pady=(0, 5))

        # ---------- 角度设置 ----------
        angle_frame = ttk.LabelFrame(
            main_frame,
            text="1. 发射角度(°)",
            padding=10,
        )
        angle_frame.pack(fill="x", pady=5)

        self.yaw_var = tk.StringVar(value=str(DEFAULT_YAW))
        self.pitch_var = tk.StringVar(value=str(DEFAULT_PITCH))
        self.roll_var = tk.StringVar(value=str(DEFAULT_ROLL))

        angle_items = (
            ("偏航 YAW", self.yaw_var),
            ("俯仰 PITCH", self.pitch_var),
            ("滚转 ROLL", self.roll_var),
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

        # ---------- 初始速度 ----------
        speed_frame = ttk.LabelFrame(
            main_frame,
            text="2. 发射速度(m/s)",
            padding=10,
        )
        speed_frame.pack(fill="x", pady=5)

        self.speed_var = tk.DoubleVar(value=DEFAULT_SPEED)
        self.speed_value_text = tk.StringVar(value=f"{DEFAULT_SPEED:.3f}")
        self.velocity_text = tk.StringVar(value="")

        speed_top = ttk.Frame(speed_frame)
        speed_top.pack(fill="x")

        ttk.Label(speed_top, text="初速度大小").pack(side="left")
        ttk.Label(speed_top, text="m/s").pack(side="right", padx=(4, 0))
        self.speed_entry = ttk.Entry(
            speed_top,
            textvariable=self.speed_value_text,
            width=10,
            justify="right",
        )
        self.speed_entry.pack(side="right")
        self.speed_entry.bind("<Return>", self._on_speed_entry)
        self.speed_entry.bind("<FocusOut>", self._on_speed_entry)

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
            textvariable=self.velocity_text,
            foreground="#1769aa",
        ).pack(anchor="w")

        # 参数变化时实时更新速度分量预览；实际仿真状态需点击“应用参数并重置”。
        self.speed_var.trace_add("write", self._on_speed_slider_changed)
        self.yaw_var.trace_add("write", self._update_velocity_display)
        self.pitch_var.trace_add("write", self._update_velocity_display)
        self.roll_var.trace_add("write", self._update_velocity_display)
        self._update_velocity_display()

        # ---------- 对面基地靶的横向平移 ----------
        target_frame = ttk.LabelFrame(
            main_frame,
            text="3. 基地镖靶位置",
            padding=10,
        )
        target_frame.pack(fill="x", pady=5)

        self.target_offset_var = tk.DoubleVar(master=self.root, value=0.0)
        self.target_offset_entry_var = tk.StringVar(master=self.root, value="0")

        target_top = ttk.Frame(target_frame)
        target_top.pack(fill="x")
        ttk.Label(target_top, text="相对初始位置的偏移量").pack(side="left")
        ttk.Label(target_top, text="mm").pack(side="right", padx=(4, 0))
        self.target_offset_entry = ttk.Entry(
            target_top,
            textvariable=self.target_offset_entry_var,
            width=9,
            justify="right",
        )
        self.target_offset_entry.pack(side="right")
        self.target_offset_entry.bind("<Return>", self._on_target_offset_entry)
        self.target_offset_entry.bind("<FocusOut>", self._on_target_offset_entry)

        self.target_offset_scale = tk.Scale(
            target_frame,
            from_=BASE_TARGET_MIN_OFFSET_MM,
            to=BASE_TARGET_MAX_OFFSET_MM,
            variable=self.target_offset_var,
            command=self._on_target_offset_slider,
            resolution=1,
            orient="horizontal",
            showvalue=False,
            length=410,
            highlightthickness=0,
        )
        self.target_offset_scale.pack(fill="x")

        target_range = ttk.Frame(target_frame)
        target_range.pack(fill="x")
        ttk.Label(target_range, text="-280 mm(右)").pack(side="left")
        ttk.Label(target_range, text="0(中)").pack(side="left", expand=True)
        ttk.Label(target_range, text="+280 mm(左)").pack(side="right")

        # ---------- 目标模式（竖向单选），固定模式下不允许手动偏离中心 ----------
        mode_frame = ttk.LabelFrame(main_frame, text="目标模式", padding=(10, 6))
        mode_frame.pack(fill="x", pady=(4, 2))
        for mode_value, mode_name in (
            (TARGET_MODE_FIXED, "固定目标"),
            (TARGET_MODE_RANDOM_FIXED, "随机固定"),
            (TARGET_MODE_RANDOM_MOVING, "随机移动"),
            (TARGET_MODE_TERMINAL_MOVING, "末端移动"),
        ):
            mode_row = ttk.Frame(mode_frame)
            mode_row.pack(fill="x", anchor="w", pady=1)
            ttk.Radiobutton(
                mode_row,
                text=mode_name,
                variable=self.target_mode_var,
                value=mode_value,
                command=self._on_target_mode_changed,
            ).pack(side="left")
            if mode_value == TARGET_MODE_RANDOM_FIXED:
                ttk.Button(
                    mode_row, text="⟳", width=3,
                    command=self._randomize_fixed_target,
                ).pack(side="left", padx=(12, 0))

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
        # 保留 status_text 供内部逻辑使用，界面不再显示底部状态文字。
        self._build_distance_chart(chart_frame)

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
                self._sample_distances(force=True)
            self._update_distance_chart(force=True)
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
            # 仿真 t=0 的轨迹点用刚抽取的随机初始靶位，不保留先前中心位置。
            self._sample_distances(force=True)
        self._sync_target_controls(force_entry=True)
        self._update_distance_chart(force=True)
        self.viewer.sync()
        print(
            f"[镖靶] {mode}: 初始偏移="
            f"{self.target_motion.offset_mm:+.1f} mm, "
            f"速度={self.target_motion.speed_mps:.3f} m/s, "
            f"初始方向={'+' if self.target_motion.direction > 0 else '-'}Y"
        )

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
            # 暂停时可反复拖动镖靶：同一时刻覆盖末尾采样点，避免重复时间点。
            if history["time"] and abs(history["time"][-1] - sim_time) < 1e-9:
                history["dx"][-1] = dx
                history["dy"][-1] = dy
                history["dz"][-1] = dz
                history["distance"][-1] = distance
            else:
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

    def _on_speed_slider_changed(self, *_):
        """拖动滑条时，将数值同步到速度输入框并刷新速度分量。"""
        self.speed_value_text.set(f"{self.speed_var.get():.3f}")
        self._update_velocity_display()

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
            self._update_velocity_display()
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

    def _update_velocity_display(self, *_):
        """根据当前面板参数实时显示速度分量，不重复显示角度。"""
        try:
            speed, yaw, pitch, roll = self._read_parameters()
            vx, vy, vz = launch_velocity(speed, pitch, yaw)
            self.velocity_text.set(
                f"Vx = {vx:.3f}    Vy = {vy:.3f}    Vz = {vz:.3f} m/s"
            )
        except (tk.TclError, ValueError):
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

        # 每次应用参数或重置，都同步清空并重建实时距离曲线。
        self._reset_distance_history()
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
                # 只按 GUI tick 更新滑条/数字，不在每个物理步操作 Tk 控件。
                if target_moved:
                    self._sync_target_controls()

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
