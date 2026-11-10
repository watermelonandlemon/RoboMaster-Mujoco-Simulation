# -*- coding: utf-8 -*-
"""
MuJoCo 飞镖仿真：保留 view.py 中的场景、初始位置和姿态，
增加一个只用于设置初始速度的 Tkinter 控制面板。

依赖：
    mujoco
    lib.py
    scene_color.xml

以上文件及模型资源按原工程目录结构放置。
"""

from pathlib import Path
import math
import time
import tkinter as tk
from tkinter import ttk

import mujoco
import mujoco.viewer
import lib


# ============================================================
# 1. 场景路径：沿用原 view.py
# ============================================================

HERE = Path(__file__).resolve().parent
SCENE = HERE / "scene_color.xml"


# ============================================================
# 2. 飞镖姿态：沿用原 view.py，只在代码中设置，不放进面板
# ============================================================

YAW = 0.0       # 偏航角（度）
PITCH = 0.0     # 俯仰角（度）
ROLL = 0.0      # 绕飞镖自身轴滚转角（度）


# ============================================================
# 3. 控制面板唯一可调参数
# ============================================================

DEFAULT_SPEED =15.0       # 初始速度（m/s）
MIN_SPEED = 0.0
MAX_SPEED = 40.0
SPEED_RESOLUTION = 0.1

# 优先按此名称查找飞镖自由关节；如果场景中名称不同，会尝试自动识别。
DART_JOINT_NAME = "dart_free"
DART_BODY_NAME = "dart_preview"


def launch_velocity(speed, pitch_deg, yaw_deg):
    """按照 view.py 中的偏航角、俯仰角，计算世界坐标系下的初速度。"""
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    return [
        speed * math.cos(pitch) * math.cos(yaw),
        speed * math.cos(pitch) * math.sin(yaw),
        speed * math.sin(pitch),
    ]


def _joint_name(model, joint_id):
    return mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
    ) or ""


def _body_name(model, body_id):
    return mujoco.mj_id2name(
        model, mujoco.mjtObj.mjOBJ_BODY, body_id
    ) or ""


def find_dart_freejoint(model):
    """查找飞镖的 freejoint，避免把普通固定关节误当成发射关节。"""
    free_type = mujoco.mjtJoint.mjJNT_FREE
    joint_obj = mujoco.mjtObj.mjOBJ_JOINT
    body_obj = mujoco.mjtObj.mjOBJ_BODY

    # 1) 优先使用约定名称 dart_free。
    joint_id = mujoco.mj_name2id(model, joint_obj, DART_JOINT_NAME)
    if joint_id >= 0:
        if model.jnt_type[joint_id] != free_type:
            raise RuntimeError(
                f"关节 {DART_JOINT_NAME!r} 存在，但它不是 freejoint。"
            )
        return joint_id

    # 2) 尝试在名为 dart 的刚体上寻找 freejoint。
    body_id = mujoco.mj_name2id(model, body_obj, DART_BODY_NAME)
    if body_id >= 0:
        first = int(model.body_jntadr[body_id])
        count = int(model.body_jntnum[body_id])
        for candidate in range(first, first + count):
            if model.jnt_type[candidate] == free_type:
                return candidate

    # 3) 如果整个场景只有一个 freejoint，就将其视为飞镖。
    candidates = [
        j for j in range(model.njnt)
        if model.jnt_type[j] == free_type
    ]
    if len(candidates) == 1:
        return candidates[0]

    # 4) 多个 freejoint 时，按关节名或刚体名中的 dart 识别。
    dart_candidates = []
    for candidate in candidates:
        body_id = int(model.jnt_bodyid[candidate])
        if "dart" in (
            _joint_name(model, candidate) + " " + _body_name(model, body_id)
        ).lower():
            dart_candidates.append(candidate)

    if len(dart_candidates) == 1:
        return dart_candidates[0]

    raise RuntimeError(
        "在 scene_color.xml 中无法明确找到飞镖的自由关节。\n"
        "飞镖根 body 需要有 freejoint，例如：\n"
        '<freejoint name="dart_free"/>\n'
        "如果当前场景中的自由关节名称不同，请修改代码顶部的 "
        "DART_JOINT_NAME / DART_BODY_NAME。"
    )


def _changed(before, after, tol=1e-10):
    return any(
        abs(float(a) - float(b)) > tol
        for a, b in zip(before, after)
    )


def main():
    if not SCENE.is_file():
        raise FileNotFoundError(f"找不到场景文件：{SCENE}")

    print(f"正在加载场景：{SCENE}")
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    # 查找飞镖自由关节。
    joint_id = find_dart_freejoint(model)
    qpos_adr = int(model.jnt_qposadr[joint_id])
    qvel_adr = int(model.jnt_dofadr[joint_id])
    body_id = int(model.jnt_bodyid[joint_id])

    # 保留原 view.py 的姿态设置方式。
    # 如果 lib 直接改了 body_pos/body_quat，而没有同步 qpos0，
    # 则在下面把姿态同步到 freejoint 的初始状态。
    old_body_pos = model.body_pos[body_id].copy()
    old_body_quat = model.body_quat[body_id].copy()
    old_qpos0 = model.qpos0[qpos_adr:qpos_adr + 7].copy()

    lib.set_dart_attitude(
        model,
        yaw=YAW,
        pitch=PITCH,
        roll=ROLL,
    )

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    initial_qpos = data.qpos.copy()

    if (
        not _changed(old_qpos0[:3], model.qpos0[qpos_adr:qpos_adr + 3])
        and _changed(old_body_pos, model.body_pos[body_id])
    ):
        initial_qpos[qpos_adr:qpos_adr + 3] = model.body_pos[body_id]

    if (
        not _changed(old_qpos0[3:], model.qpos0[qpos_adr + 3:qpos_adr + 7])
        and _changed(old_body_quat, model.body_quat[body_id])
    ):
        initial_qpos[qpos_adr + 3:qpos_adr + 7] = model.body_quat[body_id]

    # 确保初始四元数合法。
    quat = initial_qpos[qpos_adr + 3:qpos_adr + 7]
    quat_norm = math.sqrt(sum(float(v) ** 2 for v in quat))
    if quat_norm < 1e-12:
        raise RuntimeError(
            "飞镖初始姿态四元数无效，请检查 lib.set_dart_attitude。"
        )
    initial_qpos[qpos_adr + 3:qpos_adr + 7] = quat / quat_norm

    mujoco.mj_forward(model, data)

    dart_name = _body_name(model, body_id)
    print(f"飞镖刚体：{dart_name}")
    print(f"发射位置沿用场景设置：{initial_qpos[qpos_adr:qpos_adr + 3]}")
    print(f"发射姿态沿用 view.py：YAW={YAW}°, PITCH={PITCH}°, ROLL={ROLL}°")

    # ========================================================
    # 4. Tkinter 控制面板：只有初速度滑块
    # ========================================================

    root = tk.Tk()
    root.title("RoboMaster 飞镖初始速度控制面板")
    root.geometry("540x390")
    root.minsize(500, 350)

    style = ttk.Style()
    try:
        style.theme_use("clam")
    except tk.TclError:
        pass

    main_frame = ttk.Frame(root, padding=10)
    main_frame.pack(fill="both", expand=True)

    ttk.Label(
        main_frame,
        text="Dart Initial Velocity / 飞镖初始速度",
        font=("Microsoft YaHei UI", 14, "bold"),
    ).pack(anchor="w", pady=(0, 8))

    ttk.Label(
        main_frame,
        text="设置飞镖初速度，点击“应用参数并重置”，再点击“运行”。",
        wraplength=500,
    ).pack(anchor="w", pady=(0, 10))

    speed_frame = ttk.LabelFrame(
        main_frame,
        text="1. 初始速度 Velocity",
        padding=8,
    )
    speed_frame.pack(fill="x", pady=5)

    speed_var = tk.DoubleVar(value=DEFAULT_SPEED)
    speed_value_text = tk.StringVar(value=f"{DEFAULT_SPEED:.3f}")
    velocity_text = tk.StringVar(value="")

    speed_top = ttk.Frame(speed_frame)
    speed_top.pack(fill="x")

    ttk.Label(speed_top, text="初速度大小 (m/s)").pack(side="left")
    ttk.Label(
        speed_top,
        textvariable=speed_value_text,
        width=12,
        anchor="e",
    ).pack(side="right")

    speed_scale = tk.Scale(
        speed_frame,
        variable=speed_var,
        from_=MIN_SPEED,
        to=MAX_SPEED,
        orient="horizontal",
        resolution=SPEED_RESOLUTION,
        showvalue=False,
        length=470,
        highlightthickness=0,
    )
    speed_scale.pack(fill="x")

    ttk.Label(
        speed_frame,
        text="速度方向自动沿用 view.py 中的 YAW 和 PITCH；发射位置、姿态不在面板中设置。",
        wraplength=480,
    ).pack(anchor="w", pady=(4, 3))

    ttk.Label(
        speed_frame,
        textvariable=velocity_text,
        foreground="#1769aa",
    ).pack(anchor="w")

    def update_velocity_display(*_):
        try:
            speed = float(speed_var.get())
            vx, vy, vz = launch_velocity(speed, PITCH, YAW)
            speed_value_text.set(f"{speed:.3f}")
            velocity_text.set(
                f"Vx = {vx:.3f}    Vy = {vy:.3f}    Vz = {vz:.3f} m/s"
            )
        except (tk.TclError, ValueError):
            speed_value_text.set("--")
            velocity_text.set("速度分量：请输入有效数值")

    speed_var.trace_add("write", update_velocity_display)
    update_velocity_display()

    # ========================================================
    # 5. 启动 MuJoCo 被动查看器
    # ========================================================

    try:
        viewer = mujoco.viewer.launch_passive(model, data)
    except Exception:
        root.destroy()
        raise

    running = False
    accumulator = 0.0
    last_wall_time = time.perf_counter()
    closing = False

    status_text = tk.StringVar(value="状态：模型已加载，等待设置初始速度。")

    # ========================================================
    # 6. 应用速度 / 重置 / 运行暂停
    # ========================================================

    def apply_initial_state():
        """恢复 view.py 的初始位置和姿态，只更新初速度。"""
        nonlocal running, accumulator, last_wall_time

        speed = float(speed_var.get())
        velocity = launch_velocity(speed, PITCH, YAW)

        running = False
        accumulator = 0.0
        last_wall_time = time.perf_counter()

        with viewer.lock():
            mujoco.mj_resetData(model, data)
            data.qpos[:] = initial_qpos
            data.qvel[:] = 0.0

            # freejoint 的前三个速度分量为世界坐标系线速度。
            data.qvel[qvel_adr:qvel_adr + 3] = velocity

            # 不额外赋予飞镖自转角速度。
            data.qvel[qvel_adr + 3:qvel_adr + 6] = [0.0, 0.0, 0.0]

            mujoco.mj_forward(model, data)

        viewer.sync()
        run_button.configure(text="运行 Run")
        status_text.set(
            f"状态：初始状态已应用。初速度={speed:.2f} m/s；"
            "位置和姿态沿用 view.py。"
        )

    def toggle_run():
        nonlocal running, accumulator, last_wall_time

        running = not running
        accumulator = 0.0
        last_wall_time = time.perf_counter()

        run_button.configure(
            text="暂停 Pause" if running else "运行 Run"
        )
        status_text.set(
            "状态：仿真运行中。" if running else "状态：仿真已暂停。"
        )

    def reset_simulation():
        # 与旧控制面板一致：重置到原位置、原姿态，并重新应用当前滑块速度。
        apply_initial_state()
        status_text.set("状态：已重置，当前初速度参数已重新应用。")

    # ========================================================
    # 7. 按钮
    # ========================================================

    button_frame = ttk.Frame(main_frame)
    button_frame.pack(fill="x", pady=10)

    apply_button = ttk.Button(
        button_frame,
        text="应用参数并重置",
        command=apply_initial_state,
    )
    apply_button.pack(side="left", fill="x", expand=True, padx=(0, 4))

    run_button = ttk.Button(
        button_frame,
        text="运行 Run",
        command=toggle_run,
    )
    run_button.pack(side="left", fill="x", expand=True, padx=4)

    reset_button = ttk.Button(
        button_frame,
        text="重置 Reset",
        command=reset_simulation,
    )
    reset_button.pack(side="left", fill="x", expand=True, padx=(4, 0))

    ttk.Label(
        main_frame,
        textvariable=status_text,
        wraplength=500,
    ).pack(anchor="w", pady=(0, 5))

    ttk.Label(
        main_frame,
        text="提示：调整初速度 → 应用参数并重置 → 运行。关闭任一窗口即可结束。",
        wraplength=500,
    ).pack(anchor="w")

    # ========================================================
    # 8. Tkinter 事件循环中驱动 MuJoCo
    # ========================================================

    def simulation_tick():
        nonlocal last_wall_time, accumulator, running


        if closing:
            return

        if not viewer.is_running():
            on_close()
            return

        now = time.perf_counter()
        elapsed = min(max(now - last_wall_time, 0.0), 0.1)
        last_wall_time = now

        if running:
            accumulator += elapsed
            timestep = float(model.opt.timestep)
            steps = min(int(accumulator / timestep), 50)

            if steps > 0:
                with viewer.lock():
                    for _ in range(steps):

                        # 记录单步仿真前的时间和位置
                        old_time = data.time
                        old_pos = data.qpos[qpos_adr:qpos_adr + 3].copy()

                        mujoco.mj_step(model, data)

                        # 如果时间突然回到 0，说明发生了自动重置
                        if data.time < old_time:
                            print("\n========== 检测到 MuJoCo 自动重置 ==========")
                            print(f"重置发生前时间：{old_time:.3f} s")
                            print(f"重置前位置：{old_pos}")
                            print(f"重置后时间：{data.time:.3f} s")
                            print(f"重置后位置：{data.qpos[qpos_adr:qpos_adr + 3]}")
                            print("============================================\n")

                            running = False
                            accumulator = 0.0
                            break
                accumulator -= steps * timestep

        viewer.sync()
        root.after(8, simulation_tick)

    def on_close():
        nonlocal closing

        if closing:
            return

        closing = True
        try:
            if viewer.is_running():
                viewer.close()
        finally:
            root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # 启动时先应用默认初速度，但保持暂停，等待用户点击“运行”。
    apply_initial_state()

    root.after(8, simulation_tick)
    root.mainloop()


if __name__ == "__main__":
    main()
