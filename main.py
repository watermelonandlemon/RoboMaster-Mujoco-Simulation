from pathlib import Path
import math
import time
import tkinter as tk
from tkinter import ttk, messagebox

import mujoco
import mujoco.viewer


# ============================================================
# 文件路径：main.py 与 XML 同目录；meshes 文件夹由 XML 的 meshdir 指定
# ============================================================
PROJECT_DIR = Path(__file__).resolve().parent
XML_PATH = PROJECT_DIR / "xianjiaoda_wukong.xml"

if not XML_PATH.is_file():
    raise FileNotFoundError(
        f"找不到 XML 文件：{XML_PATH}\n"
        "请确认 xianjiaoda_wukong.xml 与 main.py 位于同一目录。"
    )


# ============================================================
# 四元数与发射方向计算
# 模型自身 +Z 轴为镖头方向
# ============================================================
def quat_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return [
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ]


def launch_quaternion(pitch_deg, yaw_deg, roll_deg):
    # 先绕自身纵轴 Z 轴滚转，再将 +Z 镖头方向抬高 pitch，
    # 最后绕世界 Z 轴转到 yaw 方位。
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    roll = math.radians(roll_deg)

    q_roll = [
        math.cos(roll / 2), 0.0, 0.0, math.sin(roll / 2)
    ]
    theta = math.radians(90.0 - pitch_deg)
    q_pitch = [
        math.cos(theta / 2), 0.0, math.sin(theta / 2), 0.0
    ]
    q_yaw = [
        math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)
    ]

    q = quat_multiply(q_yaw, quat_multiply(q_pitch, q_roll))
    norm = math.sqrt(sum(v*v for v in q))
    return [v / norm for v in q]


def launch_velocity(speed, pitch_deg, yaw_deg):
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    return [
        speed * math.cos(pitch) * math.cos(yaw),
        speed * math.cos(pitch) * math.sin(yaw),
        speed * math.sin(pitch),
    ]


# ============================================================
# 加载 MuJoCo 模型
# ============================================================
model = mujoco.MjModel.from_xml_path(str(XML_PATH))
data = mujoco.MjData(model)

joint_id = mujoco.mj_name2id(
    model, mujoco.mjtObj.mjOBJ_JOINT, "dart_free"
)
if joint_id < 0:
    raise RuntimeError(
        'XML 中没有找到名为 "dart_free" 的自由关节。'
        '请检查飞镖 body 内是否有 <freejoint name="dart_free"/>。'
    )
if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
    raise RuntimeError('"dart_free" 不是 free 类型关节。')

qpos_adr = int(model.jnt_qposadr[joint_id])
qvel_adr = int(model.jnt_dofadr[joint_id])

# ============================================================
# 创建窗口和参数变量
# ============================================================
root = tk.Tk()
root.title("RoboMaster 飞镖初始状态控制面板")
root.geometry("540x850")
root.minsize(500, 760)

style = ttk.Style()
try:
    style.theme_use("clam")
except tk.TclError:
    pass

main = ttk.Frame(root, padding=10)
main.pack(fill="both", expand=True)

title = ttk.Label(
    main, text="Dart Initial State / 飞镖初始状态",
    font=("Microsoft YaHei UI", 14, "bold")
)
title.pack(anchor="w", pady=(0, 8))

hint = ttk.Label(
    main,
    text="修改滑条或输入数值，点击“应用参数并重置”，再点击“运行”。",
    wraplength=500
)
hint.pack(anchor="w", pady=(0, 10))

position_vars = {
    "X (m)": tk.DoubleVar(value=0.0),
    "Y (m)": tk.DoubleVar(value=0.0),
    "Z (m)": tk.DoubleVar(value=1.0),
}
angle_vars = {
    "Pitch 仰角 (°)": tk.DoubleVar(value=30.0),
    "Yaw 方位角 (°)": tk.DoubleVar(value=0.0),
    "Roll 自转姿态 (°)": tk.DoubleVar(value=0.0),
}
speed_var = tk.DoubleVar(value=15.0)
spin_var = tk.DoubleVar(value=0.0)

air_enabled = tk.BooleanVar(value=True)
density_var = tk.DoubleVar(value=1.225)
viscosity_var = tk.DoubleVar(value=1.48e-5)

velocity_text = tk.StringVar(value="Vx = 12.99   Vy = 0.00   Vz = 7.50 m/s")
status_text = tk.StringVar(value="状态：已加载模型，等待设置初始条件。")

def add_slider(parent, label, variable, low, high, resolution=0.01):
    row = ttk.Frame(parent)
    row.pack(fill="x", pady=2)

    top = ttk.Frame(row)
    top.pack(fill="x")
    ttk.Label(top, text=label).pack(side="left")
    value_label = ttk.Label(top, width=12, anchor="e")
    value_label.pack(side="right")

    def update_label(*_):
        try:
            value_label.configure(text=f"{variable.get():.3f}")
        except (tk.TclError, ValueError):
            value_label.configure(text="--")

    scale = tk.Scale(
        row, variable=variable, from_=low, to=high,
        orient="horizontal", resolution=resolution,
        showvalue=False, length=470, highlightthickness=0
    )
    scale.pack(fill="x")
    variable.trace_add("write", update_label)
    update_label()


def update_velocity_display(*_):
    try:
        p = angle_vars["Pitch 仰角 (°)"].get()
        y = angle_vars["Yaw 方位角 (°)"].get()
        s = speed_var.get()
        v = launch_velocity(s, p, y)
        velocity_text.set(
            f"Vx = {v[0]:.3f}    Vy = {v[1]:.3f}    Vz = {v[2]:.3f} m/s"
        )
    except (tk.TclError, ValueError):
        velocity_text.set("速度分量：输入有效数值后显示")


position_frame = ttk.LabelFrame(main, text="1. 初始位置 Position（世界坐标，m）", padding=8)
position_frame.pack(fill="x", pady=5)
add_slider(position_frame, "X", position_vars["X (m)"], -5.0, 5.0)
add_slider(position_frame, "Y", position_vars["Y (m)"], -5.0, 5.0)
add_slider(position_frame, "Z", position_vars["Z (m)"], 0.0, 5.0)

angle_frame = ttk.LabelFrame(main, text="2. 初始姿态 Orientation（度）", padding=8)
angle_frame.pack(fill="x", pady=5)
add_slider(angle_frame, "Pitch 仰角", angle_vars["Pitch 仰角 (°)"], -90.0, 90.0, 0.5)
add_slider(angle_frame, "Yaw 方位角", angle_vars["Yaw 方位角 (°)"], -180.0, 180.0, 0.5)
add_slider(angle_frame, "Roll 绕自身轴滚转", angle_vars["Roll 自转姿态 (°)"], -180.0, 180.0, 0.5)

speed_frame = ttk.LabelFrame(main, text="3. 初始速度 Velocity", padding=8)
speed_frame.pack(fill="x", pady=5)
add_slider(speed_frame, "初速度大小 (m/s)", speed_var, 0.0, 40.0, 0.1)
add_slider(speed_frame, "绕自身纵轴角速度 (rad/s)", spin_var, -100.0, 100.0, 0.5)
ttk.Label(
    speed_frame,
    text="速度方向自动与飞镖镖头轴线一致，由 Pitch 和 Yaw 决定。",
    wraplength=480
).pack(anchor="w", pady=(3, 2))
ttk.Label(speed_frame, textvariable=velocity_text, foreground="#1769aa").pack(anchor="w")
for var in list(angle_vars.values()) + [speed_var]:
    var.trace_add("write", update_velocity_display)
update_velocity_display()

air_frame = ttk.LabelFrame(main, text="4. 空气模型（MuJoCo 内置近似流体力）", padding=8)
air_frame.pack(fill="x", pady=5)
# 初始化状态与空气参数
# ============================================================
running = False
accumulator = 0.0
last_wall_time = time.perf_counter()
viewer = None


def apply_air_settings():
    if air_enabled.get():
        rho = float(density_var.get())
        nu = float(viscosity_var.get())
        if rho <= 0 or nu <= 0:
            raise ValueError("空气密度和运动黏度必须大于 0。")
        model.opt.density = rho
        model.opt.viscosity = nu
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    else:
        model.opt.density = 0.0
        model.opt.viscosity = 0.0
        model.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4


def apply_initial_state():
    global running, accumulator

    x = position_vars["X (m)"].get()
    y = position_vars["Y (m)"].get()
    z = position_vars["Z (m)"].get()

    pitch = angle_vars["Pitch 仰角 (°)"].get()
    yaw = angle_vars["Yaw 方位角 (°)"].get()
    roll = angle_vars["Roll 自转姿态 (°)"].get()
    speed = speed_var.get()
    spin = spin_var.get()

    apply_air_settings()

    quat = launch_quaternion(pitch, yaw, roll)
    vel = launch_velocity(speed, pitch, yaw)

    mujoco.mj_resetData(model, data)

    data.qpos[qpos_adr:qpos_adr + 3] = [x, y, z]
    data.qpos[qpos_adr + 3:qpos_adr + 7] = quat

    # freejoint 的线速度分量为世界坐标系下的 vx, vy, vz；
    # 角速度分量按自由关节的局部角速度坐标设置。
    data.qvel[qvel_adr:qvel_adr + 3] = vel
    data.qvel[qvel_adr + 3:qvel_adr + 6] = [0.0, 0.0, spin]

    mujoco.mj_forward(model, data)

    running = False
    accumulator = 0.0
    run_button.configure(text="运行 Run")
    status_text.set(
        f"状态：初始状态已应用。位置=({x:.2f}, {y:.2f}, {z:.2f}) m；"
        f"速度={speed:.2f} m/s；空气模型={'开' if air_enabled.get() else '关'}。"
    )

    if viewer is not None and viewer.is_running():
        viewer.cam.lookat[:] = [x, y, z]
        viewer.sync()


def toggle_run():
    global running, last_wall_time
    running = not running
    last_wall_time = time.perf_counter()
    run_button.configure(text="暂停 Pause" if running else "运行 Run")
    status_text.set("状态：仿真运行中。" if running else "状态：仿真已暂停。")


def reset_simulation():
    apply_initial_state()


# ============================================================
# 启动 MuJoCo Viewer
# ============================================================
try:
    viewer = mujoco.viewer.launch_passive(model, data)
except Exception as exc:
    messagebox.showerror(
        "MuJoCo Viewer 启动失败",
        f"{exc}\n\n请检查 MuJoCo 安装和图形环境。"
    )
    root.destroy()
    raise

viewer.cam.lookat[:] = [0.0, 0.0, 1.0]
viewer.cam.distance = 1.5
viewer.cam.azimuth = 90
viewer.cam.elevation = -15

# ============================================================
# Tk 界面事件循环中驱动 MuJoCo，避免额外线程同时改写 data
# ============================================================
def simulation_tick():
    global last_wall_time, accumulator

    if viewer is None or not viewer.is_running():
        try:
            root.destroy()
        except tk.TclError:
            pass
        return

    now = time.perf_counter()
    elapsed = min(now - last_wall_time, 0.1)
    last_wall_time = now

    if running:
        accumulator += elapsed
        timestep = model.opt.timestep
        steps = min(int(accumulator / timestep), 50)

        for _ in range(steps):
            mujoco.mj_step(model, data)

        accumulator -= steps * timestep

    viewer.sync()
    root.after(8, simulation_tick)


button_frame = ttk.Frame(main)
button_frame.pack(fill="x", pady=10)

apply_button = ttk.Button(
    button_frame, text="应用参数并重置", command=apply_initial_state
)
apply_button.pack(side="left", fill="x", expand=True, padx=(0, 4))

run_button = ttk.Button(
    button_frame, text="运行 Run", command=toggle_run
)
run_button.pack(side="left", fill="x", expand=True, padx=4)

reset_button = ttk.Button(
    button_frame, text="重置 Reset", command=reset_simulation
)


reset_button.pack(side="left", fill="x", expand=True, padx=(4, 0))

# 初始化飞镖状态
apply_initial_state()

ttk.Label(
    main, textvariable=status_text, wraplength=500
).pack(anchor="w", pady=(0, 5))

ttk.Label(
    main,
    text="提示：先调位置/角度/速度 → 应用参数并重置 → 运行。关闭任一窗口即可结束。",
    wraplength=500
).pack(anchor="w")

root.after(8, simulation_tick)


def on_close():
    try:
        if viewer is not None and viewer.is_running():
            viewer.close()
    finally:
        root.destroy()


root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()