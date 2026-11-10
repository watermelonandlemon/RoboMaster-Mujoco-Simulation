import math
import time
import mujoco
import mujoco.viewer

# 生成绕指定轴旋转的四元数
def axis_quat(axis, angle):
    half = math.radians(angle) / 2
    c = math.cos(half)
    s = math.sin(half)

    if axis == "x":
        return (c, s, 0, 0)
    if axis == "y":
        return (c, 0, s, 0)
    return (c, 0, 0, s)

# 四元数相乘
def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

#print("MuJoCo version:", mujoco.__version__)
#print("Loading optimized scene:", SCENE)
#print(f"Loaded: {model.nmesh} meshes, {model.ngeom} geoms")

def set_dart_attitude(model, yaw=0.0, pitch=0.0, roll=0.0):
    """
    设置飞镖姿态。

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


def run_viewer(model, data):
    """
    启动 MuJoCo 查看器。

    默认自由摄像头从指定位置开始。
    """

    with mujoco.viewer.launch_passive(model, data) as viewer:

        camera_init(viewer)

        print("MuJoCo 查看器已启动")
        print("默认使用自由摄像头")
        print("鼠标左键：旋转")
        print("Shift + 右键：平移")
        print("滚轮：缩放")

        while viewer.is_running():

            start = time.perf_counter()

            with viewer.lock():
                mujoco.mj_step(model, data)

            viewer.sync()

            elapsed = time.perf_counter() - start

            time.sleep(
                max(0.0, model.opt.timestep - elapsed)
            )
