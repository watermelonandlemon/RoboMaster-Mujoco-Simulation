# -*- coding: utf-8 -*-
from dataclasses import dataclass, replace
import math
from pathlib import Path
import random
import time
import tkinter as tk
from tkinter import messagebox
from typing import Optional

import mujoco
import mujoco.viewer
import numpy as np

from control_input import ControlInputReceiver, ForceTorque
from sensor_output import INTERFACE_PERIOD_S, SensorPublisher


# ======================== 飞镖姿态与速度计算 ========================

DEFAULT_YAW = -15.2
DEFAULT_PITCH = 28.5
DEFAULT_ROLL = 0.0

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


# ======================== MuJoCo Viewer 摄像机 ========================

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


# ======================== 镖靶运动模型 ========================

TARGET_MODE_FIXED = "fixed"
TARGET_MODE_RANDOM_FIXED = "random_fixed"
TARGET_MODE_RANDOM_MOVING = "random_moving"
TARGET_MODE_TERMINAL_MOVING = "terminal_moving"
TARGET_MOVING_MODES = (TARGET_MODE_RANDOM_MOVING, TARGET_MODE_TERMINAL_MOVING)

BASE_TARGET_BODY_NAME = "base_right_target"
BASE_TARGET_MIN_OFFSET_MM = -280
BASE_TARGET_MAX_OFFSET_MM = 280
TARGET_MIN_SPEED_MPS = 0.4
TARGET_MAX_SPEED_MPS = 1.0
TARGET_DIRECTION_INTERVAL_S = 0.4


class TargetMotionPlan:
    """单次仿真的随机轨迹；速度仅在创建时抽取一次。

    位置单位为 mm，速度单位为 m/s；仿真时间必须单调递增。
    对 0.4 秒换向时刻进行分段积分，对边界进行反射，因此不会越过 ±280mm。
    """

    def __init__(self, mode, rng):
        if mode not in TARGET_MOVING_MODES:
            raise ValueError(f"不支持的自动镖靶模式：{mode}")
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


# ======================== 绿灯像素几何真值 ========================

# 从原始 move2_m03 绿色灯罩网格中确定的灯面中心参考点，
# 坐标位于 base_right_target 的局部坐标系。该 Body 原本就是 mocap body。
# 当前 mocap Body 原点在世界原点，仅做平移；若未来修改旋转中心，须同步修改模型及参考点。
BASE_LED_CENTER_LOCAL = np.array([25.488436, 7.499998, 1.243904], dtype=float)
BASE_GEOM_NAME = "base_right_target_visual_03"
SURFACE_TOLERANCE_M = 1e-4


def camera_intrinsics(width, height, fovy_deg):
    if (not isinstance(width, (int, np.integer)) or
            not isinstance(height, (int, np.integer)) or width < 1 or height < 1):
        raise ValueError("图像宽高必须为正整数")
    fovy_deg = float(fovy_deg)
    if not math.isfinite(fovy_deg) or not 0.0 < fovy_deg < 180.0:
        raise ValueError("垂直视场角必须在 (0, 180) 度之间")
    focal = height / (2.0 * math.tan(math.radians(fovy_deg) / 2.0))
    return focal, focal, (width - 1.0) / 2.0, (height - 1.0) / 2.0


def base_led_world_position(body_position, body_rotation):
    """把基地绿灯局部参考点变换到世界坐标系。rotation 为 body-local -> world。"""
    pos = np.asarray(body_position, dtype=float).reshape(3)
    rot = np.asarray(body_rotation, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(pos)) or not np.all(np.isfinite(rot)):
        raise ValueError("基地 Body 位姿必须为有限数值")
    return pos + rot @ BASE_LED_CENTER_LOCAL


def project_world_point(point_world, camera_position, camera_rotation,
                        width, height, fovy_deg, znear, zfar):
    """将世界点投影到 MuJoCo 原始 RGB 图像像素坐标。"""
    fx, fy, cx, cy = camera_intrinsics(width, height, fovy_deg)
    znear, zfar = float(znear), float(zfar)
    if not (math.isfinite(znear) and math.isfinite(zfar) and 0 < znear < zfar):
        raise ValueError("裁剪范围必须满足 0 < znear < zfar")
    point = np.asarray(point_world, dtype=float).reshape(3)
    position = np.asarray(camera_position, dtype=float).reshape(3)
    rotation = np.asarray(camera_rotation, dtype=float).reshape(3, 3)
    if not all(np.all(np.isfinite(x)) for x in (point, position, rotation)):
        raise ValueError("相机和目标坐标必须为有限数值")
    x, y, z = rotation.T @ (point - position)
    depth = float(-z)
    front = depth > 1e-12
    u = float(cx + fx * x / depth) if front else None
    v = float(cy - fy * y / depth) if front else None
    if front and not (math.isfinite(u) and math.isfinite(v)):
        raise ValueError("投影超出有限数值范围")
    in_frame = front and -0.5 <= u < width - 0.5 and -0.5 <= v < height - 0.5
    return {
        "projected_u": u, "projected_v": v,
        "depth_m": depth, "in_front": bool(front),
        "in_frame": bool(in_frame),
        "in_clip_range": bool(znear <= depth <= zfar),
    }


class VisionGroundTruth:
    """只计算对面基地绿灯中心的像素投影和中心视线遮挡。"""
    def __init__(self, model, data, camera="dart_fpv", width=640, height=480):
        self.mj = mujoco
        self.model, self.data = model, data
        self.camera = str(camera)
        self.width, self.height = int(width), int(height)
        self.camera_id = self._id(mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
        self.body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, BASE_TARGET_BODY_NAME)
        self.geom_id = self._id(mujoco.mjtObj.mjOBJ_GEOM, BASE_GEOM_NAME)
        camera_intrinsics(self.width, self.height, model.cam_fovy[self.camera_id])
        self.geomgroup = mujoco.MjvOption().geomgroup.copy()

    def _id(self, kind, name):
        value = int(self.mj.mj_name2id(self.model, kind, name))
        if value < 0:
            raise ValueError(f"视觉真值缺少模型对象 {name!r}")
        return value

    def sample(self, image_available=False):
        m, d, cid = self.model, self.data, self.camera_id
        if hasattr(m, "cam_orthographic") and bool(m.cam_orthographic[cid]):
            raise ValueError("视觉真值接口当前仅支持 fovy 对称透视相机")
        if hasattr(m, "cam_sensorsize") and np.any(m.cam_sensorsize[cid] > 0):
            raise ValueError("相机已设置 sensorsize/intrinsic；不能再用 fovy 投影")
        fovy = float(m.cam_fovy[cid])
        fx, fy, cx, cy = camera_intrinsics(self.width, self.height, fovy)
        camera_position = d.cam_xpos[cid].copy()
        camera_rotation = d.cam_xmat[cid].reshape(3, 3).copy()
        # 使用原模型中已有的 mocap Body 位姿，不要求修改 model.xml。
        point = base_led_world_position(d.xpos[self.body_id],
                                        d.xmat[self.body_id].reshape(3, 3))
        near = float(m.vis.map.znear * m.stat.extent)
        far = float(m.vis.map.zfar * m.stat.extent)
        projected = project_world_point(point, camera_position, camera_rotation,
                                        self.width, self.height, fovy, near, far)
        depth = projected.pop("depth_m")
        target = dict(projected, u=None, v=None, visible=False, occluded=None,
                      reason="offscreen")
        if not projected["in_front"]:
            target["reason"] = "behind_camera"
        elif not projected["in_clip_range"]:
            target["reason"] = "clipped"
        elif projected["in_frame"]:
            vector = point - camera_position
            distance = float(np.linalg.norm(vector))
            direction = vector / distance
            start_distance = near * distance / depth
            start = camera_position + start_distance * direction
            hit_id = np.array([-1], dtype=np.int32)
            hit = float(self.mj.mj_ray(m, d, start, direction,
                                       self.geomgroup, 1, -1, hit_id))
            total_hit = hit + start_distance
            on_target = (hit >= 0 and int(hit_id[0]) == self.geom_id and
                         abs(total_hit - distance) <= SURFACE_TOLERANCE_M)
            if on_target:
                target.update(u=projected["projected_u"],
                              v=projected["projected_v"],
                              visible=True, occluded=False, reason="visible")
            elif hit >= 0 and total_hit <= distance + SURFACE_TOLERANCE_M:
                target.update(occluded=True, reason="occluded")
            else:
                target["reason"] = "ray_miss"
        return {
            "valid": True, "camera": self.camera,
            "width": self.width, "height": self.height,
            "coordinate_frame": "image_pixels_top_left",
            "pixel_origin": "top_left_pixel_center",
            "center_definition": "base_led_front_center_reference",
            "visibility_definition": "center_ray",
            "fovy_deg": fovy, "fx": fx, "fy": fy, "cx": cx, "cy": cy,
            "image_available": bool(image_available),
            "targets": {"base": target},
        }


# ======================== MuJoCo 飞镖动力学与传感采样 ========================

@dataclass(frozen=True)
class SensorSample:
    """传感接口输出（坐标全部为飞行机体系）。

    image_rgb: (H,W,3) uint8 NumPy 数组；未捕获前为 None，保留最近一帧。
               其时间戳为 image_time_s。
    acceleration_xyz_mps2: 数值微分 dv_world/dt 转到当前飞行机体系；
               包含重力，区别于真实 IMU 传感器测量的比力。
    angular_velocity_xyz_rad_s: 飞行机体系角速度 (rad/s)，非欧拉角变化率。
    """
    acceleration_xyz_mps2: tuple
    angular_velocity_xyz_rad_s: tuple
    acceleration_valid: bool
    image_rgb: Optional[np.ndarray] = None
    image_time_s: Optional[float] = None


# STL 长轴沿 +Z，模型装配时右乘 Ry(+90°) 使机头指向 +X；
# 去掉该固定姿态后的飞行机体系四元数为 q_stl * Ry(-90°)。
_SQRT_HALF = math.sqrt(0.5)
_STL_TO_FLIGHT_Q = (_SQRT_HALF, 0.0, -_SQRT_HALF, 0.0)


def _quat_rotate(q, vec):
    """将向量从四元数局部系旋转到父系，无 scipy 依赖。"""
    w, x, y, z = (float(v) for v in q)
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-12:
        raise ValueError("飞镖姿态四元数为零")
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    vx, vy, vz = (float(v) for v in vec)
    tx = 2.0 * (y*vz - z*vy)
    ty = 2.0 * (z*vx - x*vz)
    tz = 2.0 * (x*vy - y*vx)
    return (vx + w*tx + y*tz - z*ty,
            vy + w*ty + z*tx - x*tz,
            vz + w*tz + x*ty - y*tx)


def _world_to_flight(q_stl, vector_world):
    q_flight = quat_mul(q_stl, _STL_TO_FLIGHT_Q)
    w, x, y, z = q_flight
    return _quat_rotate((w, -x, -y, -z), vector_world)


def _flight_to_world(q_stl, vector_flight):
    q_flight = quat_mul(q_stl, _STL_TO_FLIGHT_Q)
    return _quat_rotate(q_flight, vector_flight)


class DartInterfaces:
    """MuJoCo 内部飞镖接口：施加六维力/力矩并采样 IMU 与 FPV 图像。

    主动/被动输入值由 control_input.py 的接收器提供；本类只负责把两路
    飞行机体系力/力矩相加、转换到世界系并施加到 MuJoCo 刚体，同时采样
    仿真状态。图像与绿灯真值的网络发布不由本类负责。

    此对象由运行 MuJoCo 的 Tk 主线程驱动；外部线程不要直接访问其状态。
    """
    def __init__(self, model, data, body_id, qpos_adr, qvel_adr,
                 image_camera="dart_fpv", image_width=320, image_height=240):
        self.model = model
        self.data = data
        self.body_id = int(body_id)
        self.qpos_adr = int(qpos_adr)
        self.qvel_adr = int(qvel_adr)
        self.image_camera = image_camera
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self._renderer = None
        self._vision_gt = None
        self.vision_error = None
        self.camera_error = None
        if self.image_width < 1 or self.image_height < 1:
            raise ValueError("图像尺寸必须大于零")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, image_camera) < 0:
            raise RuntimeError(f"飞镖 FPV 摄像机不存在：{image_camera}")
        self.reset()

    def _q_stl(self):
        return self.data.qpos[self.qpos_adr+3:self.qpos_adr+7]

    def set_active_input(self, fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
        """更新仿真当前周期使用的主动六维力/力矩。"""
        self.active_input = ForceTorque(fx, fy, fz, mx, my, mz)

    def set_passive_input(self, fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
        """更新仿真当前周期使用的被动六维力/力矩。"""
        self.passive_input = ForceTorque(fx, fy, fz, mx, my, mz)

    def reset(self):
        """重置控制值、力矩与采样缓存，防止旧一局残余六维力影响新一局。"""
        self.active_input = ForceTorque()
        self.passive_input = ForceTorque()
        self.last_active = ForceTorque()
        self.last_passive = ForceTorque()
        self.last_total = ForceTorque()
        self._frame_rgb = None
        self._frame_time_s = None
        self.data.xfrc_applied[self.body_id, :] = 0.0
        self.latest_sensor = self._new_sample((0.0, 0.0, 0.0), valid=False)

    def _new_sample(self, acceleration, valid):
        omega = self.data.qvel[self.qvel_adr+3:self.qvel_adr+6]
        return SensorSample(
            acceleration_xyz_mps2=tuple(float(v) for v in acceleration),
            angular_velocity_xyz_rad_s=dart_body_angular_velocity(omega),
            acceleration_valid=bool(valid),
            image_rgb=self._frame_rgb,
            image_time_s=self._frame_time_s,
        )

    def read_sensor(self):
        """传感接口：最新的仿真状态及最近一帧 RGB，均为只读快照。"""
        return self.latest_sensor

    def capture_vision_gt(self, image_available=False):
        """调用方持锁并已刷新 kinematics/camlight；不读取RGB、不创建Renderer。"""
        try:
            if self._vision_gt is None:
                self._vision_gt = VisionGroundTruth(
                    self.model, self.data, camera=self.image_camera,
                    width=self.image_width, height=self.image_height,
                )
            result = self._vision_gt.sample(image_available=image_available)
            self.vision_error = None
            return result
        except Exception as error:
            message = f"{type(error).__name__}: {error}"
            if message != self.vision_error:
                print(f"[视觉真值] {message}")
            self.vision_error = message
            # 明确发布无效状态，绝不把上一次的坐标当成本次输出。
            return {
                "valid": False, "error": message, "camera": self.image_camera,
                "width": self.image_width, "height": self.image_height,
                "image_available": bool(image_available), "targets": {},
            }

    def before_step(self):
        """在 mj_step 之前将主动、被动六维输入相加并施加到飞镖刚体。"""
        active = self.active_input
        passive = self.passive_input
        if not isinstance(active, ForceTorque) or not isinstance(passive, ForceTorque):
            raise TypeError("主动/被动输入必须为 ForceTorque 类型")
        total = active + passive
        self.last_active = active
        self.last_passive = passive
        self.last_total = total

        target = self.data.xfrc_applied[self.body_id]
        if total == ForceTorque():
            target[:] = 0.0
        else:
            q = self._q_stl()
            target[:3] = _flight_to_world(q, (total.fx, total.fy, total.fz))
            target[3:] = _flight_to_world(q, (total.mx, total.my, total.mz))
        # 所有加速度与速度采样都在 mj_step 之后，不在此做微分。

    def after_step(self, velocity_before_world, dt):
        """从 mj_step 的前后速度差计算包含重力的质心线加速度。"""
        if dt <= 0:
            raise ValueError("物理步长必须大于零")
        v_after = self.data.qvel[self.qvel_adr:self.qvel_adr+3]
        a_world = (np.asarray(v_after) - np.asarray(velocity_before_world)) / dt
        a_body = _world_to_flight(self._q_stl(), a_world)
        self.latest_sensor = self._new_sample(a_body, valid=True)

    def capture_image(self):
        """按需渲染一帧 FPV 图像；调用方须持有 viewer.lock()。"""
        if self.camera_error is not None:
            return False
        now = float(self.data.time)
        try:
            if self._renderer is None:
                self._renderer = mujoco.Renderer(
                    self.model, height=self.image_height, width=self.image_width
                )
            self._renderer.update_scene(self.data, camera=self.image_camera)
            frame = self._renderer.render().copy()
            frame.setflags(write=False)
            self._frame_rgb = frame
            self._frame_time_s = now
            self.latest_sensor = replace(
                self.latest_sensor, image_rgb=frame, image_time_s=now
            )
            return True
        except Exception as error:
            # 离屏渲染失败不影响动力学和数值传感器。
            self.camera_error = f"{type(error).__name__}: {error}"
            print(f"[传感接口] FPV 离屏渲染失败：{self.camera_error}")
            print("[传感接口] 线加速度/角速度仍有效；请检查 OpenGL 环境。")
            return False

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


# ======================== 仿真控制面板与物理循环 ========================

DEFAULT_SPEED = 18.0
MIN_SPEED = 5.0
MAX_SPEED = 30.0
SPEED_RESOLUTION = 0.1


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


class DartSimulationApp:
    """飞镖仿真控制面板及 MuJoCo 仿真循环。"""

    def __init__(self, model, data, viewer, qpos_adr, qvel_adr):
        self.model = model
        self.data = data
        self.viewer = viewer
        self.qpos_adr = qpos_adr
        self.qvel_adr = qvel_adr

        # xipos 是飞镖的世界坐标系质心（不是靶心距离）。
        self.dart_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "dart_preview"
        )
        if self.dart_body_id < 0:
            raise RuntimeError("模型中找不到飞镖刚体 dart_preview")

        # 镖靶在 XML 中已有 mocap body，直接更新 mocap_pos 即可让外观和碰撞体同步移动。
        target_body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, BASE_TARGET_BODY_NAME
        )
        if target_body_id < 0:
            raise RuntimeError(f"模型中找不到活动镖靶 {BASE_TARGET_BODY_NAME}")
        self.base_target_mocap_id = int(model.body_mocapid[target_body_id])
        if self.base_target_mocap_id < 0:
            raise RuntimeError(f"{BASE_TARGET_BODY_NAME} 不是 mocap body，无法控制平移")
        self.base_target_mocap_origin = data.mocap_pos[self.base_target_mocap_id].copy()
        self.base_target_offset_mm = 0.0
        self._updating_target_controls = False
        self.target_rng = random.Random()
        self.target_motion = None

        # 对外数据接口：主动/被动输入和传感输出均使用 UDP。
        # 三个通道独立运行；没有输入发布者时，主动/被动力默认为零。
        self.control_input = ControlInputReceiver()
        try:
            self.sensor_output = SensorPublisher()
        except Exception:
            self.control_input.close()
            raise
        try:
            self.interfaces = DartInterfaces(
                model=model, data=data,
                body_id=self.dart_body_id,
                qpos_adr=qpos_adr, qvel_adr=qvel_adr,
                image_camera="dart_fpv", image_width=640, image_height=480,
            )
        except Exception:
            self.sensor_output.close()
            self.control_input.close()
            raise
        self._io_next_wall = time.perf_counter()
        self._io_frame_id = 0

        # 接口的六维力由 DartInterfaces 独立管理。

        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()
        self.closing = False

        self.root = tk.Tk()
        self.root.title("RoboMaster")
        self.root.geometry("500x860")
        self.root.minsize(480, 680)
        self.target_mode_var = tk.StringVar(master=self.root, value=TARGET_MODE_FIXED)

        self._build_ui()
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        # 启动时应用默认参数，但保持暂停。
        self._apply_initial_state()
        self.root.after(8, self._simulation_tick)

    def _build_ui(self):
        """构建深色控制面板。"""
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

        main_frame = tk.Frame(self.root, bg=background, width=455)
        main_frame.pack(side="left", fill="y", padx=(12, 24), pady=12)
        main_frame.pack_propagate(False)


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
        speed_entry = dark_entry(speed_top, self.speed_value_text, width=10, justify="right")
        speed_entry.pack(side="right")
        speed_entry.bind("<Return>", self._on_speed_entry)
        speed_entry.bind("<FocusOut>", self._on_speed_entry)
        speed_scale = _DarkSlider(
            speed_frame, variable=self.speed_var, minimum=MIN_SPEED,
            maximum=MAX_SPEED, resolution=SPEED_RESOLUTION
        )
        speed_scale.pack(fill="x", pady=(0, 5))
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
        target_offset_scale = _DarkSlider(
            target_frame, variable=self.target_offset_var,
            minimum=BASE_TARGET_MIN_OFFSET_MM,
            maximum=BASE_TARGET_MAX_OFFSET_MM, resolution=1,
            command=self._on_target_offset_slider, reverse=True,
        )
        target_offset_scale.pack(fill="x", pady=(0, 3))
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
        for column in range(2):
            button_frame.grid_columnconfigure(column, weight=1, uniform="actions")
        apply_button = dark_button(button_frame, "应用参数并重置", self._apply_initial_state)
        apply_button.grid(row=0, column=0, sticky="ew", padx=(0, 5))
        self.run_button = dark_button(button_frame, "运行 Run", self._toggle_run, primary=True)
        self.run_button.grid(row=0, column=1, sticky="ew")

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

    # ======================== 飞镖状态读取 ========================

    def _read_dart_state_locked(self):
        """读取 MuJoCo 飞镖真值。调用方必须持有 viewer.lock()。"""
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
            # 保留原生四元数，避免后续仅凭欧拉角重建姿态时丢失信息。
            "qw": float(quaternion[0]), "qx": float(quaternion[1]),
            "qy": float(quaternion[2]), "qz": float(quaternion[3]),
        }
        return snapshot


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

    def _reset_episode(self):
        """重置公共状态；调用方必须持有 viewer.lock()。"""
        self.target_motion = None
        if self.target_mode_var.get() != TARGET_MODE_RANDOM_FIXED:
            self.base_target_offset_mm = 0.0
        self._write_target_mocap_offset()
        mujoco.mj_forward(self.model, self.data)
        self.interfaces.reset()
        self._io_next_wall = time.perf_counter()

    def _apply_initial_state(self):
        """根据面板参数更新飞镖姿态和初速度，并重置仿真。"""
        try:
            speed, yaw, pitch, roll = self._read_parameters()
        except (tk.TclError, ValueError) as error:
            messagebox.showerror("参数错误", str(error), parent=self.root)
            return

        velocity = launch_velocity(speed, pitch, yaw)
        self.running = False
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()

        with self.viewer.lock():
            # 模型默认姿态和 data 重置在同一把锁内完成。
            quat = set_dart_attitude(self.model, yaw=yaw, pitch=pitch, roll=roll)
            self.model.qpos0[self.qpos_adr + 3:self.qpos_adr + 7] = quat
            mujoco.mj_resetData(self.model, self.data)
            self.data.qvel[:] = 0.0

            # freejoint 的前三个速度分量为世界坐标系线速度。
            self.data.qvel[self.qvel_adr:self.qvel_adr + 3] = velocity

            # 接收器继续保留最新网络输入；公共重置仅清理仿真状态。
            self._reset_episode()

        self._sync_target_controls(force_entry=True)
        self.viewer.sync()
        self.run_button.configure(text="运行 Run")

    def _toggle_run(self):
        if not self.running:
            self._prepare_target_motion_for_run()
        self.running = not self.running
        self.accumulator = 0.0
        self.last_wall_time = time.perf_counter()

        self.run_button.configure(
            text="暂停 Pause" if self.running else "运行 Run"
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
            # 单次 GUI 回调最多推进 100 步，未处理的时间留在累加器中。
            # 渲染或计算过慢时，仿真时间可能落后于真实时间。
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
                        # 每个物理步读取输入接收器的最新值，再由仿真接口转换坐标系并施加。
                        active, passive = self.control_input.poll_inputs()
                        self.interfaces.set_active_input(*active)
                        self.interfaces.set_passive_input(*passive)
                        # 同一物理时刻：主动 Wrench + 被动 Wrench，重力仍由 MuJoCo 自己计算。
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

                            # 自动重置后恢复等待状态，保留原有异常诊断。
                            self._reset_episode()
                            target_moved = True
                            self.running = False
                            self.accumulator = 0.0
                            self.run_button.configure(text="运行 Run")
                            break
                        # 读取刚完成的物理步，输出机体系线加速度和角速度。
                        # 此处用的 dt 是实际仿真时间增量，而非 GUI 帧间隔。
                        self.interfaces.after_step(
                            velocity_before, float(self.data.time - old_time)
                        )

                    # mj_step 后刷新运动学派生数据，供后续传感采样使用。
                    mujoco.mj_kinematics(self.model, self.data)

                if self.running:
                    self.accumulator = max(
                        0.0,
                        self.accumulator - steps * timestep,
                    )
                # 只按 GUI tick 更新滑条/数字，不在每个物理步操作 Tk 控件。
                if target_moved:
                    self._sync_target_controls()

        # 传感输出以真实时间 30Hz 为目标；物理步继续独立读取最新输入。
        self._exchange_io_tick()

        self.viewer.sync()
        self.root.after(8, self._simulation_tick)

    def _exchange_io_tick(self):
        """目标 30Hz 的传感输出周期；UDP 发送在后台线程中执行。"""
        now = time.perf_counter()
        if now < self._io_next_wall:
            return
        self._io_next_wall += INTERFACE_PERIOD_S
        if self._io_next_wall < now:
            self._io_next_wall = now + INTERFACE_PERIOD_S

        self._io_frame_id += 1
        if not self.sensor_output.sensor_connected:
            return

        # 只读取接收器当前缓存用于状态消息，不重复写入物理步输入。
        active, passive = self.control_input.poll_inputs()

        with self.viewer.lock():
            # 捕获当前 MuJoCo data 的 dart_fpv 真实画面，而不是打印形状代替画面。
            # 必须与此时刻的加速度、角速度一起构建同一个传感消息。
            # mj_step 更新 qpos 后，机体与摄像机的派生位姿都要刷新。
            # 图像、site 投影、射线查询都读取下面同一个锁内的状态。
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_camlight(self.model, self.data)
            captured = (
                self.interfaces.capture_image()
                if self.sensor_output.image_subscribed else False
            )
            sample = self.interfaces.read_sensor()
            # 未订阅图像或渲染失败时，不把旧缓存标成新frame_id重新发出。
            frame = sample.image_rgb if captured else None
            vision_gt = (
                self.interfaces.capture_vision_gt(image_available=frame is not None)
                if self.sensor_output.vision_subscribed else None
            )

            # 上帝视角：直接读取 MuJoCo 当前真值，不通过 IMU 积分反推。
            world_state = self._read_dart_state_locked()
            meta = {
                "frame_id": (self._io_frame_id - 1) & 0xffffffff,
                "time_s": float(self.data.time),
                "image_time_s": sample.image_time_s if captured else None,

                # IMU 输出：飞行机体系。
                "coordinate_frame": "dart_flight_body",
                "acceleration_body_mps2": list(sample.acceleration_xyz_mps2),
                "angular_velocity_body_rad_s": list(sample.angular_velocity_xyz_rad_s),
                "acceleration_valid": sample.acceleration_valid,

                # Full State：MuJoCo 真值。位置、线速度均为世界坐标系。
                "position_world_m": [
                    world_state["x_m"], world_state["y_m"], world_state["z_m"]
                ],
                # 右侧基地靶板碰撞盒中心的实时世界坐标（m），不是绿灯中心。
                "target_position_world_m": self.data.geom("dart_target_collision_1").xpos.tolist(),
                "linear_velocity_world_mps": [
                    world_state["vx_mps"], world_state["vy_mps"], world_state["vz_mps"]
                ],
                "orientation_quaternion_wxyz": [
                    world_state["qw"], world_state["qx"],
                    world_state["qy"], world_state["qz"]
                ],
                "orientation_yaw_pitch_roll_deg": [
                    world_state["yaw_deg"], world_state["pitch_deg"],
                    world_state["roll_deg"]
                ],
                "angular_velocity_body_deg_s": [
                    world_state["wx_deg_s"], world_state["wy_deg_s"],
                    world_state["wz_deg_s"]
                ],
                # 同时保留原有标量字段，便于接收端直接读取。
                **world_state,

                # 最近物理步施加的力/力矩，以及当前接收器缓存。
                "active_force_body": list(self.interfaces.last_active.as_tuple()),
                "passive_force_body": list(self.interfaces.last_passive.as_tuple()),
                "total_force_body": list(self.interfaces.last_total.as_tuple()),
                "next_active_force_body": list(active),
                "next_passive_force_body": list(passive),
                "camera_error": self.interfaces.camera_error,
            }

        # 真正的 socket 发送在独立线程内，队列始终只保留最近一帧。
        self.sensor_output.publish_sensor(meta, frame, vision_gt)

    def _on_close(self):
        if self.closing:
            return

        self.closing = True
        try:
            try:
                self.sensor_output.close()
            finally:
                try:
                    self.control_input.close()
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


# ======================== 仿真启动 ========================


def run_simulation():
    """加载场景、初始化飞镖状态与 viewer，并启动控制面板。"""
    scene = Path(__file__).resolve().with_name("model.xml")
    dart_joint_name = "dart_free"
    if not scene.is_file():
        raise FileNotFoundError(f"找不到场景文件：{scene}")

    print(f"正在加载场景：{scene}")
    model = mujoco.MjModel.from_xml_path(str(scene))
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, dart_joint_name)
    if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError(f"场景中找不到自由关节 {dart_joint_name!r}")

    qpos_adr = int(model.jnt_qposadr[joint_id])
    qvel_adr = int(model.jnt_dofadr[joint_id])
    quat = set_dart_attitude(
        model,
        yaw=DEFAULT_YAW,
        pitch=DEFAULT_PITCH,
        roll=DEFAULT_ROLL,
    )

    model.qpos0[qpos_adr + 3:qpos_adr + 7] = quat
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)

    print("飞镖刚体：dart_preview")
    print(f"发射位置沿用场景设置：{data.qpos[qpos_adr:qpos_adr + 3]}")
    print(
        "初始姿态："
        f"YAW={DEFAULT_YAW}°，"
        f"PITCH={DEFAULT_PITCH}°，"
        f"ROLL={DEFAULT_ROLL}°"
    )

    viewer = mujoco.viewer.launch_passive(model, data)
    try:
        camera_init(viewer)
        app = DartSimulationApp(
            model=model,
            data=data,
            viewer=viewer,
            qpos_adr=qpos_adr,
            qvel_adr=qvel_adr,
        )
        app.run()
    except Exception:
        if viewer.is_running():
            viewer.close()
        raise
