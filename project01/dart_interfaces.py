# -*- coding: utf-8 -*-
"""RoboMaster MuJoCo 飞镖 I/O 接口（与 GUI 分离）。

飞行机体系：+X 指向飞镖机头，+Y/+Z 由原 STL 本体绕 Y 轴 -90° 得到。
三个接口：
  sensor: 读取相机 RGB、机体系线加速度、角速度。
  active: 主动控制力/力矩 (N, N*m)。
  passive: 被动等效力/力矩 (N, N*m)。

主动+被动六维力/力矩同一仿真时刻逐分量相加后，转换到世界系，
施加到飞镖质心（不改变模型中的重力、惯性或碰撞）。
"""
from dataclasses import dataclass, replace
import math
from typing import Callable, Optional

import numpy as np
import mujoco


@dataclass(frozen=True)
class ForceTorque:
    """飞行机体系的等效质心力与力矩。力 N，力矩 N·m。"""
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0

    def __post_init__(self):
        if not all(math.isfinite(float(v)) for v in self.as_tuple()):
            raise ValueError("接口力和力矩必须全部是有限数字")

    def as_tuple(self):
        return (self.fx, self.fy, self.fz, self.mx, self.my, self.mz)

    def __add__(self, other):
        if not isinstance(other, ForceTorque):
            return NotImplemented
        return ForceTorque(*(a + b for a, b in zip(self.as_tuple(), other.as_tuple())))


@dataclass(frozen=True)
class SensorSample:
    """传感接口输出（坐标全部为飞行机体系）。

    image_rgb: (H,W,3) uint8 NumPy 数组；取帧间隔内保持最近一帧，
               初次捕获之前为 None。其时间戳为 image_time_s。
    acceleration_xyz_mps2: 数值微分 dv_world/dt 转到当前飞行机体系；
               包含重力，区别于真实 IMU 传感器测量的比力。
    angular_velocity_xyz_rad_s: 飞行机体系角速度 (rad/s)，非欧拉角变化率。
    """
    time_s: float
    acceleration_xyz_mps2: tuple
    angular_velocity_xyz_rad_s: tuple
    acceleration_valid: bool
    image_rgb: Optional[np.ndarray] = None
    image_time_s: Optional[float] = None
    coordinate_frame: str = "dart_flight_body"


def _quat_product(a, b):
    wa, xa, ya, za = (float(x) for x in a)
    wb, xb, yb, zb = (float(x) for x in b)
    return (wa*wb-xa*xb-ya*yb-za*zb,
            wa*xb+xa*wb+ya*zb-za*yb,
            wa*yb-xa*zb+ya*wb+za*xb,
            wa*zb+xa*yb-ya*xb+za*wb)


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
    q_flight = _quat_product(q_stl, _STL_TO_FLIGHT_Q)
    w, x, y, z = q_flight
    return _quat_rotate((w, -x, -y, -z), vector_world)


def _flight_to_world(q_stl, vector_flight):
    q_flight = _quat_product(q_stl, _STL_TO_FLIGHT_Q)
    return _quat_rotate(q_flight, vector_flight)


def _stl_omega_to_flight(omega_stl):
    # R_y(+90°) @ omega_STL
    x, y, z = omega_stl
    return (float(z), float(y), -float(x))


class DartInterfaces:
    """统一管理飞镖传感、主动输入、被动输入三条通道。

    active_provider / passive_provider:
        Callable[[SensorSample], ForceTorque]，每个物理步前调用。
        也可用 set_active_input / set_passive_input 写入保持值；
        调用 set_*_input 会取消对应的 provider，方便其他模块直接接管。

    listener: Callable[[SensorSample], None]，按仿真时间限速发布，
        与物理循环解耦；latest_sensor 在每个物理步都更新。

    注意：此对象由运行 MuJoCo 的 Tk 主线程驱动。外部线程需自行同步。
    """
    def __init__(self, model, data, body_id, qpos_adr, qvel_adr,
                 active_provider=None, passive_provider=None, sensor_listener=None,
                 image_camera="dart_fpv", image_width=320, image_height=240,
                 image_fps=30.0, publish_interval_s=1.0/30.0,
                 auto_capture_image=True):
        self.model = model
        self.data = data
        self.body_id = int(body_id)
        self.qpos_adr = int(qpos_adr)
        self.qvel_adr = int(qvel_adr)
        self.active_provider = active_provider
        self.passive_provider = passive_provider
        self.sensor_listener = sensor_listener
        self.active_input = ForceTorque()
        self.passive_input = ForceTorque()
        self.last_active = ForceTorque()
        self.last_passive = ForceTorque()
        self.last_total = ForceTorque()
        self.latest_sensor = None
        self.image_camera = image_camera
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.image_fps = float(image_fps)
        self.publish_interval_s = float(publish_interval_s)
        self.auto_capture_image = bool(auto_capture_image)
        self._renderer = None
        self.camera_error = None
        self._last_image_time = -math.inf
        self._last_publish_time = -math.inf
        self._frame_rgb = None
        self._frame_time_s = None
        if self.image_fps <= 0.0 or self.publish_interval_s <= 0.0:
            raise ValueError("图像帧率和传感发布间隔必须大于零")
        if self.image_width < 1 or self.image_height < 1:
            raise ValueError("图像尺寸必须大于零")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, image_camera) < 0:
            raise RuntimeError(f"飞镖 FPV 摄像机不存在：{image_camera}")
        self.reset()

    def _q_stl(self):
        return self.data.qpos[self.qpos_adr+3:self.qpos_adr+7]

    def set_active_input(self, fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
        """主动接口：直接设置保持值，取消主动 provider。"""
        self.active_input = ForceTorque(fx, fy, fz, mx, my, mz)
        self.active_provider = None

    def set_passive_input(self, fx=0.0, fy=0.0, fz=0.0, mx=0.0, my=0.0, mz=0.0):
        """被动接口：直接设置保持值，取消被动 provider。"""
        self.passive_input = ForceTorque(fx, fy, fz, mx, my, mz)
        self.passive_provider = None

    def set_active_provider(self, callback):
        self.active_provider = callback

    def set_passive_provider(self, callback):
        self.passive_provider = callback

    def reset(self):
        """重置控制值、力矩与采样缓存，防止旧一局残余六维力影响新一局。"""
        self.active_input = ForceTorque()
        self.passive_input = ForceTorque()
        self.last_active = ForceTorque()
        self.last_passive = ForceTorque()
        self.last_total = ForceTorque()
        self._last_publish_time = -math.inf
        self._last_image_time = -math.inf
        self._frame_rgb = None
        self._frame_time_s = None
        self.data.xfrc_applied[self.body_id, :] = 0.0
        self.latest_sensor = self._new_sample((0.0, 0.0, 0.0), valid=False)

    def _new_sample(self, acceleration, valid):
        omega = self.data.qvel[self.qvel_adr+3:self.qvel_adr+6]
        return SensorSample(
            time_s=float(self.data.time),
            acceleration_xyz_mps2=tuple(float(v) for v in acceleration),
            angular_velocity_xyz_rad_s=_stl_omega_to_flight(omega),
            acceleration_valid=bool(valid),
            image_rgb=self._frame_rgb,
            image_time_s=self._frame_time_s,
        )

    def read_sensor(self):
        """传感接口：最新的仿真状态及最近一帧 RGB，均为只读快照。"""
        return self.latest_sensor

    def before_step(self):
        """在 mj_step 之前执行：同一状态求两路输入，线性相加并施加。"""
        sensor = self.latest_sensor
        active = self.active_provider(sensor) if self.active_provider else self.active_input
        passive = self.passive_provider(sensor) if self.passive_provider else self.passive_input
        if not isinstance(active, ForceTorque) or not isinstance(passive, ForceTorque):
            raise TypeError("主动/被动控制函数必须返回 ForceTorque")
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

    def capture_image_if_due(self, force=False):
        """主线程中调用；调用方持有 viewer.lock()，以防渲染时 data 被改动。

        真正离屏渲染 model.xml 内现有 dart_fpv 摄像机，不是占位图。
        缓存图像 RGB uint8 (H,W,3)。对大场地仅每 1/image_fps 秒仿真时间取一次。
        """
        if not force and not self.auto_capture_image:
            return False
        now = float(self.data.time)
        if (not force and now - self._last_image_time < 1.0 / self.image_fps - 1e-9):
            return False
        if self.camera_error is not None:
            return False
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
            self._last_image_time = now
            self.latest_sensor = replace(
                self.latest_sensor, image_rgb=frame, image_time_s=now
            )
            return True
        except Exception as error:
            # 如果本机 OpenGL/离屏渲染不可用，传感数值与动力学仍可正常运行。
            self.camera_error = f"{type(error).__name__}: {error}"
            print(f"[传感接口] FPV 离屏渲染失败：{self.camera_error}")
            print("[传感接口] 线加速度/角速度仍有效；请检查 OpenGL 环境。")
            self.auto_capture_image = False
            return False

    def publish_sensor_if_due(self):
        """按仿真时间限速执行用户输出示例；不在锁内调用外部 listener。"""
        if self.sensor_listener is None:
            return
        now = self.latest_sensor.time_s
        if now - self._last_publish_time < self.publish_interval_s - 1e-9:
            return
        self._last_publish_time = now
        self.sensor_listener(self.latest_sensor)

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
