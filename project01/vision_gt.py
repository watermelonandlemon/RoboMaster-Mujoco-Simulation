# -*- coding: utf-8 -*-
"""对面基地绿灯中心的几何视觉真值；不读取 RGB、不进行视觉识别。"""
from dataclasses import dataclass
import math

import numpy as np

# 从原始 move2_m03 绿色灯罩网格中确定的灯面中心参考点，
# 坐标位于 base_right_target 的局部坐标系。该 Body 原本就是 mocap body。
# 因而无需在 model.xml 中新增 site 或修改任何模型资产。
BASE_LED_CENTER_LOCAL = np.array([25.488436, 7.499998, 1.243904], dtype=float)
BASE_BODY_NAME = "base_right_target"
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
        import mujoco
        self.mj = mujoco
        self.model, self.data = model, data
        self.camera = str(camera)
        self.width, self.height = int(width), int(height)
        self.camera_id = self._id(mujoco.mjtObj.mjOBJ_CAMERA, self.camera)
        self.body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, BASE_BODY_NAME)
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
