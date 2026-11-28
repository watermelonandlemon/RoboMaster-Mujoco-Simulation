# -*- coding: utf-8 -*-
"""PN 比例导引 · 世界系靶心输入，飞行机体系力输出。"""
import math

GRAVITY_WORLD_MPS2 = (0.0, 0.0, -9.81)
EPS = 1e-9
_SQRT_HALF = math.sqrt(0.5)
DEFAULT_PARAMS = {
    "max_accel_mps2": 20.0,
    "navigation_constant": 3.0,
}


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(a):
    return math.sqrt(max(0.0, dot(a, a)))


def unit(a):
    length = norm(a)
    if length < EPS:
        return (0.0, 0.0, 0.0)
    return scale(a, 1.0 / length)


def clamp(value, low, high):
    return max(low, min(high, value))


def project_perpendicular(vector, axis_unit):
    """将 vector 投影到垂直于 axis_unit 的平面。"""
    return sub(vector, scale(axis_unit, dot(vector, axis_unit)))


def quat_normalize(q):
    q = tuple(float(x) for x in q)
    n = math.sqrt(sum(x * x for x in q))
    if n < EPS:
        raise ValueError("姿态四元数无效")
    return tuple(x / n for x in q)


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_rotate(q, vector):
    """单位四元数：将局部向量旋转到父坐标系。"""
    w, x, y, z = quat_normalize(q)
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


class TargetMotionEstimator:
    """
    由连续目标世界位置估计目标速度和加速度。
    v3.0 镖靶只沿世界 Y 轴移动，因此只估计 Y 分量。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_position = None
        self.last_time = None
        self.velocity = (0.0, 0.0, 0.0)
        self.acceleration = (0.0, 0.0, 0.0)

    def update(self, position, sim_time):
        position = tuple(float(x) for x in position)
        sim_time = float(sim_time)

        if self.last_position is None or self.last_time is None:
            self.last_position = position
            self.last_time = sim_time
            return self.velocity, self.acceleration

        dt = sim_time - self.last_time
        if dt <= 1e-4 or dt > 0.25:
            self.last_position = position
            self.last_time = sim_time
            self.velocity = (0.0, 0.0, 0.0)
            self.acceleration = (0.0, 0.0, 0.0)
            return self.velocity, self.acceleration

        raw_vy = (position[1] - self.last_position[1]) / dt
        raw_vy = clamp(raw_vy, -1.5, 1.5)
        if abs(raw_vy) < 0.015:
            raw_vy = 0.0

        old_vy = self.velocity[1]
        filtered_vy = old_vy + 0.35 * (raw_vy - old_vy)

        raw_ay = clamp((filtered_vy - old_vy) / dt, -5.0, 5.0)
        filtered_ay = self.acceleration[1] + 0.25 * (
            raw_ay - self.acceleration[1]
        )

        self.velocity = (0.0, filtered_vy, 0.0)
        self.acceleration = (0.0, filtered_ay, 0.0)
        self.last_position = position
        self.last_time = sim_time
        return self.velocity, self.acceleration


def _pn_acceleration(relative_position, relative_velocity, velocity_hat,
                     closing_speed, navigation_constant):
    if closing_speed <= 0.0:
        return (0.0, 0.0, 0.0)
    r2 = max(dot(relative_position, relative_position), 1e-9)
    los_angular_velocity = scale(
        cross(relative_position, relative_velocity), 1.0 / r2
    )
    # 沿速度法平面的三维比例导引：a = N * Vc * (omega_LOS × V_hat)。
    return scale(
        cross(los_angular_velocity, velocity_hat),
        navigation_constant * closing_speed,
    )


def compute_guidance_acceleration(
    missile_position,
    missile_velocity,
    target_position,
    target_velocity=(0.0, 0.0, 0.0),
    target_acceleration=(0.0, 0.0, 0.0),
    params=None,
):
    """
    计算世界系主动法向加速度（PN 净法向需求减去法向重力，再统一限幅）。

    返回：
        acceleration_world_mps2, diagnostics
    """
    p_m = tuple(float(x) for x in missile_position)
    v_m = tuple(float(x) for x in missile_velocity)
    p_t = tuple(float(x) for x in target_position)
    v_t = tuple(float(x) for x in target_velocity)
    a_t = tuple(float(x) for x in target_acceleration)

    cfg = dict(DEFAULT_PARAMS)
    if params:
        cfg.update(params)

    speed = norm(v_m)
    relative_position = sub(p_t, p_m)
    distance = norm(relative_position)

    diagnostics = {
        "range_m": distance,
        "speed_mps": speed,
        "closing_mps": 0.0,
        "t_go_s": None,
        "accel_mps2": 0.0,
    }

    if distance < 0.12 or speed < 0.5:
        return (0.0, 0.0, 0.0), diagnostics

    velocity_hat = unit(v_m)
    los_hat = unit(relative_position)
    relative_velocity = sub(v_t, v_m)
    closing_speed = -dot(los_hat, relative_velocity)
    diagnostics["closing_mps"] = closing_speed

    t_go = distance / max(closing_speed, max(0.5, speed * 0.25))
    t_go = clamp(t_go, 0.05, 3.0)
    diagnostics["t_go_s"] = t_go

    if closing_speed <= 0.0:  # 已越过最近点，不再施加制导力
        return (0.0, 0.0, 0.0), diagnostics

    accel = _pn_acceleration(
        relative_position, relative_velocity, velocity_hat,
        closing_speed, float(cfg["navigation_constant"]),
    )

    # PN 给出期望的净法向加速度；仿真引擎仍会独立施加重力。
    # 用主动力抵消重力的法向分量，不消除切向重力，不增加轴向推力。
    # PN 和前馈共用下方的 20 m/s²（默认）总主动加速度预算。
    accel = project_perpendicular(sub(accel, GRAVITY_WORLD_MPS2), velocity_hat)
    accel_norm = norm(accel)
    max_accel = max(0.0, float(cfg["max_accel_mps2"]))
    if accel_norm > max_accel > 0.0:
        accel = scale(accel, max_accel / accel_norm)
        accel_norm = max_accel
    elif max_accel == 0.0:
        accel = (0.0, 0.0, 0.0)
        accel_norm = 0.0

    diagnostics["accel_mps2"] = accel_norm
    return accel, diagnostics


def world_acceleration_to_flight_body_force(acceleration_world, quaternion_stl,
                                            mass_kg=0.3):
    """
    将期望世界系附加加速度换算为飞行机体系主动等效力。

    v3.0 的 MuJoCo 接口会把飞行机体系 wrench 变换到世界系再施加。
    这里仅生成力；主动力矩为零，不强制机体 X 轴与速度方向重合。
    """
    q_stl = quat_normalize(quaternion_stl)

    # STL 长轴沿 +Z；飞行机体系 +X 指向镖头。
    q_stl_to_flight = (_SQRT_HALF, 0.0, -_SQRT_HALF, 0.0)
    q_flight_world = quat_normalize(quat_mul(q_stl, q_stl_to_flight))
    acceleration_body = quat_rotate(
        quat_conjugate(q_flight_world),
        tuple(float(x) for x in acceleration_world),
    )
    return scale(acceleration_body, float(mass_kg))
