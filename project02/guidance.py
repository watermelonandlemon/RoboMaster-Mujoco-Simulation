# -*- coding: utf-8 -*-
"""有限时域线性 MPC · 世界系靶心输入，飞行机体系力输出。"""
import math

EPS = 1e-9
_SQRT_HALF = math.sqrt(0.5)
DEFAULT_PARAMS = {
    "max_accel_mps2": 20.0,
    "mpc_steps": 20,
    "gravity_world_mps2": (0.0, 0.0, -9.81),
    # 目标是到达靶心，不是把相对速度也减到零。以下为本仿真离散代价的权重。
    "mpc_q_position": 0.3,
    "mpc_q_velocity": 0.03,
    "mpc_q_terminal_position": 1_000_000.0,
    "mpc_q_terminal_velocity": 0.0,
    "mpc_r_control": 1.0,
}


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, k):
    return tuple(x * k for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


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
    v2.6 镖靶只沿世界 Y 轴移动，因此只估计 Y 分量。
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


def _mpc_gains(t_go, params):
    """Finite horizon relative-motion LQT with a constant known disturbance.

    x=[r,v_rel]. x_next=A*x+B*u-B*d, d=a_target-g; returns the
    first control u=-(k_pos*r+k_vel*v_rel+k_dist*d), for each normal axis.
    The cost is x_N' F x_N + sum(x' Q x + R*u**2) over N controls.
    """
    steps = max(4, int(params["mpc_steps"]))
    # 使用本次相遇的剩余时间；调用处已有 0.05~3 s 数值保护。
    dt = t_go / steps
    q0, q1 = params["mpc_q_position"], params["mpc_q_velocity"]
    r = float(params["mpc_r_control"])
    if not math.isfinite(r) or r <= 0.0:
        raise ValueError("mpc_r_control 必须为有限正数")
    p00, p01, p11 = params["mpc_q_terminal_position"], 0.0, params["mpc_q_terminal_velocity"]
    if not all(math.isfinite(x) and x >= 0.0 for x in (q0, q1, p00, p11)):
        raise ValueError("MPC 状态权重必须为非负有限数")
    l0 = l1 = 0.0  # value function: x' P x + 2*d*l' x + constant
    b0, b1 = -0.5 * dt * dt, -dt
    for _ in range(steps):
        pb0, pb1 = p00 * b0 + p01 * b1, p01 * b0 + p11 * b1
        h = r + b0 * pb0 + b1 * pb1
        m0, m1 = pb0, dt * pb0 + pb1   # B' P A
        f0, f1 = l0 - pb0, l1 - pb1     # l + P*(-B)
        k0, k1, kd = m0 / h, m1 / h, (b0 * f0 + b1 * f1) / h
        l0, l1 = f0 - m0 * kd, dt * f0 + f1 - m1 * kd
        p00, p01, p11 = (
            q0 + p00 - m0 * k0,
            dt * p00 + p01 - m0 * k1,
            q1 + dt * dt * p00 + 2 * dt * p01 + p11 - m1 * k1,
        )
    return k0, k1, kd


def compute_guidance_acceleration(
    missile_position,
    missile_velocity,
    target_position,
    target_velocity=(0.0, 0.0, 0.0),
    target_acceleration=(0.0, 0.0, 0.0),
    params=None,
):
    """
    计算世界坐标系下的期望法向附加加速度。

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

    # 在当前速度的法平面内滚动预测。重力属于已知外部扰动，不是另加的一路力。
    r_normal = project_perpendicular(relative_position, velocity_hat)
    vrel_normal = project_perpendicular(relative_velocity, velocity_hat)
    d_normal = project_perpendicular(sub(a_t, cfg["gravity_world_mps2"]), velocity_hat)
    k0, k1, kd = _mpc_gains(t_go, cfg)
    accel = tuple(
        -(k0 * r_normal[i] + k1 * vrel_normal[i] + kd * d_normal[i])
        for i in range(3)
    )

    # 统一限制为垂直于速度方向的法向附加加速度，便于公平比较。
    accel = project_perpendicular(accel, velocity_hat)
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

    v2.6 的 MuJoCo 接口会把飞行机体系 wrench 变换到世界系再施加。
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
