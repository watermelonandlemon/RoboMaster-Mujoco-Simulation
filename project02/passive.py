# -*- coding: utf-8 -*-
"""
根据 MuJoCo Full State 实时计算飞镖空气阻力，并通过 Passive 接口发送。

运行：
    python passive.py

当前模型：
    1. 假设空气静止、无风。
    2. 飞镖按轴对称物体近似：X 为飞镖纵轴，Y/Z 为横向。
    3. 分别计算纵向阻力和横向阻力。
    4. 横向气动力作用于质心后方的压力中心，产生恢复力矩，使飞镖机头趋向速度方向。
    5. 当前不计算独立升力曲线、舵面效应和旋转阻尼。

注意：
    CDA_AXIAL / CDA_SIDE 是 Cd*A（阻力系数×参考面积），单位 m²。
    CP_OFFSET_X 是压力中心相对质心沿飞镖机体 X 轴的位置，必须为负数（位于尾部）。
    这些参数目前是联调占位值，不是经试验标定的真实气动参数。
"""

import argparse
import math
import time

from simulator_link import ForcePublisher, SensorSubscriber, TOPIC_STATE, FrameTracker
from record import RunRecorder, ZERO


# ======================== 可调参数 ========================

SEND_HZ = 30.0                 # Passive 力发送频率 Hz
AIR_DENSITY = 1.225            # 空气密度 kg/m^3，标准海平面近似值

# Cd*A，单位 m^2。以下仅为联调占位值，需要后续标定。
CDA_AXIAL = 0.0010             # 沿飞镖纵轴方向（机体系 X）的等效阻力面积
CDA_SIDE = 0.0080              # 垂直飞镖纵轴方向（机体系 YZ）的等效阻力面积

# 压力中心相对质心的位置，单位 m；负值表示位于飞镖尾部。
# -0.08 m 仅为联调占位值，必须根据真实质心/压力中心位置校准。
CP_OFFSET_X = -0.08

MAX_STATE_AGE_S = 0.30         # 超过该时间没有收到新状态，则发送零力，避免沿用过期阻力
PRINT_INTERVAL_S = 1.0        # 控制台状态输出间隔


# ======================== 四元数与坐标变换 ========================

_SQRT_HALF = math.sqrt(0.5)

# MuJoCo STL 本体坐标到飞行机体系的固定校正：
# STL 长轴沿 +Z，飞行机体系 +X 指向飞镖机头。
_Q_STL_TO_FLIGHT = (_SQRT_HALF, 0.0, -_SQRT_HALF, 0.0)


def quat_mul(a, b):
    """四元数乘法，顺序表示 a*b；四元数排列为 w,x,y,z。"""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def quat_conjugate(q):
    """单位四元数的共轭，也就是逆旋转。"""
    return (q[0], -q[1], -q[2], -q[3])


def quat_rotate(q, vector):
    """使用单位四元数将 vector 从局部坐标系旋转到父坐标系。"""
    w, x, y, z = q
    vx, vy, vz = vector

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def world_velocity_to_flight_body(velocity_world, quaternion_stl):
    """
    将世界系线速度转换到飞行机体系。

    Full State 中的 quaternion 是 MuJoCo STL 本体四元数；
    需要右乘 STL->飞行机体系的固定校正，再进行逆旋转。
    """
    norm = math.sqrt(sum(float(v) ** 2 for v in quaternion_stl))
    if norm < 1e-12:
        raise ValueError("姿态四元数无效")

    q_stl = tuple(float(v) / norm for v in quaternion_stl)
    q_flight = quat_mul(q_stl, _Q_STL_TO_FLIGHT)

    q_norm = math.sqrt(sum(v * v for v in q_flight))
    q_flight = tuple(v / q_norm for v in q_flight)

    return quat_rotate(
        quat_conjugate(q_flight),
        tuple(float(v) for v in velocity_world),
    )


# ======================== 空气阻力模型 ========================

def calculate_drag_force_body(
    velocity_world,
    quaternion_stl,
    rho=AIR_DENSITY,
    cda_axial=CDA_AXIAL,
    cda_side=CDA_SIDE,
    cp_offset_x=CP_OFFSET_X,
):
    """
    根据世界系速度和 MuJoCo STL 姿态，计算飞行机体系下的六维气动力。

    横向气动力作用于质心后方的压力中心，使用 M = r × F 计算恢复力矩。
    这样飞镖会趋向沿当前速度方向指向；弹道向下弯曲时，机头也会逐渐低下。
    """
    values = tuple(float(v) for v in velocity_world) + tuple(
        float(v) for v in quaternion_stl
    )
    if not all(math.isfinite(v) for v in values):
        raise ValueError("速度或姿态包含非有限数值")
    if rho < 0.0 or cda_axial < 0.0 or cda_side < 0.0:
        raise ValueError("空气密度和 CdA 参数不能为负数")
    if not math.isfinite(cp_offset_x) or cp_offset_x >= 0.0:
        raise ValueError("CP_OFFSET_X 必须为负数，表示压力中心位于质心后方")

    vx, vy, vz = world_velocity_to_flight_body(
        velocity_world, quaternion_stl
    )

    # 纵向阻力。
    fx = -0.5 * rho * cda_axial * abs(vx) * vx

    # 横向阻力。
    speed_side = math.hypot(vy, vz)
    fy = -0.5 * rho * cda_side * speed_side * vy
    fz = -0.5 * rho * cda_side * speed_side * vz

    # 压力中心 r=[cp_offset_x, 0, 0]，相对质心位于尾部。
    # M = r × F：My=-r_x*Fz，Mz=r_x*Fy。
    # 该力矩会使机头趋向速度方向。
    mx = 0.0
    my = -cp_offset_x * fz
    mz = cp_offset_x * fy

    return (fx, fy, fz, mx, my, mz)


# ======================== Passive 实时发布 ========================

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zero", action="store_true", help="只向被动端口发送零值后退出")
    args = parser.parse_args()
    if not math.isfinite(SEND_HZ) or SEND_HZ <= 0.0:
        raise ValueError("SEND_HZ 必须大于零")
    if args.zero:
        with ForcePublisher("passive") as publisher:
            for _ in range(3):
                publisher.send(ZERO)
                time.sleep(0.02)
        print("[Passive] 已发送三次零被动输入；不要同时运行其他 passive 发送端。")
        return
    recorder = RunRecorder("passive", "original_drag", {
        "source": "v2.6 dart/state", "send_hz": SEND_HZ, "rho_kg_m3": AIR_DENSITY,
        "cda_axial_m2": CDA_AXIAL, "cda_side_m2": CDA_SIDE, "cp_offset_x_m": CP_OFFSET_X,
        "gravity_in_command": False, "note": "原气动模型与参数，非实测标定",
    })
    tracker = FrameTracker()
    latest_event = latest_timing = None
    latest_received = None
    latest_force = ZERO
    period = 1.0 / SEND_HZ
    next_send = time.monotonic()
    last_print = 0.0
    print(f"[Passive] 订阅 {TOPIC_STATE}；原阻力及恢复力矩模型；日志 {recorder.directory}", flush=True)
    try:
        with SensorSubscriber(topics=(TOPIC_STATE,)) as subscriber, ForcePublisher("passive") as publisher:
            try:
                publisher.send(ZERO)
                recorder.write(status="startup_zero", command=ZERO, publisher=publisher)
                while True:
                    event = subscriber.recv(timeout=0.002)
                    now = time.monotonic()
                    if event is not None:
                        timing = {}
                        try:
                            frame_status, timing, accepted = tracker.accept(event, now)
                            if accepted:
                                if frame_status in ("reset", "restart"):
                                    recorder.new_run()
                                    latest_event = latest_received = None
                                    latest_force = ZERO
                                data = event.get("data")
                                if not isinstance(data, dict):
                                    raise ValueError("无效 state.data")
                                v = data.get("linear_velocity_world_mps")
                                q = data.get("orientation_quaternion_wxyz")
                                if not all(isinstance(a, (list, tuple)) and len(a) == n and all(
                                        type(x) in (int, float) and math.isfinite(x) for x in a)
                                        for a, n in ((v, 3), (q, 4))):
                                    raise ValueError("无效的世界系速度或姿态")
                                latest_force = calculate_drag_force_body(v, q)
                                if not all(math.isfinite(x) for x in latest_force):
                                    raise ValueError("非有限被动力")
                                latest_event, latest_received, latest_timing = event, now, timing
                                recorder.write(event, frame_status, now, timing)
                            else:
                                recorder.write(event, frame_status, now, timing)
                        except (ValueError, TypeError, OverflowError, KeyError) as exc:
                            latest_event = latest_received = None
                            latest_force = ZERO
                            publisher.send(ZERO)
                            recorder.write(event, "invalid_state_zero", now, timing, command=ZERO,
                                           publisher=publisher, error=str(exc))
                    if now >= next_send:
                        fresh = latest_received is not None and now - latest_received <= MAX_STATE_AGE_S
                        force = latest_force if fresh else ZERO
                        publisher.send(force)
                        recorder.write(latest_event if fresh else None,
                            "passive_command" if fresh else "timeout_zero", latest_received if fresh else None,
                            latest_timing if fresh else None, command=force, publisher=publisher)
                        if now - last_print >= PRINT_INTERVAL_S:
                            print(f"[Passive] {'正常' if fresh else '等待/超时清零'} | "
                                  f"F={tuple(round(x, 4) for x in force[:3])} N | "
                                  f"M={tuple(round(x, 4) for x in force[3:])} N·m", flush=True)
                            last_print = now
                        next_send += period
                        if next_send < now:
                            next_send = now + period
            except KeyboardInterrupt:
                pass
            finally:
                try:
                    publisher.send(ZERO)
                    recorder.write(status="exit_zero", command=ZERO, publisher=publisher)
                except OSError as exc:
                    print(f"[Passive] 退出清零发送失败：{exc}", flush=True)
    finally:
        recorder.close()
        print(f"[CSV] 记录结束：{recorder.directory}", flush=True)


if __name__ == "__main__":
    main()
