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

import math
import socket
import time
import uuid

from bridge_ipc import (
    HOST,
    PASSIVE_PORT,
    TOPIC_STATE,
    SensorSubscriber,
    encode_force_packet,
)


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
    if SEND_HZ <= 0.0:
        raise ValueError("SEND_HZ 必须大于零")

    subscriber = SensorSubscriber(
        topics=(TOPIC_STATE,),
        fields=(
            "linear_velocity_world_mps",
            "orientation_quaternion_wxyz",
        ),
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session_id = uuid.uuid4().hex
    sequence = 0
    period = 1.0 / SEND_HZ
    next_send = time.monotonic()
    last_state_wall = None
    latest_velocity = None
    latest_quaternion = None
    last_print = 0.0
    last_force = (0.0,) * 6

    print(f"[Passive Drag] 订阅 {TOPIC_STATE}，发送 127.0.0.1:{PASSIVE_PORT}")
    print(f"[Passive Drag] 发送频率：{SEND_HZ:g} Hz")
    print(f"[Passive Drag] 空气密度：{AIR_DENSITY:g} kg/m³")
    print(f"[Passive Drag] CdA axial={CDA_AXIAL:g} m², side={CDA_SIDE:g} m²")
    print(f"[Passive Drag] 压力中心相对质心 X={CP_OFFSET_X:g} m")
    print("[Passive Drag] 等待 Full State 数据……")

    try:
        while True:
            # 非阻塞式获取最新状态；只保留最新一帧，旧帧不排队参与计算。
            event = subscriber.recv(timeout=0.002)
            if event is not None and event.get("topic") == TOPIC_STATE:
                data = event.get("data", {})
                velocity = data.get("linear_velocity_world_mps")
                quaternion = data.get("orientation_quaternion_wxyz")

                if (
                    isinstance(velocity, (list, tuple)) and len(velocity) == 3
                    and isinstance(quaternion, (list, tuple)) and len(quaternion) == 4
                ):
                    latest_velocity = tuple(float(v) for v in velocity)
                    latest_quaternion = tuple(float(v) for v in quaternion)
                    last_state_wall = time.monotonic()

            now = time.monotonic()
            if now >= next_send:
                if (
                    latest_velocity is not None
                    and latest_quaternion is not None
                    and last_state_wall is not None
                    and now - last_state_wall <= MAX_STATE_AGE_S
                ):
                    force = calculate_drag_force_body(
                        latest_velocity, latest_quaternion
                    )
                else:
                    # 没有状态或状态过期时主动发送零值，避免旧阻力无限保持。
                    force = (0.0,) * 6

                sequence += 1
                packet = encode_force_packet(
                    "passive", force, sequence, session_id
                )
                sock.sendto(packet, (HOST, PASSIVE_PORT))
                last_force = force

                if now - last_print >= PRINT_INTERVAL_S:
                    if last_state_wall is None:
                        status = "等待状态"
                    elif now - last_state_wall > MAX_STATE_AGE_S:
                        status = "状态超时，已发送零力"
                    else:
                        status = "正常"
                    print(
                        f"[Passive Drag] {status} | "
                        f"F_body=({force[0]:+.4f}, {force[1]:+.4f}, {force[2]:+.4f}) N | "
                        f"M_body=({force[3]:+.4f}, {force[4]:+.4f}, {force[5]:+.4f}) N·m | "
                        f"速度_world={latest_velocity if latest_velocity is not None else '--'}"
                    )
                    last_print = now

                next_send += period
                if next_send < now:
                    next_send = now + period

    except KeyboardInterrupt:
        print("\n[Passive Drag] 停止，发送一次零力清除上一条被动输入。")
        try:
            sequence += 1
            zero_packet = encode_force_packet(
                "passive", (0.0,) * 6, sequence, session_id
            )
            sock.sendto(zero_packet, (HOST, PASSIVE_PORT))
        except OSError:
            pass
    finally:
        subscriber.close()
        sock.close()


if __name__ == "__main__":
    main()
