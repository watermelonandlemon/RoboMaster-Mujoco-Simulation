# -*- coding: utf-8 -*-
"""被动六维力/力矩发布示例：独立设置发送频率。"""
from bridge_ipc import send_force_loop

SEND_HZ = 20.0  # 被动发送频率 Hz，可独立修改


def main():
    # 被动力，单位 N（不包含重力）
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0

    # 被动力矩，单位 N·m
    Mx = 0.0
    My = 0.0
    Mz = 0.0

    force = [Fx, Fy, Fz, Mx, My, Mz]
    send_force_loop("passive", force, hz=SEND_HZ)


if __name__ == "__main__":
    main()
