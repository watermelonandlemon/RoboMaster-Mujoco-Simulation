# -*- coding: utf-8 -*-
"""主动六维力/力矩发布示例：python active.py。"""
from simulator_link import send_force_loop

SEND_HZ = 10.0  # 主动发送频率 Hz，可独立修改

def main():
    # 主动修正力，单位 N
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0

    # 主动修正力矩，单位 N·m
    Mx = 0.0
    My = 0.001
    Mz = 0.0

    force = [Fx, Fy, Fz, Mx, My, Mz]
    send_force_loop("active", force, hz=SEND_HZ)

if __name__ == "__main__":
    main()
