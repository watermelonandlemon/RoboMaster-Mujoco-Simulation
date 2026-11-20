# -*- coding: utf-8 -*-
from bridge_ipc import send_force_loop

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
    send_force_loop("passive", force)

if __name__ == "__main__":
    main()
