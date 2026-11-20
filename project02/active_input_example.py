# -*- coding: utf-8 -*-
from bridge_ipc import send_force_loop

def main():

    # 主动修正力，单位 N
    Fx = 0.0
    Fy = 0.0
    Fz = 0.0

    # 主动修正力矩，单位 N·m
    Mx = 0.0
    My = 0.0
    Mz = 0.0

    force = [Fx, Fy, Fz, Mx, My, Mz]
    send_force_loop("active", force)

if __name__ == "__main__":
    main()