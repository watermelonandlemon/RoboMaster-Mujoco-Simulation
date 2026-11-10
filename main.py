# -*- coding: utf-8 -*-
from pathlib import Path
import mujoco
import lib

HERE = Path(__file__).resolve().parent
SCENE = HERE / "scene_color.xml"

# ========== 加载场景 ==========
model = mujoco.MjModel.from_xml_path(str(SCENE))

# ========== 飞镖初始姿态（度）==========
YAW = 0.0
PITCH = 0.0
ROLL = 0.0

# 设置飞镖姿态
lib.set_dart_attitude(model,yaw=YAW,pitch=PITCH,roll=ROLL)

# ========== 初始化仿真 ==========
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)

# ========== 启动查看器 ==========
lib.run_viewer(model, data)

