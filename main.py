# -*- coding: utf-8 -*-
from pathlib import Path
import math
import mujoco
import mujoco.viewer
import lib

HERE = Path(__file__).resolve().parent
SCENE = HERE / "scene_fast.xml"
model = mujoco.MjModel.from_xml_path(str(SCENE))# 加载场景

# ========== 飞镖姿态参数（单位：度）==========
YAW = 0.0       # 偏航角：正值向左转
PITCH = 0.0     # 俯仰角：正值抬头
ROLL = 0.0      # 滚转角：绕飞镖自身长轴旋转
# ===========================================

# 依次叠加偏航、俯仰、滚转和模型方向校正
q = lib.quat_mul(lib.quat_mul(lib.quat_mul(lib.axis_quat("z", YAW),lib.axis_quat("y", -PITCH)),
                              lib.axis_quat("x", ROLL)),lib.axis_quat("y", 90))

model.body("dart_preview").quat[:] = q  # 将姿态设置到飞镖
mujoco.viewer.launch(model)# 打开 MuJoCo
