# -*- coding: utf-8 -*-
"""在 PyCharm 的 mujoco Python 环境中运行。"""
from pathlib import Path
import mujoco
import mujoco.viewer

HERE = Path(__file__).resolve().parent
SCENE = HERE / "scene_fast.xml"
print("MuJoCo version:", mujoco.__version__)
print("Loading optimized scene:", SCENE)
model = mujoco.MjModel.from_xml_path(str(SCENE))
print(f"Loaded: {model.nmesh} meshes, {model.ngeom} geoms")
print("打开查看器。可在查看器选择 overview 相机。")
mujoco.viewer.launch(model)
