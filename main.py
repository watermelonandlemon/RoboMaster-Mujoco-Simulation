# -*- coding: utf-8 -*-
from pathlib import Path

import mujoco
import mujoco.viewer
import lib


HERE = Path(__file__).resolve().parent
SCENE = HERE / "model.xml"
DART_JOINT_NAME = "dart_free"


def main():
    if not SCENE.is_file():
        raise FileNotFoundError(f"找不到场景文件：{SCENE}")

    print(f"正在加载场景：{SCENE}")
    model = mujoco.MjModel.from_xml_path(str(SCENE))

    # 当前 XML 中飞镖自由关节固定名为 dart_free。
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        DART_JOINT_NAME,
    )
    if joint_id < 0 or model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
        raise RuntimeError(f"场景中找不到自由关节 {DART_JOINT_NAME!r}")

    qpos_adr = int(model.jnt_qposadr[joint_id])
    qvel_adr = int(model.jnt_dofadr[joint_id])

    # 设置默认姿态；运行后可在控制面板中修改三个角度。
    quat = lib.set_dart_attitude(
        model,
        yaw=lib.DEFAULT_YAW,
        pitch=lib.DEFAULT_PITCH,
        roll=lib.DEFAULT_ROLL,
    )

    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)

    initial_qpos = data.qpos.copy()
    initial_qpos[qpos_adr + 3:qpos_adr + 7] = quat

    # 同步模型默认姿态，确保 viewer 的重置操作也使用相同姿态。
    model.qpos0[qpos_adr + 3:qpos_adr + 7] = quat
    data.qpos[:] = initial_qpos
    mujoco.mj_forward(model, data)

    print("飞镖刚体：dart_preview")
    print(f"发射位置沿用场景设置：{initial_qpos[qpos_adr:qpos_adr + 3]}")
    print(
        "初始姿态："
        f"YAW={lib.DEFAULT_YAW}°，"
        f"PITCH={lib.DEFAULT_PITCH}°，"
        f"ROLL={lib.DEFAULT_ROLL}°"
    )

    viewer = mujoco.viewer.launch_passive(model, data)

    try:
        # 自由摄像头观察整个场地中央，初始化后仍可用鼠标自由操作。
        lib.camera_init(viewer)

        app = lib.DartSimulationApp(
            model=model,
            data=data,
            viewer=viewer,
            initial_qpos=initial_qpos,
            qpos_adr=qpos_adr,
            qvel_adr=qvel_adr,
        )
        app.run()
    except Exception:
        if viewer.is_running():
            viewer.close()
        raise


if __name__ == "__main__":
    main()
