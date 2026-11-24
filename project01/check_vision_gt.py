# -*- coding: utf-8 -*-
"""在本机 MuJoCo 环境自检：python check_vision_gt.py [--render]。"""
from pathlib import Path
import argparse
import numpy as np
import mujoco

from vision_gt import VisionGroundTruth
from lib import set_dart_attitude, DEFAULT_YAW, DEFAULT_PITCH, DEFAULT_ROLL


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--render", action="store_true", help="额外用渲染器对象ID核对基地绿灯中心像素")
    args = parser.parse_args()
    model = mujoco.MjModel.from_xml_path(str(Path(__file__).with_name("model.xml")))
    quat = set_dart_attitude(model, DEFAULT_YAW, DEFAULT_PITCH, DEFAULT_ROLL)
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "dart_free")
    qadr = int(model.jnt_qposadr[jid])
    data = mujoco.MjData(model)
    data.qpos[qadr + 3:qadr + 7] = quat
    mujoco.mj_forward(model, data)
    truth = VisionGroundTruth(model, data)
    first = truth.sample()["targets"]["base"]
    print("默认姿态基地绿灯：", first)

    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_right_target")
    mocap_id = int(model.body_mocapid[body_id])
    initial = data.mocap_pos[mocap_id].copy()
    initial_point = data.xpos[body_id].copy()
    for dy in (-0.28, 0.0, 0.28):
        data.mocap_pos[mocap_id] = initial + [0.0, dy, 0.0]
        mujoco.mj_forward(model, data)
        point = data.xpos[body_id]
        assert np.allclose(point - initial_point, [0.0, dy, 0.0], atol=1e-9)
        result = truth.sample()["targets"]["base"]
        print(f"mocap Y偏移 {dy:+.2f}m：", result)
    data.mocap_pos[mocap_id] = initial
    mujoco.mj_forward(model, data)

    if args.render:
        # 设置相机朝向基地灯面中心，并检查预测像素对应的渲染对象ID。
        from vision_gt import base_led_world_position
        point = base_led_world_position(data.xpos[body_id], data.xmat[body_id].reshape(3, 3))
        eye = point + np.array([-3.0, 0.0, 1.0])
        direction = point - eye
        yaw = np.degrees(np.arctan2(direction[1], direction[0]))
        pitch = np.degrees(np.arctan2(direction[2], np.hypot(direction[0], direction[1])))
        quat = set_dart_attitude(model, yaw=yaw, pitch=pitch)
        rot = np.empty(9)
        mujoco.mju_quat2Mat(rot, quat)
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "dart_fpv")
        data.qpos[qadr:qadr + 3] = eye - rot.reshape(3, 3) @ model.cam_pos[camera_id]
        data.qpos[qadr + 3:qadr + 7] = quat
        mujoco.mj_forward(model, data)
        result = truth.sample()["targets"]["base"]
        assert result["visible"], result
        renderer = mujoco.Renderer(model, height=480, width=640)
        try:
            renderer.enable_segmentation_rendering()
            renderer.update_scene(data, camera="dart_fpv")
            labels = renderer.render()
            u, v = int(np.floor(result["u"] + 0.5)), int(np.floor(result["v"] + 0.5))
            expected = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "base_right_target_visual_03")
            assert int(labels[v, u, 0]) == expected, (u, v, labels[v, u].tolist(), expected)
            print(f"[PASS] 基地绿灯像素({u},{v})对象ID与绿灯Mesh一致")
        finally:
            renderer.close()


if __name__ == "__main__":
    main()
