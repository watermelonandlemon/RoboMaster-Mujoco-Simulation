# Project02：外部控制与传感器监测

Project02 通过 UDP 与 Project01 的 MuJoCo 仿真端通信，提供主动/被动六维力与力矩输入，以及飞镖状态、IMU、第一视角图像和视觉真值订阅。两个工程仅通过 UDP 交换数据，不相互导入 Python 模块，目录位置互相独立。

## 1. 文件结构

| 文件 | 作用 |
|---|---|
| `simulator_link.py` | UDP 通信客户端：订阅传感器话题、发布六维力/力矩、重组图像数据。 |
| `active.py` | 按设定频率发送主动六维力/力矩。 |
| `passive.py` | 根据飞镖速度和姿态计算简化空气阻力及气动力矩，并发送被动输入。 |
| `state.py` | 显示飞镖位置、速度、姿态、角速度及六维力/力矩。 |
| `IMU.py` | 显示三轴线加速度（m/s²）和三轴角速度（rad/s）。 |
| `video.py` | 显示飞镖第一视角图像，可选叠加同帧视觉真值。 |
| `target_position.py` | 显示基地绿灯中心像素坐标，支持 JSON 输出。 |

## 2. 通信接口

默认仿真端地址：`127.0.0.1`

| 用途 | UDP 端口 |
|---|---:|
| 传感器数据与订阅请求 | `18980` |
| 主动输入 | `18981` |
| 被动输入 | `18982` |

传感器话题：

- `dart/state`：飞镖完整状态真值。
- `dart/imu`：IMU 加速度与角速度。
- `dart/image`：飞镖第一视角 RGB 图像。
- `dart/vision_gt`：仿真端输出的目标像素坐标真值。

六维力/力矩顺序为 `[Fx, Fy, Fz, Mx, My, Mz]`，单位分别为 N、N·m。主动和被动输入使用独立端口。

跨电脑运行时，将 `simulator_link.py` 中的 `HOST` 改为 Project01 所在电脑的 IP，并确保网络及防火墙允许对应 UDP 通信。

## 3. 运行方式

先启动 Project01，再进入 Project02 文件夹，按需运行：

```bash
python active.py
python passive.py
python state.py
python IMU.py
python video.py
python video.py --show-gt
python target_position.py
python target_position.py --console
```

各程序独立运行，可同时启动。图形界面需要 Python Tkinter。

## 4. 参数与注意事项

- `active.py`：修改 `SEND_HZ` 和 `main()` 中的六维数组，调整主动输入频率及各轴力/力矩；默认频率为 10 Hz。
- `passive.py`：可调整 `SEND_HZ`、`AIR_DENSITY`、`CDA_AXIAL`、`CDA_SIDE`、`CP_OFFSET_X` 和 `MAX_STATE_AGE_S`；默认发送频率为 30 Hz。
- 当前空气阻力模型假设无风，计算纵向/横向阻力及压力中心产生的力矩；暂未考虑独立升力、舵面效应和旋转阻尼。Cd·A 与压力中心参数仅为联调占位值，尚未经过真实气动标定。
- `state.py` 和 `dart/vision_gt` 提供的是仿真真值，不等同于实际机载传感器或视觉算法的输出。`target_position.py` 当前显示基地绿灯中心坐标。
- `video.py --show-gt` 只叠加 `frame_id` 和 `time_s` 均匹配的真值，不用其他帧坐标代替。
- 停止 `active.py` 不会自动清零；需要归零时应主动发送六维零值。`passive.py` 正常按 `Ctrl+C` 退出时会发送一次零值；状态数据超过 `MAX_STATE_AGE_S` 未更新时也会发送零值。
