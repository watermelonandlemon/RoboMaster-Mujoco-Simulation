# RoboMaster 2027 MuJoCo 飞镖仿真

## 1. 工程结构

三个核心函数库：仿真、对外输入、对外输出。

| 文件 | 定位 | 主要职责 |
|---|---|---|
| `main.py` | 程序入口 | 只负责启动仿真 |
| `mujoco_sim.py` | MuJoCo 仿真函数库 | 模型加载、飞镖运动、镖靶运动、控制面板、物理步进、力/力矩施加、相机、IMU/图像采样及绿灯像素真值计算 |
| `control_input.py` | 对外输入函数库 | 接收外部程序发送的主动/被动六维力与力矩，并提供发送端辅助函数 |
| `sensor_output.py` | 对外输出函数库 | 发布飞镖状态、IMU、FPV 图像、基地绿灯像素真值，并提供订阅客户端 |
| `model.xml` | 场景模型 | MuJoCo 场景、刚体、相机、碰撞体和场地对象定义 |
| `meshes/` | 三维模型 | 场地、飞镖及移动镖靶的网格资源 |

## 2. 系统数据流

```text
外部控制程序
      │
      │ 主动/被动六维力与力矩(UDP)
      ▼
control_input.py
      │
      ▼
mujoco_sim.py  ◄──── model.xml / meshes
      │                
      │                 
      │
      │ 
      ▼
sensor_output.py
      │
      │ UDP 发布/订阅
      ▼
外部视觉、制导与监测程序
```

- `control_input.py`：负责外部数据进入仿真。
- `mujoco_sim.py`：负责仿真内部计算和运行。
- `sensor_output.py`：负责仿真结果对外发布。

## 3. 对外输入接口

输入由 `control_input.py` 提供，主动输入与被动输入分别监听：

| 输入通道 | UDP 端口 | 数据 |
|---|---:|---|
| 主动输入 | `18981` | `[Fx, Fy, Fz, Mx, My, Mz]` |
| 被动输入 | `18982` | `[Fx, Fy, Fz, Mx, My, Mz]` |

力的单位为 N，力矩的单位为 N·m，数据均以飞行机体系表达。仿真内部将主动与被动六维量逐分量相加，再转换到世界坐标系，施加于飞镖质心；MuJoCo 原有重力仍然生效，被动力不包含重力。

输入采用独立 UDP 发布者。后台接收线程约每 10ms 检查数据；每个通道保持最新有效值，多发布者按 session 和 sequence 去重。停止发送不会自动清零，需要主动发送六维零值才能归零。

发送端辅助函数：`send_force_loop(channel, values, hz)`；命令行辅助函数：`parse_force_arguments(description)`。

外部程序发送示例：

```python
from control_input import send_force_loop

# 主动输入：Fx Fy Fz Mx My Mz
send_force_loop("active", (0, 0, 0, 0, 0, 0), hz=30)

# 被动输入使用 channel="passive"，参数格式相同。
```

## 4. 对外输出接口

输出由 `sensor_output.py` 提供，发布端口为 UDP `18980`，支持多个程序同时订阅，并可选择 Topic 和数值字段。

| Topic | 内容 |
|---|---|
| `dart/state` | MuJoCo 真值：质心位置、线速度、四元数、姿态角、角速度、当前主动/被动力等 |
| `dart/imu` | 飞行机体系线加速度、角速度及有效标记 |
| `dart/image` | 飞镖第一视角 RGB 图像，zlib 压缩后分片发送 |
| `dart/vision_gt` | 对面基地绿灯中心的准确像素投影、视野状态和中心视线遮挡结果 |

输出目标频率为 30Hz，实际频率取决于 MuJoCo 仿真、图像渲染和系统负载。没有图像订阅者时不会执行 FPV 离屏渲染；视觉真值订阅独立于图像订阅。

外部程序订阅示例：

```python
from sensor_output import SensorSubscriber, TOPIC_VISION_GT

with SensorSubscriber(topics=(TOPIC_VISION_GT,)) as subscriber:
    while True:
        packet = subscriber.recv(timeout=1.0)
        if packet is not None:
            print(packet)
```

如果同一程序需要接收多种输出，可将对应 Topic 一起传入 `SensorSubscriber(topics=(...))`。
