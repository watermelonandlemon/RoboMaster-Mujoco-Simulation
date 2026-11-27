# RoboMaster 2027 MuJoCo 飞镖仿真

Project01 为独立仿真工程，负责场地建模、飞镖动力学、镖靶运动、传感器模拟及对外通信。外部控制与数据记录程序由 Project02 或其他独立工程实现。

## 1. 工程结构

| 文件 | 主要职责 |
|---|---|
| `main.py` | 仿真程序入口 |
| `mujoco_sim.py` | 仿真函数库：物理步进、控制面板、镖靶运动、力与力矩施加、状态及传感器采样 |
| `control_input.py` | 输入函数库：主动/被动六维力与力矩的接收及发送辅助函数 |
| `sensor_output.py` | 输出函数库：状态、IMU、FPV 图像及绿灯像素真值的发布与订阅 |
| `model.xml` | 场地、飞镖刚体、质量、转动惯量、相机及碰撞体定义 |
| `meshes/` | 静态场地、活动镖靶及飞镖三维模型 |

## 2. 系统数据流

```text
外部程序 ── 主动/被动六维力与力矩 ──► control_input.py
                                             │
                                             ▼
model.xml / meshes ──────────────────► mujoco_sim.py
                                             │
                                             ▼
外部程序 ◄── State / IMU / Image / Vision ── sensor_output.py
```

输入与输出均采用 UDP，默认使用本地地址 `127.0.0.1`，外部工程通过通信接口与仿真交互，无需固定两个工程的相对路径。

## 3. 对外输入接口

| 通道 | UDP 端口 | 数据 |
|---|---:|---|
| 主动输入 | `18981` | `[Fx, Fy, Fz, Mx, My, Mz]` |
| 被动输入 | `18982` | `[Fx, Fy, Fz, Mx, My, Mz]` |

- 力的单位为 N，力矩的单位为 N·m，均采用飞行机体系（+X 指向镖头）。
- 两路输入逐分量相加，转换为世界坐标系后施加于飞镖质心。MuJoCo 内置重力独立生效，被动力不包含重力。
- 后台接收线程约以 100Hz 检查 UDP 数据；每个物理步读取当前最新有效输入。30Hz 是传感器输出的目标频率，不是输入更新频率。
- 各通道按 `session` 和 `sequence` 去重并保留最新有效值，每路最多记录最近 32 个有效会话。淘汰的旧会话不再享有历史序号去重保障。停止发送或重置仿真不会清除接收器缓存，归零需主动发送六维零值。

辅助函数：`send_force_loop(channel, values, hz)`、`parse_force_arguments(description)`。

## 4. 对外输出接口

发布端口为 UDP `18980`，支持多程序独立订阅、Topic 选择及数值字段筛选，目标发布频率为 30Hz。

| Topic | 输出内容 |
|---|---|
| `dart/state` | 飞镖完整状态、右侧基地镖靶碰撞体中心位置、当前及待生效的六维输入 |
| `dart/imu` | 飞行机体系线加速度、角速度及加速度有效标记 |
| `dart/image` | 640×480 飞镖第一视角 RGB 图像，采用 zlib 压缩及 UDP 分片 |
| `dart/vision_gt` | 对面基地绿灯中心的像素投影、视野及中心视线遮挡状态 |

### 4.1 State 主要字段

| 字段 | 含义 |
|---|---|
| `position_world_m` | 飞镖质心世界坐标（m） |
| `target_position_world_m` | 右侧基地镖靶碰撞体 `dart_target_collision_1` 的实时中心世界坐标（m） |
| `linear_velocity_world_mps` | 世界坐标系线速度（m/s） |
| `orientation_quaternion_wxyz` | MuJoCo 原生刚体四元数 `[w,x,y,z]` |
| `orientation_yaw_pitch_roll_deg` | 飞行机体系偏航、俯仰、滚转角（°） |
| `angular_velocity_body_rad_s` | 飞行机体系角速度（rad/s） |
| `active_force_body` / `passive_force_body` / `total_force_body` | 最近物理步实际施加的主动、被动及合计六维量，不含内置重力 |
| `next_active_force_body` / `next_passive_force_body` | 发布时接收器内的最新六维输入快照（不保证下一物理步仍为该值） |

四元数对应 STL 原生刚体坐标系（+Z 指向镖头），不同于 +X 指向镖头的飞行机体系。数值消息的外层还包含 `frame_id`、`time_s`、`topic` 等信息，具体字段位于 `data` 中。`time_s` 为仿真时间，而非真实时钟；判断通信数据是否超时时，应使用接收端的单调时钟记录到包时间。

### 4.2 IMU 与视觉

- IMU 线加速度由仿真速度差分得到，包含重力，并非真实加速度计测量的比力。
- 视觉真值通过相机几何投影与射线检测得到，不依赖 OpenCV 图像识别；像素坐标可为小数，绿灯中心与镖靶碰撞体中心为不同位置。
- 订阅客户端绑定本机临时 UDP 端口，`host` 仅用于指定服务端地址；仿真端仍默认监听本机回环地址。
- 未订阅图像时不执行 FPV 离屏渲染；视觉真值可独立订阅。实际发布频率受仿真、渲染和系统负载影响。

## 5. 仿真参数与运行

- 物理步长：0.2ms；重力加速度：`(0, 0, -9.81)` m/s²。每次 GUI 回调最多推进 100 步，渲染或计算负载过高时仿真时间可能落后于真实时间。
- 飞镖质量：0.3kg；STL 刚体坐标系主转动惯量：`(0.001, 0.001, 0.0001)` kg·m²（估值）。
- 支持初始速度、姿态设定以及固定、随机固定、随机移动、末端移动四种镖靶模式。

安装 `mujoco`、`numpy`，确保 Python 环境支持 Tkinter，在 Project01 中运行：

```bash
python main.py
```
