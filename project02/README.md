# 滑模制导 · Project02（适配 Project01 v3.0）

## 运行

先在原有 `project01` 运行 `python main.py`，本包不含 Project01，也不修改其任何文件。
在本目录另开终端运行 `python active.py`，自动显示六条世界系曲线并写入 CSV；无图模式：`python active.py --no-plot`。
如果要考虑被动空气阻力和恢复力矩，在本目录另开终端运行 `python passive.py`。如果不考虑被动力，确保此前的被动发送程序已停止，并可运行 `python passive.py --zero` 清除接收器旧值。不要同时运行多个 active 或多个 passive 发送端。正常停止使用 Ctrl+C，退出时发送零值。直接杀进程或 UDP 丢包可能留下缓存。

## 文件

- `active.py`：算法入口、同帧真值、实时曲线与自动记录。
- `guidance.py`：本版本原算法、参数与坐标变换。
- `passive.py`：保留原始气动模型，独立发送被动力和恢复力矩；不包含重力。
- `simulator_link.py`：UDP v1 接口（state: 18980，active: 18981，passive: 18982）及帧检查。
- `record.py`：日志工具，由 active/passive 自动调用，不需要独立启动。

## 数据及单位

只订阅完整的 `dart/state`，不订阅图像、视觉真值或 IMU 话题。目标为 `target_position_world_m`（右侧基地碰撞盒 `dart_target_collision_1` 的几何中心）；飞镖位置为 `position_world_m`；对地速度为 `linear_velocity_world_mps`；姿态使用 STL 原生 `[w,x,y,z]` 四元数。靶标速度/加速度在有需要的算法中沿世界 Y 轴用仿真时间差分滤波估计，并非直接发布的真值。纯追踪不需要靶标运动估计。主动力转换到飞行机体系（+X 朝镖头），主动力矩为零。图表实线表示靶心相对位置（世界系 m），虚线表示飞镖对地速度（世界系 m/s）。

## 日志

`logs/<日期时间>_active/run_001.csv`、`logs/<日期时间>_passive/run_001.csv`（若启用），仿真时间回退或识别到仿真进程重启时分表。`config.json` 记录算法参数或被动模型参数；质量、重力和转动惯量为 v3.0 model.xml 中的参考值，不是实时 UDP 读数。

CSV 保留完整 `raw_packet_json` 及展开字段：位置、靶心、速度、姿态、角速度、State 中的加速度、控制器发出的六维量 `command_force_body_*`、接收器缓存 `next_*` 和最近物理步的 `active_force_body/passive_force_body/total_force_body`。另记录仿真时间、接收单调时钟、发送单调时钟、序号、会话、丢帧推断、事件、目标距离以及制导计算诊断。空字段表示未提供/不适用。`passive_command` 行是被动发送时所用最近状态，可能与前一条采样行重复。

## 边界

本次适配保持原七种算法的公式、增益和加速度限幅；不重复施加重力，不修改 Project01 的质量/惯量，不增加主动姿态内环。被动气动参数仍为原联调占位值。状态频率目标 30 Hz，UDP 与物理步异步；无碰撞接触真值或每步施力时刻，CSV 不能单独证明命中或确定实际每步输入。适配验证不是命中率验证。
官方源码参考：https://github.com/watermelonandlemon/RoboMaster-Mujoco-Simulation/tree/v3.0/project01
