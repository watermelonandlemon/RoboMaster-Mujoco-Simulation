# Project02：外部控制与传感器监测

Project02 通过 UDP 与 MuJoCo 仿真端通信。**Project02 不导入 Project01 的任何 Python 模块，也不要求两个工程位于同一目录。**

## 1. 文件结构

| 文件 | 作用 |
|---|---|
| `simulator_link.py` | Project02 自己的 UDP 通信客户端：六维力/力矩发送、状态/IMU/图像/视觉真值订阅 |
| `active.py` | 发送主动六维力/力矩 |
| `passive.py` | 订阅飞镖状态，计算空气阻力并发送被动六维力/力矩 |
| `state.py` | 监测飞镖完整状态真值 |
| `IMU.py` | 独立监测 IMU 数据 |
| `video.py` | 接收飞镖第一视角图像，可选叠加同帧绿灯真值 |
| `target_position.py` | 监测对面基地绿灯中心像素坐标，也支持纯 JSON 输出 |

## 2. 与 Project01 的通信关系

两个工程只通过 UDP 通信，互相不需要导入对方的代码。`simulator_link.py` 中的 IP、端口、Topic、JSON 字段和图像分包格式与 Project01 的新接口保持一致。

默认通信参数：

| 项目 | 默认值 |
|---|---|
| 仿真端地址 | `127.0.0.1` |
| 传感器输出端口 | `18980` |
| 主动输入端口 | `18981` |
| 被动输入端口 | `18982` |

默认 `127.0.0.1` 表示两个程序运行在同一台电脑上。若要跨电脑通信，需要将 Project02 的 `simulator_link.py` 中 `HOST` 改为运行 Project01 的电脑 IP，并同步将 Project01 的 UDP 服务绑定地址配置为可被局域网访问的地址；还需确保防火墙放行对应 UDP 端口。

## 3. 运行方式

先独立启动 Project01 的仿真程序。然后打开终端，进入 Project02 文件夹，按需启动下列程序。也可以直接通过脚本的绝对路径启动，不要求 Project01 与 Project02 具有固定的目录关系。

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

各程序独立运行，可按需启动。`active.py` 与 `passive.py` 分别向主动、被动输入通道发送数据。

## 4. 注意事项

- Project02 不会自动启动 Project01；使用前需先运行仿真端。
- `simulator_link.py` 是 Project02 的新 UDP 客户端，不是旧接口兼容层。
- 停止主动/被动输入发送不会自动清零；需要清零时应主动发送六维零值。`passive.py` 在正常按 Ctrl+C 退出时会额外发送一次零力。
