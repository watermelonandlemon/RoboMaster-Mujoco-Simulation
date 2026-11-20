# 更新

与v1.9相比:

## 1.增加 1个“输出接口”+2个“输入接口”

    传感接口(输出):输出飞镖视角画面、三轴线加速度和三轴角速度;
    主动接口(输入):接收外部控制算法给出的三轴修正力和三轴修正力矩;
    被动接口(输入):接收空气阻力、侧风及其他环境因素产生的三轴力和三轴力矩;

### a.传感接口

| 输出数据 | 变量 | 单位 |
|---|---|---|
| 飞镖视角图像 | Image(t) | 图像帧 |
| X 轴线加速度 | ax | m/s² |
| Y 轴线加速度 | ay | m/s² |
| Z 轴线加速度 | az | m/s² |
| X 轴角速度 | ωx | rad/s |
| Y 轴角速度 | ωy | rad/s |
| Z 轴角速度 | ωz | rad/s |

传感数据向量
`S(t) = [ax, ay, az, ωx, ωy, ωz]`

传感器默认不进行重力补偿,即输出含g的加速度。

### 2. 主动接口

| 控制量 | 变量 | 单位 |
|---|---|---|
| X 轴修正力 | Fx_active | N |
| Y 轴修正力 | Fy_active | N |
| Z 轴修正力 | Fz_active | N |
| X 轴修正力矩 | Mx_active | N·m |
| Y 轴修正力矩 | My_active | N·m |
| Z 轴修正力矩 | Mz_active | N·m |

主动控制向量
`A(t) = [Fx_active, Fy_active, Fz_active, Mx_active, My_active, Mz_active]`

主动修正力用于改变飞镖的运动轨迹，主动修正力矩用于改变飞镖的角速度和姿态。

### 3. 被动接口

| 物理量 | 变量 | 单位 |
|---|---|---|
| X 轴被动力 | Fx_passive | N |
| Y 轴被动力 | Fy_passive | N |
| Z 轴被动力 | Fz_passive | N |
| X 轴被动力矩 | Mx_passive | N·m |
| Y 轴被动力矩 | My_passive | N·m |
| Z 轴被动力矩 | Mz_passive | N·m |

被动力学向量
`P(t) = [Fx_passive, Fy_passive, Fz_passive, Mx_passive, My_passive, Mz_passive]`

被动接口初始值全部为零。

重力已在Mujoco中，不通过被动接口重复施加。

### 4.其他
主动接口与被动接口施加的力和力矩，在同一仿真时刻进行矢量相加。

**合成力**
`F_total = F_active + F_passive`

**合成力矩**
`M_total = M_active + M_passive`

**合成广义力向量**
`W_total = [Fx_total, Fy_total, Fz_total, Mx_total, My_total, Mz_total]`


## 2.飞镖的完整运动状态定义

**`X = (r, v, q, ω)`**

| 状态 | 变量 | 含义 |
|---|---|---|
| 质心位置 r | `x、y、z` | 飞镖质心的三维位置 |
| 线速度 v | `Vx、Vy、Vz` | 飞镖质心的三轴速度 |
| 姿态 q | `qw、qx、qy、qz` | 四元数表示飞镖姿态 |
| 角速度 ω | `ωx、ωy、ωz` | 飞镖绕三个机体轴的角速度 |

共使用13个数保存状态，其中四元数满足单位长度约束，因此实际具有12个独立状态变量。

对外显示的姿态仍使用三个欧拉角(Yaw/Pitch/Roll)。

MuJoCo内部使用四元数计算姿态，显示时再转换为欧拉角，避免欧拉角奇异性影响动力学求解。

## 3.运行效果
![运行效果图](运行效果图.gif)


## 4.神经网络舵机控制方案

### 1. 网络输入

神经网络输入为 25 维,输入向量由以下四部分构成：

**① 飞镖完整运动状态: 12 维**

质心位置`[x, y, z]`

质心线速度`[Vx, Vy, Vz]`

姿态角`[Yaw, Pitch, Roll]`

机体角速度`[ωx, ωy, ωz]`

状态向量
`X(t) = [x, y, z, Vx, Vy, Vz, Yaw, Pitch, Roll, ωx, ωy, ωz]`

**② 主动控制量: 6 维**

`A(t) = [Fx_active, Fy_active, Fz_active, Mx_active, My_active, Mz_active]`

表示希望主动控制系统产生的修正力和修正力矩。

**③ 被动力学作用: 6 维**

`P(t) = [Fx_passive, Fy_passive, Fz_passive, Mx_passive, My_passive, Mz_passive]`

表示当前环境和被动气动特性产生的力和力矩。

**④ 仿真时间: 1 维**

`[t]`

完整输入向量：

`U(t) = [X(t), A(t), P(t), t]`

### 2. 网络输出

假设飞镖安装 n 个独立控制舵机，则输出向量为
`Y(t) = [δ1, δ2, ..., δn]`

其中 δ 表示各舵机的目标角度。

### 3. 其他

此神经网络解决的问题是“已知希望飞镖受到多大的主动修正力和修正力矩,应该如何控制各个舵机?”

实际传感器存在误差,视觉输入与识别也存在误差,还需要对输入进行处理;

“希望飞镖受到多大的主动修正力和修正力矩”由制导算法或制导神经网络得到;
