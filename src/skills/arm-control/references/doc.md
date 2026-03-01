# arm-control 技能参考文档

## 概述

`arm-control` 是 OpenClaw Agent 的核心技能之一，负责控制 Galaxea R1 Lite 双臂机器人执行物理操作。

## 系统架构

```
用户指令 (自然语言)
    │
    ▼
OpenClaw Agent (任务理解与决策)
    │
    ▼
arm-control skill (本技能)
    │
    ├── SDK 模式 ──► 飞特舵机 UART 串口 ──► 机械臂
    │
    └── ROS2 模式 ──► ROS2 话题/服务 ──► 机械臂驱动节点 ──► 机械臂
```

## 机器人规格

| 项目 | 参数 |
|------|------|
| 机型 | Galaxea R1 Lite |
| 臂数 | 2（左臂 + 右臂） |
| 自由度 | 每臂 6 关节 + 1 夹爪，共 14 维 |
| 舵机 | 飞特 ST-3215-C001 (7.4V) |
| 齿轮比 | 1:345 |
| 通信 | UART 串口，波特率 1000000 |
| 电源 | 5.5mm×2.1mm DC 5V 4A |

## 状态空间

机械臂状态为 14 维向量：

| 维度 | 名称 | 类型 | 范围 |
|------|------|------|------|
| 0-5 | 左臂关节 1-6 | float32 | [-π, π] rad |
| 6 | 左臂夹爪 | float32 | [0.0, 1.0] |
| 7-12 | 右臂关节 1-6 | float32 | [-π, π] rad |
| 13 | 右臂夹爪 | float32 | [0.0, 1.0] |

## 控制模式

### 1. SDK 直连模式

直接通过 UART 串口与飞特舵机通信，无需 ROS2 环境。

**适用场景**：
- 快速原型验证
- 无 ROS2 环境的部署
- 低延迟控制需求

**接线说明**：
- 使用 USB-TTL 转接器连接 PC 与舵机控制板
- 串口默认端口：Linux `/dev/ttyUSB0`，Windows `COM3`
- 波特率：1000000 (1Mbps)

**舵机 ID 分配**：
- 左臂关节 1-6：ID 1-6
- 左臂夹爪：ID 7
- 右臂关节 1-6：ID 8-13
- 右臂夹爪：ID 14

### 2. ROS2 模式

通过 ROS2 话题发布控制指令，需要机械臂 ROS2 驱动节点已启动。

**适用场景**：
- 完整的 ROS2 机器人系统
- 需要与其他 ROS2 节点协作
- 使用 MoveIt2 等运动规划

**ROS2 话题**：

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 订阅 | 关节状态反馈 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | 发布 | 关节轨迹指令 |
| `/left_gripper_controller/command` | `std_msgs/Float64` | 发布 | 左夹爪指令 |
| `/right_gripper_controller/command` | `std_msgs/Float64` | 发布 | 右夹爪指令 |

## 命令行使用

### 基本命令

```bash
# 读取机械臂状态
python main.py --mode sdk --port COM3 --action status

# 回到初始位置
python main.py --mode sdk --port /dev/ttyUSB0 --action home

# 紧急停止
python main.py --mode sdk --action stop
```

### 关节控制

```bash
# 设置左臂 6 个关节位置（弧度）
python main.py --mode sdk --action move --arm left --joints 0.1 0.2 0.3 0.0 0.0 0.0

# 设置双臂 12 个关节位置
python main.py --mode sdk --action move --arm both --joints 0.1 0.2 0.3 0.0 0.0 0.0 -0.1 -0.2 -0.3 0.0 0.0 0.0
```

### 夹爪控制

```bash
# 打开左臂夹爪
python main.py --mode sdk --action gripper --arm left --gripper-value 1.0

# 关闭右臂夹爪
python main.py --mode sdk --action gripper --arm right --gripper-value 0.0
```

### 任务执行

```bash
# 倒水任务
python main.py --mode sdk --action pour_water

# 抓取物体（配合预设位置）
python main.py --mode sdk --action pick --arm left --joints 0.5 -0.3 1.0 0.0 0.5 0.0

# 从 Lerobot 轨迹文件执行
python main.py --mode sdk --action trajectory --trajectory-file data/chunk-000/episode_000000.parquet

# 从 numpy 文件执行轨迹
python main.py --mode sdk --action trajectory --trajectory-file trajectory.npy
```

## Python API 使用

```python
from main import create_controller, TaskExecutor, ArmSide
import numpy as np

# 创建 SDK 控制器
ctrl = create_controller(mode="sdk", port="COM3")
ctrl.connect()

# 读取状态
state = ctrl.get_state()
print(f"左臂关节: {state.left_joints}")
print(f"右臂关节: {state.right_joints}")

# 设置关节位置
ctrl.set_joint_positions([0.1, 0.2, 0.3, 0.0, 0.0, 0.0], arm=ArmSide.LEFT)

# 控制夹爪
ctrl.set_gripper(1.0, arm=ArmSide.LEFT)   # 打开
ctrl.set_gripper(0.0, arm=ArmSide.LEFT)   # 关闭

# 回到初始位置
ctrl.go_home()

# 执行轨迹
trajectory = np.random.randn(100, 14).astype(np.float32) * 0.1  # 示例
ctrl.execute_trajectory(trajectory, fps=30)

# 使用任务执行器
executor = TaskExecutor(ctrl)
executor.pour_water()

# 断开连接
ctrl.disconnect()
```

## 与 Lerobot 集成

本技能可直接执行 Lerobot 策略模型生成的动作轨迹。

### 数据格式

Lerobot 数据集使用 parquet 格式存储，其中 `action` 列包含 14 维动作向量：

```
[left_joint_1, ..., left_joint_6, left_gripper,
 right_joint_1, ..., right_joint_6, right_gripper]
```

### 执行流程

```
Lerobot 策略模型
    │
    ▼
生成 action 序列 (T × 14)
    │
    ▼
arm-control.execute_trajectory()
    │
    ▼
逐帧发送关节指令 @ 30 FPS
```

## 安全注意事项

1. **首次测试**：务必先在无负载条件下测试，确认关节运动方向和范围正确
2. **紧急停止**：按 `Ctrl+C` 或执行 `--action stop` 可立即停止所有舵机
3. **电源**：确保供电稳定，多关节同时运动时电流较大
4. **标定**：实际部署前需根据具体机械臂完成舵机 ID 和零位标定
5. **速度限制**：避免过大的关节跳变，建议使用轨迹模式平滑运动

## 飞特舵机通信协议参考

### 数据帧格式

```
[0xFF] [0xFF] [ID] [Length] [Instruction] [Param1] ... [Checksum]
```

### 主要指令

| 指令码 | 名称 | 说明 |
|--------|------|------|
| 0x01 | PING | 查询舵机是否在线 |
| 0x02 | READ | 读取寄存器 |
| 0x03 | WRITE | 写入寄存器 |
| 0x83 | SYNC_WRITE | 同步写入多个舵机 |

### 关键寄存器地址

| 地址 | 长度 | 说明 |
|------|------|------|
| 0x21 | 1 | 工作模式（0=位置, 1=速度, 2=PWM） |
| 0x28 | 1 | 扭矩使能（0=关闭, 1=打开） |
| 0x2A | 2 | 目标位置（0-4095） |
| 0x2C | 2 | 运行时间 |
| 0x2E | 2 | 运行速度 |
| 0x38 | 2 | 当前位置（只读） |
| 0x3A | 2 | 当前速度（只读） |
| 0x3C | 2 | 当前负载（只读） |
