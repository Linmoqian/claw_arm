# arm-control 技能参考文档

## 概述

`arm-control` 是 OpenClaw Agent 的核心技能，直接通过本地飞特舵机 SDK 控制 SO100 机械臂，无需 LeRobot 等外部框架。

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
    ▼
SO100Controller (封装层)
    │
    ▼
SDK: PacketHandler + PortHandler (飞特舵机协议)
    │
    ▼
USB 串口 → STS3215 舵机 × 6 → SO100 机械臂
```

## 与旧版本的区别

| 项目 | 旧版 (LeRobot) | 新版 (SDK 直连) |
|------|----------------|-----------------|
| 依赖 | lerobot[feetech] + 校准 | 仅本地 SDK/ 目录 |
| 位置范围 | -100 ~ 100 (归一化) | 0 ~ 4095 (原始舵机值) |
| 通信链路 | LeRobot → scservo_sdk → 串口 | SDK → 串口 (少一层) |
| 校准 | 需要 lerobot-calibrate | 不需要 |
| 扭矩控制 | 隐式 | 显式 (enable/disable_torque) |
| 自由拖动 | 不支持 | 原生支持 (freedrag) |

## 机器人规格

| 项目 | 参数 |
|------|------|
| 机型 | SO100 |
| 自由度 | 6（5 旋转关节 + 1 夹爪） |
| 电机 | 飞特 STS3215 × 6 |
| 电源 | 12V DC 适配器 |
| 通信 | USB 串口 (默认 COM7, 波特率 1000000) |
| 控制方式 | 本地 SDK 直接读写寄存器 |
| 负载限制 | 约 500g |

## 关节说明

| 电机 ID | 关节名称 | 描述 | 范围 | 零位 |
|---------|----------|------|------|------|
| 1 | shoulder_pan | 底部旋转 | 0 ~ 4095 | 2047 |
| 2 | shoulder_lift | 大臂升降 | 0 ~ 4095 | 2047 |
| 3 | elbow_flex | 小臂弯曲 | 0 ~ 4095 | 2047 |
| 4 | wrist_flex | 手腕俯仰 | 0 ~ 4095 | 2047 |
| 5 | wrist_roll | 手腕旋转 | 0 ~ 4095 | 2047 |
| 6 | gripper | 夹爪开合 | 0 ~ 4095 | 2047 |

**位置值说明**：
- 值为舵机原始脉冲位置，范围 0-4095
- 中位 2047 对应关节零点
- 值越大越向一个方向转，值越小越向另一个方向
- 夹爪：值大 → 张开，值小 → 闭合

## STS3215 寄存器参考

| 寄存器名 | 地址 | 长度 | 说明 |
|----------|------|------|------|
| TORQUE_ENABLE | 40 | 1 byte | 0=释放, 1=锁定 |
| GOAL_POSITION | 42 | 2 bytes | 目标位置 (0-4095) |
| MOVING_SPEED | 46 | 2 bytes | 运动速度 |
| PRESENT_POSITION | 56 | 2 bytes | 当前位置 (只读) |
| PRESENT_SPEED | 58 | 2 bytes | 当前速度 (只读) |

## SDK API 说明

### PortHandler — 串口管理

```python
from SDK import PortHandler

ph = PortHandler("COM7")
ph.openPort()          # 打开串口
ph.setBaudRate(1000000)  # 设置波特率
ph.closePort()         # 关闭串口
```

### PacketHandler — 舵机通信协议

```python
from SDK import PacketHandler

pkt = PacketHandler(0.0)  # SCS 协议版本

# ping 检测电机
model, result, error = pkt.ping(port_handler, motor_id)

# 读写 1 字节
pkt.write1ByteTxRx(ph, motor_id, address, value)

# 读写 2 字节
pkt.write2ByteTxRx(ph, motor_id, address, value)
data, result, error = pkt.read2ByteTxRx(ph, motor_id, address)
```

## SO100Controller API 说明

### 连接管理

```python
ctrl = SO100Controller(port="COM7", baudrate=1_000_000)
ctrl.connect()       # → bool
ctrl.disconnect()    # 自动禁用扭矩
ctrl.is_connected    # → bool
```

### 底层电机操作

```python
# 检测
ctrl.ping(1)            # → bool (电机1是否在线)
ctrl.scan_motors()      # → [1, 2, 3, 4, 5, 6]

# 扭矩
ctrl.enable_torque(1)       # 启用电机1扭矩
ctrl.disable_torque(1)      # 禁用电机1扭矩
ctrl.enable_all_torque()    # 全部启用
ctrl.disable_all_torque()   # 全部禁用

# 位置读写（原始电机 ID）
pos = ctrl.read_position(1)       # → int (0-4095)
ctrl.write_position(1, 2500)      # 写入目标位置
positions = ctrl.read_all_positions()   # → {1: 2047, 2: 2047, ...}
ctrl.write_all_positions({1: 2500, 6: 3000})
```

### 关节控制（按名称）

```python
# 读取状态
state = ctrl.get_state()             # → ArmState
js = ctrl.get_joint("shoulder_pan")  # → JointState

# 设置位置（自动启用扭矩）
ctrl.set_joint("shoulder_pan", 2500)
ctrl.set_joints({"shoulder_pan": 2500, "gripper": 3000})
ctrl.set_all_joints([2047, 2047, 2047, 2047, 2047, 2047])
```

### 夹爪

```python
ctrl.open_gripper(3000)    # 打开（值越大越张开，默认 3000）
ctrl.close_gripper(1000)   # 关闭（值越小越闭合，默认 1000）
```

### 平滑运动

```python
# 线性插值移动（30步，每步20ms → 约0.6秒完成）
ctrl.move_smooth({"shoulder_pan": 2500, "elbow_flex": 1500})
ctrl.move_smooth({"gripper": 3000}, steps=50, dt=0.01)  # 更快更平滑
```

### 预设动作

```python
ctrl.go_home()         # 全部关节回 2047
ctrl.pour_water()      # 预设倒水序列
ctrl.pick_object()     # 抓取
ctrl.place_object()    # 放置
```

### 自由拖动

```python
positions = ctrl.freedrag()  # 释放扭矩, 返回当前位置
# 配合循环轮询：
while True:
    positions = {jc.name: ctrl.read_position(jc.motor_id) for jc in JOINT_REGISTRY}
    print(positions)
    time.sleep(0.1)
```

### 动作序列 / 轨迹

```python
# 动作序列
actions = [
    {"shoulder_pan": 2500, "elbow_flex": 1500, "gripper": 3000},
    {"shoulder_pan": 2047, "elbow_flex": 2047, "gripper": 2047},
]
ctrl.execute_sequence(actions, interval=2.0)

# numpy 轨迹回放 shape=(T, 6)
import numpy as np
traj = np.load("trajectory.npy")  # (100, 6)
ctrl.execute_trajectory(traj, fps=10)

# 从文件加载执行
ctrl.load_and_execute("actions.json")
ctrl.load_and_execute("traj.npy", fps=20)
```

## 命令行使用

### 基本命令

```bash
# 查看关节配置
python main.py --action list_joints

# 扫描电机
python main.py --port COM7 --action scan

# 读取当前状态
python main.py --port COM7 --action status

# 回到零位
python main.py --port COM7 --action home

# 紧急停止
python main.py --port COM7 --action stop
```

### 关节控制

```bash
# 读取单个关节
python main.py --port COM7 --action move --joint shoulder_pan

# 设置单个关节
python main.py --port COM7 --action move --joint shoulder_pan --value 2500

# 设置全部关节
python main.py --port COM7 --action move_all --values 2047 2500 1500 2047 2047 3000
```

### 夹爪

```bash
python main.py --port COM7 --action gripper --value 3000   # 打开
python main.py --port COM7 --action gripper --value 1000   # 关闭
```

### 自由拖动

```bash
python main.py --port COM7 --action freedrag
# 释放扭矩，实时显示各关节位置，Ctrl+C 退出
```

### 任务执行

```bash
python main.py --port COM7 --action pour_water
python main.py --port COM7 --action pick
python main.py --port COM7 --action place
python main.py --port COM7 --action sequence --file actions.json
python main.py --port COM7 --action trajectory --file traj.npy --fps 10
```

## 状态数据格式

`get_state()` 返回的 JSON 结构：

```json
{
  "timestamp": 1709308800.0,
  "joints": {
    "shoulder_pan": {
      "name": "shoulder_pan",
      "motor_id": 1,
      "position": 2047,
      "description": "底部旋转关节"
    },
    "shoulder_lift": { ... },
    "elbow_flex": { ... },
    "wrist_flex": { ... },
    "wrist_roll": { ... },
    "gripper": { ... }
  }
}
```

## 动作序列 JSON 格式

```json
[
  {
    "shoulder_pan": 2047,
    "shoulder_lift": 2500,
    "elbow_flex": 1500,
    "wrist_flex": 1700,
    "wrist_roll": 2047,
    "gripper": 1200
  },
  {
    "shoulder_pan": 2047,
    "shoulder_lift": 2047,
    "elbow_flex": 2047,
    "wrist_flex": 2047,
    "wrist_roll": 2047,
    "gripper": 2047
  }
]
```
