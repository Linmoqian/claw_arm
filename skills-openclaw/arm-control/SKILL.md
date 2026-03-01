---
name: arm-control
description: SO100 六自由度机械臂控制技能，通过飞特 STS3215 舵机 SDK 直接与硬件底层通信，支持关节控制、夹爪操作、平滑运动、动作序列执行、自由拖动等。
metadata: { "openclaw": { "emoji": "🦾", "requires": { "bins": ["python3"], "env": ["CLAW_ARM_PROJECT"] } } }
---

# SO100 机械臂控制技能 (arm-control)

当用户要求控制机械臂运动（移动关节、抓取、倒水等）、读取机械臂状态、打开或关闭夹爪、释放扭矩进行自由拖动、执行动作序列或轨迹回放时，你可以调用此技能。

## 机器人配置

- 机型：SO100（6 自由度单臂）
- 电机：6 × 飞特 STS3215 舵机（ID 1-6）
- 通信：USB 串口（默认 COM7，波特率 1000000）
- 位置范围：0 ~ 4095（中位 2047）
- 关节：shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper

## 使用方法

你可以通过以下命令调用此技能：

### 查看关节信息（不需要硬件）
```bash
python3 {baseDir}/main.py --action list_joints
```

### 读取机械臂当前状态
```bash
python3 {baseDir}/main.py --port COM7 --action status
```

### 扫描电机是否在线
```bash
python3 {baseDir}/main.py --port COM7 --action scan
```

### 回到初始位置（所有关节归零位 2047）
```bash
python3 {baseDir}/main.py --port COM7 --action home
```

### 移动单个关节
```bash
python3 {baseDir}/main.py --port COM7 --action move --joint shoulder_pan --value 2500
```
可用关节名：shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper

### 同时移动所有关节
```bash
python3 {baseDir}/main.py --port COM7 --action move_all --values 2047 2047 2047 2047 2047 2047
```
参数顺序：shoulder_pan shoulder_lift elbow_flex wrist_flex wrist_roll gripper

### 控制夹爪
```bash
python3 {baseDir}/main.py --port COM7 --action gripper --value 3000
```
值越大越张开（推荐 3000），值越小越闭合（推荐 1000）

### 释放扭矩（自由拖动模式）
```bash
python3 {baseDir}/main.py --port COM7 --action freedrag
```
释放所有电机扭矩，可手动拖动机械臂。Ctrl+C 退出。

### 执行预设倒水动作
```bash
python3 {baseDir}/main.py --port COM7 --action pour_water
```

### 执行抓取/放置动作
```bash
python3 {baseDir}/main.py --port COM7 --action pick
python3 {baseDir}/main.py --port COM7 --action place
```

### 从文件执行动作序列
```bash
python3 {baseDir}/main.py --port COM7 --action sequence --file actions.json
python3 {baseDir}/main.py --port COM7 --action trajectory --file traj.npy --fps 10
```

### 紧急停止
```bash
python3 {baseDir}/main.py --port COM7 --action stop
```

## 环境变量

| 变量 | 说明 | 默认值 |
|------|------|--------|
| CLAW_ARM_PROJECT | claw_arm 项目根目录（SDK 所在位置） | 自动推断 |

## 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| --port | str | COM7 | USB 串口端口号 |
| --baudrate | int | 1000000 | 串口波特率 |
| --action | str | (必需) | 要执行的动作 |
| --joint | str | - | 关节名称（move 动作时使用） |
| --value | int | - | 目标位置 0-4095（move/gripper 时使用） |
| --values | int×6 | - | 6 个关节值（move_all 时使用） |
| --file | str | - | 动作序列/轨迹文件路径 |
| --fps | int | 10 | 轨迹回放帧率 |

## 重要提示

1. 位置值范围为 **0 到 4095**，中位为 **2047**，不是角度或弧度
2. 超出范围的值会被自动裁剪到 [0, 4095]
3. 设置关节会自动启用对应电机的扭矩
4. 断开连接时会自动禁用所有电机扭矩
5. 使用 freedrag 可释放所有扭矩让用户手动拖动机械臂
