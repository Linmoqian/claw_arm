# arm-control 技能参考文档

## 概述

`arm-control` 是 OpenClaw Agent 的核心技能，负责通过 LeRobot 框架控制 SO100 机械臂。

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
LeRobot SO100Follower API
    │
    ▼
USB 串口 → 飞特舵机控制板 → SO100 机械臂
```

## 机器人规格

| 项目 | 参数 |
|------|------|
| 机型 | SO100 |
| 自由度 | 6（5 旋转关节 + 1 夹爪） |
| 电机 | 飞特舵机 × 6 |
| 电源 | 12V DC 适配器 |
| 通信 | USB 串口 (默认 COM7) |
| 控制框架 | LeRobot (lerobot.robots.so_follower) |
| 负载限制 | 约 500g |

## 关节说明

| 电机 ID | 关节名称 | API 键名 | 范围 | 描述 |
|---------|----------|----------|------|------|
| 1 | shoulder_pan | shoulder_pan.pos | -100 ~ 100 | 底部旋转 |
| 2 | shoulder_lift | shoulder_lift.pos | -100 ~ 100 | 大臂升降 |
| 3 | elbow_flex | elbow_flex.pos | -100 ~ 100 | 小臂弯曲 |
| 4 | wrist_flex | wrist_flex.pos | -100 ~ 100 | 手腕俯仰 |
| 5 | wrist_roll | wrist_roll.pos | -100 ~ 100 | 手腕旋转 |
| 6 | gripper | gripper.pos | 0 ~ 100 | 夹爪开合 |

**注意**：位置值不是弧度，而是归一化的位置单位 (-100 到 100)。

## LeRobot API 说明

### 配置类：SO100FollowerConfig

```python
from lerobot.robots.so_follower import SO100Follower, SO100FollowerConfig

config = SO100FollowerConfig(
    port="COM7",                        # 串口端口（必需）
    id="my_so100_arm",                  # 机械臂唯一标识符（必需）
    disable_torque_on_disconnect=True,   # 断开时禁用扭矩
    max_relative_target=None,            # 最大相对移动限制（安全限制）
)
```

### 核心方法

| 方法 | 说明 | 返回 |
|------|------|------|
| `connect(calibrate=False)` | 连接机械臂 | 无 |
| `disconnect()` | 断开连接（自动禁用扭矩） | 无 |
| `get_observation()` | 读取当前关节位置 | `dict` |
| `send_action(action)` | 发送目标位置 | `dict` |

### 观测数据格式

```python
observation = robot.get_observation()
# 返回:
{
    'shoulder_pan.pos': -7.54,    # -100 ~ 100
    'shoulder_lift.pos': 96.69,
    'elbow_flex.pos': -97.40,
    'wrist_flex.pos': -13.28,
    'wrist_roll.pos': 0.07,
    'gripper.pos': 3.38           # 0 ~ 100
}
```

### 动作格式

```python
action = {
    'shoulder_pan.pos': 10.0,
    'shoulder_lift.pos': 10.0,
    'elbow_flex.pos': 10.0,
    'wrist_flex.pos': 10.0,
    'wrist_roll.pos': 10.0,
    'gripper.pos': 50.0
}
robot.send_action(action)
```

## 命令行使用

### 基本命令

```bash
# 查看所有关节信息
python main.py --action list_joints

# 读取当前状态
python main.py --port COM7 --action status

# 回到初始位置
python main.py --port COM7 --action home

# 紧急停止
python main.py --port COM7 --action stop
```

### 单关节控制

```bash
# 读取单个关节
python main.py --port COM7 --action move --joint shoulder_pan

# 设置单个关节
python main.py --port COM7 --action move --joint shoulder_pan --value 30

# 设置肘部角度
python main.py --port COM7 --action move --joint elbow_flex --value -45
```

### 全关节控制

```bash
# 设置全部 6 个关节 (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper)
python main.py --port COM7 --action move_all --values 0 30 -45 -20 0 50
```

### 夹爪控制

```bash
# 打开夹爪
python main.py --port COM7 --action gripper --value 80

# 关闭夹爪
python main.py --port COM7 --action gripper --value 0
```

### 任务执行

```bash
# 倒水
python main.py --port COM7 --action pour_water

# 抓取
python main.py --port COM7 --action pick

# 放置
python main.py --port COM7 --action place

# 轨迹回放
python main.py --port COM7 --action trajectory --file data.parquet --fps 10

# JSON 动作序列
python main.py --port COM7 --action sequence --file actions.json
```

## Python API 使用

```python
from main import create_controller, JOINT_NAMES

# 创建控制器
ctrl = create_controller(port="COM7", robot_id="my_so100_arm")
ctrl.connect()

# 读取状态
state = ctrl.get_state()
for name, js in state.joints.items():
    print(f"  {name}: {js.position:.2f}")

# 设置单个关节
ctrl.set_joint("shoulder_pan", 30.0)
ctrl.set_joint("elbow_flex", -45.0)

# 设置多个关节
ctrl.set_joints({"shoulder_pan": 10, "elbow_flex": -30, "gripper": 50})

# 设置全部关节
ctrl.set_all_joints([0, 30, -45, -20, 0, 50])

# 夹爪
ctrl.open_gripper(80)   # 80% 打开
ctrl.close_gripper()     # 完全关闭

# 回到初始位置
ctrl.go_home()

# 移动到指定位置并等待
ctrl.move_to({"shoulder_pan.pos": 20, "elbow_flex.pos": -30}, wait=2.0)

# 动作序列
actions = [
    {"shoulder_pan.pos": 30, "gripper.pos": 80},
    {"shoulder_pan.pos": -30, "gripper.pos": 10},
    {"shoulder_pan.pos": 0, "gripper.pos": 50},
]
ctrl.execute_sequence(actions, interval=2.0)

# 轨迹回放
import numpy as np
traj = np.random.uniform(-30, 30, (100, 6))
ctrl.execute_trajectory(traj, fps=10)

# 高级任务
ctrl.pour_water()
ctrl.pick_object(
    pre_grasp={"shoulder_lift.pos": 30, "elbow_flex.pos": -45, "gripper.pos": 80},
    grasp={"shoulder_lift.pos": 10, "elbow_flex.pos": -80, "gripper.pos": 80},
)
ctrl.place_object(place_pos={"shoulder_pan.pos": 40, "gripper.pos": 80})

# 从文件执行
ctrl.load_and_execute_trajectory("data.parquet", fps=10)

# 断开连接
ctrl.disconnect()
```

## 首次使用项

### 1. 设置电机 ID

每次只连接一个电机，运行：
```bash
lerobot-setup-motors --robot.type=so100_follower --robot.port=COM7 --robot.id=my_so100_arm
```

### 2. 校准

将机械臂移动到中间位置，运行：
```bash
lerobot-calibrate --robot.type=so100_follower --robot.port=COM7 --robot.id=my_so100_arm
```

校准数据保存在 `~/.cache/huggingface/lerobot/calibration/robots/so_follower/my_so100_arm.json`

### 3. 查找串口

```bash
# Python 方式
python -c "import serial.tools.list_ports; [print(p.device, p.description) for p in serial.tools.list_ports.comports()]"

# LeRobot 方式
lerobot-find-port
```

## 安全注意事项

1. **安全距离**：确保机械臂周围至少 1 米半径无障碍
2. **紧急停止**：`Ctrl+C` 或 `--action stop` 立即禁用所有扭矩
3. **缓慢测试**：使用较小的值（如 ±10）进行首次测试
4. **移动限制**：设置 `max_relative_target=10` 限制单次最大移动幅度
5. **电源安全**：使用 12V DC 适配器，勿超载
6. **夹爪安全**：注意不要夹到手指
7. **负载限制**：不超过 500g
