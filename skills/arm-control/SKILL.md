```skill
# arm-control

## 描述

SO100 机械臂控制技能，直接通过本地 SDK（飞特 STS3215 舵机协议）与硬件底层通信。

机器人配置：
- 机型：SO100（6 自由度单臂）
- 电机：6 × 飞特 STS3215 舵机（ID 1-6）
- 通信：USB 串口（默认 COM7，波特率 1000000）
- 控制方式：直接读写舵机寄存器（本地 SDK）

关节列表：

| 电机 ID | 关节名称 | 描述 | 位置范围 | 零位 |
|---------|----------|------|----------|------|
| 1 | shoulder_pan | 底部旋转 | 0 ~ 4095 | 2047 |
| 2 | shoulder_lift | 大臂升降 | 0 ~ 4095 | 2047 |
| 3 | elbow_flex | 小臂弯曲 | 0 ~ 4095 | 2047 |
| 4 | wrist_flex | 手腕俯仰 | 0 ~ 4095 | 2047 |
| 5 | wrist_roll | 手腕旋转 | 0 ~ 4095 | 2047 |
| 6 | gripper | 夹爪开合 | 0 ~ 4095 | 2047 |

核心寄存器地址：
| 寄存器 | 地址 | 说明 |
|--------|------|------|
| TORQUE_ENABLE | 40 | 扭矩开关 (0=释放, 1=锁定) |
| GOAL_POSITION | 42 | 目标位置 (0-4095) |
| PRESENT_POSITION | 56 | 当前位置 (0-4095) |

## 使用场景

- 当用户要求控制机械臂运动（移动关节、抓取、倒水等）时
- 当需要读取机械臂各关节当前位置时
- 当需要打开或关闭夹爪时
- 当需要回到初始位置（零位 2047）时
- 当需要释放扭矩实现自由拖动时
- 当需要执行动作序列或轨迹回放时
- 当需要扫描电机是否在线时

## 使用指令

```
读取机械臂当前状态
控制机械臂回到初始位置
打开夹爪
关闭夹爪
让 shoulder_pan 关节转到 2500
把所有关节设为 2047
释放机械臂让我手动拖动
扫描电机是否都在线
执行倒水动作
执行抓取动作
查看所有关节信息
```

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| port | str | "COM7" | USB 串口端口号 |
| baudrate | int | 1000000 | 串口波特率 |

## 重要提示

1. 位置值范围为 **0 到 4095**，中位为 **2047**，不是角度或弧度
2. 超出范围的值会被自动裁剪到 [0, 4095]
3. `set_joint` 会自动启用对应电机的扭矩
4. 断开连接时会自动禁用所有电机扭矩
5. 使用 `freedrag()` 可释放所有扭矩让用户手动拖动机械臂

## Python API 快速参考

```python
from main import create_controller, JOINT_NAMES

ctrl = create_controller(port="COM7")
ctrl.connect()

# 扫描电机
motors = ctrl.scan_motors()  # → [1, 2, 3, 4, 5, 6]

# 读取状态
state = ctrl.get_state()     # → ArmState 对象
js = ctrl.get_joint("gripper")  # → JointState

# 底层读写
pos = ctrl.read_position(1)        # 读电机1位置
ctrl.write_position(1, 2500)       # 写电机1位置
ctrl.enable_torque(1)              # 启用电机1扭矩
ctrl.disable_torque(1)             # 禁用电机1扭矩

# 关节控制（按名称）
ctrl.set_joint("shoulder_pan", 2500)
ctrl.set_joints({"shoulder_pan": 2500, "gripper": 3000})
ctrl.set_all_joints([2047, 2047, 2047, 2047, 2047, 2047])

# 夹爪
ctrl.open_gripper(3000)
ctrl.close_gripper(1000)

# 平滑移动
ctrl.move_smooth({"shoulder_pan": 2500, "elbow_flex": 1500})

# 预设
ctrl.go_home()       # 全部回 2047
ctrl.pour_water()    # 倒水
ctrl.pick_object()   # 抓取
ctrl.place_object()  # 放置

# 自由拖动
ctrl.freedrag()      # 释放扭矩 + 返回当前位置

# 断开
ctrl.disconnect()
```
```
