---
name: urdf-kinematics
description: SO100 机械臂 URDF 解析与运动学模型。当用户需要进行机械臂运动学计算（正/逆运动学、雅可比矩阵、工作空间分析）、读取 URDF 文件参数、计算末端位姿、规划轨迹时使用此技能。用户说类似"计算正运动学"、"求逆运动学解"、"读取 URDF 参数"、"计算末端位置"、"检查工作空间"时应触发此技能。
metadata:
  {
    "openclaw": {
      "requires": { "bins": ["python3", "pip3"] },
      "user-invocable": true
    }
  }
---

# SO100 机械臂 URDF 运动学模型

此技能提供 SO100 六自由度机械臂的完整运动学支持，包括 URDF 文件解析、正/逆运动学计算、雅可比矩阵分析等。

## 核心功能

| 功能 | 说明 | API |
|------|------|-----|
| URDF 解析 | 读取 URDF 文件，提取关节/链接参数 | `URDFParser` |
| 正运动学 | 关节角度 → 末端位姿 | `forward_kinematics()` |
| 逆运动学 | 末端位姿 → 关节角度 | `inverse_kinematics()` |
| 雅可比矩阵 | 速度映射关系 | `jacobian()` |
| 工作空间分析 | 可达空间检查 | `in_workspace()` |
| 轨迹规划 | 关节/笛卡尔轨迹 | `joint_trajectory()`, `cartesian_trajectory()` |
| 舵机转换 | 角度 ↔ 舵机位置 | `ServoConverter` |

## SO100 机械臂参数

### 关节配置（6-DOF）

| ID | 关节名称 | 类型 | 限位 (rad) | 限位 (deg) |
|----|----------|------|------------|------------|
| 1 | shoulder_pan | 旋转 | [-2.0, 2.0] | [-114.6°, 114.6°] |
| 2 | shoulder_lift | 旋转 | [0.0, 3.5] | [0°, 200.5°] |
| 3 | elbow_flex | 旋转 | [-π, 0] | [-180°, 0°] |
| 4 | wrist_flex | 旋转 | [-2.5, 1.2] | [-143.2°, 68.8°] |
| 5 | wrist_roll | 旋转 | [-π, π] | [-180°, 180°] |
| 6 | gripper | 旋转 | [-0.2, 2.0] | [-11.5°, 114.6°] |

### DH 参数表

| 关节 | d (m) | a (m) | α (rad) | θ offset |
|------|-------|-------|---------|----------|
| 1 | 0.0452 | 0 | π/2 | 0 |
| 2 | 0 | 0.1025 | 0 | -π/2 |
| 3 | 0 | 0.11257 | 0 | π/2 |
| 4 | 0.1401 | 0 | -π/2 | 0 |
| 5 | 0 | 0 | π/2 | π/2 |
| 6 | 0.0845 | 0 | 0 | 0 |

### 连杆长度

| 链接 | 长度 (m) |
|------|----------|
| base → shoulder | 0.0452 |
| shoulder → upper_arm | 0.1025 |
| upper_arm → lower_arm | 0.11257 |
| lower_arm → wrist | 0.1349 |
| wrist → gripper | 0.0601 |
| gripper → jaw | 0.0244 |

**工作空间半径**: ~0.47 m

## 使用方法

### 1. 初始化模型

```python
import sys
sys.path.insert(0, 'D:/project/claw_arm')

from skills.urdf-kinematics.scripts.urdf_kinematics import SO100Kinematics

# 创建运动学模型
model = SO100Kinematics()
```

### 2. 正运动学计算

```python
import numpy as np

# 输入关节角度 (弧度)
q = np.array([0.0, -np.pi/4, np.pi/4, -np.pi/3, 0.0, -np.pi/4])

# 计算末端变换矩阵
T = model.forward_kinematics(q)

# 获取末端位姿详情
pose = model.get_end_effector_pose(q)
print(f"位置 (m): {pose['position']}")  # [x, y, z]
print(f"姿态 RPY (rad): {pose['orientation']['rpy']}")
print(f"四元数: {pose['orientation']['quaternion']}")
```

### 3. 逆运动学求解

```python
# 方法1: 使用变换矩阵
T_target = T  # 使用正运动学的结果作为目标
q_solution = model.inverse_kinematics(T_target, method='optimize')

# 方法2: 使用位姿向量 [x, y, z, roll, pitch, yaw]
target_pose = np.array([0.2, 0.1, 0.3, 0, np.pi/4, 0])
q_solution = model.inverse_kinematics(target_pose, method='optimize')

# 方法3: 使用雅可比迭代
q_solution = model.inverse_kinematics(T_target, method='jacobian')

print(f"关节角度 (deg): {np.rad2deg(q_solution)}")
```

### 4. 雅可比矩阵

```python
# 计算几何雅可比矩阵 (6x6)
J = model.jacobian(q)
print(f"雅可比矩阵:\n{J}")
print(f"条件数: {np.linalg.cond(J):.2f}")  # 评估奇异性
```

### 5. 关节限位检查

```python
# 获取限位
limits = model.get_joint_limits()

# 检查是否在限位内
is_valid = model.check_joint_limits(q)
```

### 6. 工作空间检查

```python
# 检查点是否可达
position = np.array([0.2, 0.1, 0.3])
reachable = model.in_workspace(position)
```

### 7. 轨迹规划

```python
# 关节空间线性轨迹
q_start = np.array([0, -np.pi/6, np.pi/6, -np.pi/4, 0, 0])
q_end = np.array([np.pi/6, -np.pi/3, np.pi/3, -np.pi/2, np.pi/6, 0])
traj = model.joint_trajectory(q_start, q_end, num_points=50)

# S曲线平滑轨迹
traj_smooth = model.scurve_trajectory(q_start, q_end, num_points=50)

# 笛卡尔空间轨迹
start_pose = np.array([0.2, 0, 0.3, 0, 0, 0])
end_pose = np.array([0.3, 0.1, 0.2, 0, np.pi/4, 0])
cart_traj = model.cartesian_trajectory(start_pose, end_pose, num_points=50)
```

### 8. 舵机位置转换

```python
from skills.urdf-kinematics.scripts.urdf_kinematics import ServoConverter

# 角度 → 舵机位置 (0-4095)
pos = ServoConverter.angle_to_position(45.0)  # 45度

# 舵机位置 → 角度
angle = ServoConverter.position_to_angle(2500)

# 弧度 ↔ 舵机位置
pos = ServoConverter.rad_to_position(np.pi/4)
rad = ServoConverter.position_to_rad(2500)
```

## URDF 文件位置

SO100 的 URDF 文件位于: `SO100描述文件/so100.urdf`

## 完整示例

```python
import numpy as np
import sys
sys.path.insert(0, 'D:/project/claw_arm')

from skills.urdf-kinematics.scripts.urdf_kinematics import SO100Kinematics, ServoConverter

# 初始化
model = SO100Kinematics()

# 定义目标末端位置 (米)
target_position = [0.15, 0.05, 0.25]

# 求解逆运动学
target_pose = np.array([*target_position, 0, 0, 0])  # [x,y,z,r,p,y]
q_solution = model.inverse_kinematics(target_pose, method='optimize')

# 验证解
T_result = model.forward_kinematics(q_solution)
print(f"目标位置: {target_position}")
print(f"实际位置: {T_result[:3, 3]}")
print(f"位置误差: {np.linalg.norm(np.array(target_position) - T_result[:3, 3])*1000:.2f} mm")

# 转换为舵机位置
servo_positions = [ServoConverter.rad_to_position(q) for q in q_solution]
print(f"舵机位置: {servo_positions}")
```

## 注意事项

1. **单位**: 关节角度使用弧度制，位置使用米
2. **逆运动学多解**: 6-DOF 机械臂可能存在多组解，算法返回其中一组
3. **奇异位形**: 当雅可比条件数较大时，机械臂接近奇异位形
4. **关节限位**: 逆运动学求解时会考虑关节限位约束
5. **舵机范围**: STS3215 舵机位置范围 0-4095，中位 2047，转动范围 270°
