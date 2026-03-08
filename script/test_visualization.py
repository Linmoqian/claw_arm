#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""静态测试机械臂可视化"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 添加项目路径
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
except ImportError:
    print("请安装: pip install roboticstoolbox-python spatialmath-python")
    sys.exit(1)

# DH 参数
DH_PARAMS = [
    (0.0,    np.pi/2,  0.0165,  np.pi/2),
    (0.0,    -np.pi/2, 0.1478,  0.0),
    (0.0,    0.0,      0.14057, 0.0),
    (0.0,    0.0,      0.1349,  0.0),
    (0.0,    np.pi/2,  0.0,     0.0),
]

# 创建机器人
robot = rtb.DHRobot(
    [rtb.RevoluteDH(a=row[0], alpha=row[1], d=row[2], offset=row[3])
     for row in DH_PARAMS],
    name="SO100_DH",
    tool=SE3(0, 0, 0.0601)
)

# 测试角度
test_angles_deg = [0, 30, -45, 20, 0]
q = np.deg2rad(test_angles_deg)

print(f"测试角度: {test_angles_deg}")
print(f"弧度: {q}")

# 计算所有关节位置
points = [[0, 0, 0]]  # 基座
for i in range(5):
    q_partial = q.copy()
    q_partial[i+1:] = 0
    pose = robot.fkine(q_partial)
    points.append([pose.t[0], pose.t[1], pose.t[2]])
    print(f"关节 {i+1}: {pose.t}")

points = np.array(points)
print(f"\n点阵形状: {points.shape}")

# 绘图
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# 绘制机械臂
ax.plot(points[:, 0], points[:, 1], points[:, 2],
        'o-', linewidth=4, markersize=10, color='steelblue')

# 绘制关节点
ax.scatter(points[:, 0], points[:, 1], points[:, 2],
           s=100, c='red', zorder=10)

# 设置坐标轴
limit = 0.4
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(0, 0.5)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title(f'SO100 Robot - Angles: {test_angles_deg}')

# 绘制参考平面
xx, yy = np.meshgrid(np.linspace(-limit, limit, 5),
                     np.linspace(-limit, limit, 5))
zz = np.zeros_like(xx)
ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')

# 绘制末端坐标系
end_pose = robot.fkine(q)
origin = end_pose.t
R = end_pose.R
axis_length = 0.08

colors = ['r', 'g', 'b']
labels = ['X', 'Y', 'Z']
for i, (color, label) in enumerate(zip(colors, labels)):
    end = origin + R[:, i] * axis_length
    ax.plot([origin[0], end[0]], [origin[1], end[1]], [origin[2], end[2]],
            color=color, linewidth=3)
    ax.text(end[0], end[1], end[2], label, fontsize=12, color=color)

plt.tight_layout()
plt.savefig('D:/project/claw_arm/test_robot.png', dpi=100)
print("\n图片已保存到 test_robot.png")
print("打开图片查看结果...")
