#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂控制脚本（基于 lerobot）
支持关节空间控制和笛卡尔空间控制（简化的正向/逆向运动学）
"""

import time
import json
import os
import sys
import numpy as np
from typing import Optional, Tuple

# 添加 lerobot 路径
sys.path.insert(0, 'D:/project/lerobot/src')

from lerobot.robots import make_robot_from_config
from lerobot.robots.config import RobotConfig


# ───────── 简化的运动学模型 ─────────
class SimpleKinematics:
    """
    SO100 简化运动学模型
    使用 DH 参数法和数值优化实现 IK
    """

    # SO100 的 DH 参数（基于标准 6-DOF 机械臂）
    # 单位：长度(m), 角度(rad)
    DH_PARAMS = {
        # a, alpha, d, theta_offset
        1: (0.0,      np.pi/2,   0.098,  0),      # shoulder_pan
        2: (0.0,      0.0,       0.0,    -np.pi/2), # shoulder_lift
        3: (0.0,      0.0,       0.220,  0),      # elbow_flex
        4: (0.0,      np.pi/2,   0.0,    0),      # wrist_flex
        5: (0.0,      -np.pi/2,  0.068,  0),      # wrist_roll
        6: (0.0,      0.0,       0.035,  0),      # gripper (end effector)
    }

    # 关节限制（角度）
    JOINT_LIMITS = {
        'shoulder_pan':  (-170, 170),
        'shoulder_lift': (-85, 85),
        'elbow_flex':   (-150, 150),
        'wrist_flex':   (-120, 120),
        'wrist_roll':   (-180, 180),
        'gripper':      (0, 100),
    }

    def __init__(self):
        self.joint_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                           'wrist_flex', 'wrist_roll']

    @staticmethod
    def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
        """DH 变换矩阵"""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct,      -st*ca,   st*sa,   a*ct],
            [st,      ct*ca,    -ct*sa,  a*st],
            [0,       sa,       ca,      d],
            [0,       0,        0,       1]
        ])

    def forward_kinematics(self, joint_angles: dict) -> np.ndarray:
        """
        正向运动学：计算末端执行器位姿

        Args:
            joint_angles: 关节角度字典 (degrees)
                {'shoulder_pan': 0, 'shoulder_lift': 30, ...}

        Returns:
            4x4 齐次变换矩阵
        """
        # 获取关节角度（转换为弧度）
        theta = []
        for i, name in enumerate(self.joint_names, 1):
            angle = np.deg2rad(joint_angles.get(name, 0))
            theta.append(angle)

        # 计算 DH 变换
        T = np.eye(4)
        for i, name in enumerate(self.joint_names, 1):
            a, alpha, d, offset = self.DH_PARAMS[i]
            T = T @ self.dh_transform(a, alpha, d, theta[i-1] + offset)

        return T

    def get_ee_position(self, joint_angles: dict) -> Tuple[float, float, float]:
        """获取末端执行器位置 (x, y, z)"""
        T = self.forward_kinematics(joint_angles)
        return tuple(T[:3, 3])

    def inverse_kinematics(self,
                          target_pos: np.ndarray,
                          current_angles: dict,
                          max_iter: int = 100,
                          tol: float = 1e-3) -> dict:
        """
        逆向运动学：使用雅可比迭代法计算关节角度

        Args:
            target_pos: 目标位置 [x, y, z] (m)
            current_angles: 当前关节角度 (degrees)
            max_iter: 最大迭代次数
            tol: 位置容差 (m)

        Returns:
            关节角度字典
        """
        # 转换为 numpy 数组
        q = np.array([current_angles.get(name, 0) for name in self.joint_names])

        target = target_pos[:3] if len(target_pos) > 3 else target_pos

        for _ in range(max_iter):
            # 计算当前位置
            angles = dict(zip(self.joint_names, q))
            T = self.forward_kinematics(angles)
            current = T[:3, 3]

            # 计算误差
            error = target - current
            if np.linalg.norm(error) < tol:
                break

            # 数值雅可比
            J = self._compute_jacobian(q)

            # 阻尼最小二乘法
            damping = 0.01
            dq = J.T @ np.linalg.solve(J @ J.T + damping * np.eye(3), error)

            # 更新关节角度
            q = q + np.rad2deg(dq) * 0.5  # 0.5 是步长因子

            # 限制关节范围
            for i, name in enumerate(self.joint_names):
                limits = self.JOINT_LIMITS.get(name, (-180, 180))
                q[i] = np.clip(q[i], limits[0], limits[1])

        return dict(zip(self.joint_names, q))

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """数值计算雅可比矩阵"""
        epsilon = 1e-6
        J = np.zeros((3, len(q)))

        angles = dict(zip(self.joint_names, q))
        T0 = self.forward_kinematics(angles)
        p0 = T0[:3, 3]

        for i in range(len(q)):
            q_eps = q.copy()
            q_eps[i] += epsilon
            angles_eps = dict(zip(self.joint_names, q_eps))
            T_eps = self.forward_kinematics(angles_eps)
            p_eps = T_eps[:3, 3]

            J[:, i] = (p_eps - p0) / epsilon

        return J


# ───────── SO100 控制器 ─────────
class SO100Controller:
    """SO100 机械臂控制器（基于 lerobot）"""

    def __init__(self, port: str = "COM7"):
        self.port = port

        # 创建配置
        config_dict = {
            "type": "so100_follower",
            "id": "so100",
            "port": port,
            "disable_torque_on_disconnect": True,
            "max_relative_target": 10.0,  # 安全限制
        }

        # 使用 draccus 加载配置
        import draccus
        self.config = draccus.decode(config_dict, RobotConfig)

        # 创建机器人
        self.robot = make_robot_from_config(self.config)

        # 运动学模型
        self.kinematics = SimpleKinematics()

        # 连接状态
        self._connected = False

    def connect(self, calibrate: bool = False):
        """连接到机械臂"""
        if not self._connected:
            self.robot.connect(calibrate=calibrate)
            self._connected = True
        print(f"[*] Connected to SO100 on {self.port}")

    def disconnect(self):
        """断开连接"""
        if self._connected:
            self.robot.disconnect()
            self._connected = False
        print("[*] Disconnected")

    def get_observation(self) -> dict:
        """获取当前观测"""
        obs = self.robot.get_observation()
        # lerobot 返回归一化的值 (-100 到 100)，或角度（如果 use_degrees=True）
        return obs

    def get_joint_angles(self) -> dict:
        """获取当前关节角度"""
        obs = self.get_observation()
        angles = {}
        for key, val in obs.items():
            if key.endswith('.pos'):
                name = key.replace('.pos', '')
                angles[name] = val
        return angles

    def send_action(self, action: dict):
        """发送动作"""
        self.robot.send_action(action)

    def move_joints(self, joint_commands: dict, wait: bool = True):
        """
        关节空间运动

        Args:
            joint_commands: 关节命令
                归一化模式: {'shoulder_pan': 10.0, ...}  # -100 到 100
                或使用 send_action 的格式
            wait: 是否等待运动完成
        """
        action = {}
        for name, value in joint_commands.items():
            action[f"{name}.pos"] = value

        self.send_action(action)

        if wait:
            time.sleep(0.1)  # 等待命令发送

    def move_to_position(self,
                        target_pos: Tuple[float, float, float],
                        current_angles: Optional[dict] = None):
        """
        笛卡尔空间运动（使用 IK）

        Args:
            target_pos: 目标位置 (x, y, z) 单位：米
            current_angles: 当前关节角度（如果为 None 则读取）
        """
        if current_angles is None:
            current_angles = self.get_joint_angles()

        # 计算 IK
        target = np.array(target_pos)
        joint_angles = self.kinematics.inverse_kinematics(
            target, current_angles
        )

        # 发送命令（需要转换角度到归一化值）
        # 这里需要根据校准数据进行转换
        # 暂时直接使用角度值
        action = {}
        for name, angle in joint_angles.items():
            action[f"{name}.pos"] = angle

        self.send_action(action)

    def get_ee_position(self) -> Tuple[float, float, float]:
        """获取末端执行器位置"""
        angles = self.get_joint_angles()
        return self.kinematics.get_ee_position(angles)

    def home(self):
        """回到零位"""
        zero_pos = {name: 0.0 for name in
                   ['shoulder_pan', 'shoulder_lift', 'elbow_flex',
                    'wrist_flex', 'wrist_roll', 'gripper']}
        self.move_joints(zero_pos)


# ───────── 交互式控制台 ─────────
def interactive_control(controller: SO100Controller):
    """交互式控制台"""

    print("\n" + "="*60)
    print("    SO100 机械臂控制台 (基于 lerobot)")
    print("="*60)

    while True:
        print("\n[命令]")
        print("  j       - 关节控制模式")
        print("  c       - 笛卡尔控制模式 (IK)")
        print("  p       - 显示当前位置")
        print("  h       - 回零位")
        print("  q       - 退出")

        cmd = input("\n> ").strip().lower()

        if cmd == 'q':
            break
        elif cmd == 'h':
            print("[*] 回零位...")
            controller.home()
            time.sleep(1)
        elif cmd == 'p':
            angles = controller.get_joint_angles()
            pos = controller.get_ee_position()
            print("\n[关节位置]")
            for name, val in angles.items():
                print(f"  {name}: {val:>7.2f}")
            print(f"\n[末端位置] x: {pos[0]:.3f}, y: {pos[1]:.3f}, z: {pos[2]:.3f}")
        elif cmd == 'j':
            joint_control(controller)
        elif cmd == 'c':
            cartesian_control(controller)
        else:
            print("[!] 未知命令")


def joint_control(controller: SO100Controller):
    """关节控制模式"""
    print("\n[关节控制模式] 输入格式: <关节名> <值>")
    print("关节名: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper")
    print("值范围: -100 到 100 (归一化) 或角度")
    print("输入 'back' 返回主菜单")

    while True:
        cmd = input("\njoint> ").strip()
        if cmd == 'back':
            break

        try:
            parts = cmd.split()
            if len(parts) >= 2:
                name = parts[0]
                value = float(parts[1])
                controller.move_joints({name: value})
                print(f"[*] 移动 {name} 到 {value}")
        except ValueError as e:
            print(f"[!] 错误: {e}")


def cartesian_control(controller: SO100Controller):
    """笛卡尔控制模式"""
    print("\n[笛卡尔控制模式] 输入格式: <x> <y> <z>")
    print("坐标单位: 米，原点在底座中心")
    print("输入 'back' 返回主菜单")

    current_pos = controller.get_ee_position()
    print(f"[*] 当前位置: x={current_pos[0]:.3f}, y={current_pos[1]:.3f}, z={current_pos[2]:.3f}")

    while True:
        cmd = input("\ncartesian> ").strip()
        if cmd == 'back':
            break

        try:
            parts = cmd.split()
            if len(parts) >= 3:
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                print(f"[*] 移动到 ({x}, {y}, {z})")
                controller.move_to_position((x, y, z))

                # 显示新位置
                new_pos = controller.get_ee_position()
                print(f"[*] 实际位置: x={new_pos[0]:.3f}, y={new_pos[1]:.3f}, z={new_pos[2]:.3f}")
        except ValueError as e:
            print(f"[!] 错误: {e}")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="SO100 机械臂控制")
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--calibrate', action='store_true', help='校准机械臂')
    parser.add_argument('--test', action='store_true', help='测试模式')

    args = parser.parse_args()

    controller = SO100Controller(port=args.port)

    try:
        controller.connect(calibrate=args.calibrate)

        if args.test:
            # 测试模式：读取并显示位置
            print("\n[*] 测试模式")
            for i in range(5):
                angles = controller.get_joint_angles()
                pos = controller.get_ee_position()
                print(f"\n[帧 {i+1}]")
                for name, val in angles.items():
                    print(f"  {name}: {val:>7.2f}")
                print(f"  末端: x={pos[0]:.3f}, y={pos[1]:.3f}, z={pos[2]:.3f}")
                time.sleep(1)
        else:
            interactive_control(controller)

    except KeyboardInterrupt:
        print("\n[!] 中断")
    finally:
        controller.disconnect()


if __name__ == "__main__":
    main()
