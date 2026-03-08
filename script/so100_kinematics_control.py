#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂运动学控制脚本 (基于 DH 参数)
使用 roboticstoolbox-python 进行运动学计算

依赖安装:
    pip install roboticstoolbox spatialmath scipy
"""

import time
import sys
import os
import numpy as np

# 添加项目路径
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, PROJECT_ROOT)

from SDK import PortHandler, PacketHandler

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    from scipy.optimize import minimize
    IK_AVAILABLE = True
except ImportError as e:
    print(f"[WARN] 运动学库未安装: {e}")
    print("[WARN] 运动学功能将不可用，请运行: pip install roboticstoolbox spatialmath scipy")
    IK_AVAILABLE = False


# ───────── 常量配置 ─────────

# 舵机参数 (STS3215)
SERVO_CENTER = 2047      # 中位位置
SERVO_RANGE = 270         # 角度范围 (度，需根据实际舵机确认)
POSITION_MIN = 0
POSITION_MAX = 4095

# 安全角度限位 (度) - 比舵机物理限位更保守，防止碰撞
# 用户可以根据实际需要调整这些值
SAFE_ANGLE_LIMITS = {
    'rotation': (-135, 135),      # shoulder_pan: 保留35度余量
    'pitch': (-70, 70),           # shoulder_lift: 保留15度余量
    'elbow': (-120, 120),         # elbow_flex: 保留30度余量
    'wrist_pitch': (-100, 100),   # wrist_flex: 保留20度余量
    'wrist_roll': (-150, 150),    # wrist_roll: 保留30度余量
    'gripper': (10, 90),          # jaw: 防止完全闭合/张开
}

# 关节名称映射 (代码 → URDF)
CODE_TO_URDF = {
    'shoulder_pan': 'rotation',
    'shoulder_lift': 'pitch',
    'elbow_flex': 'elbow',
    'wrist_flex': 'wrist_pitch',
    'wrist_roll': 'wrist_roll',
    'gripper': 'jaw',
}

# URDF → 代码名称映射
URDF_TO_CODE_MAP = {
    'rotation': 'shoulder_pan',
    'pitch': 'shoulder_lift',
    'elbow': 'elbow_flex',
    'wrist_pitch': 'wrist_flex',
    'wrist_roll': 'wrist_roll',
    'jaw': 'gripper',
}

# 关节名称映射
JOINT_NAME_MAP = {
    # 代码名称 → URDF 名称
    'shoulder_pan': 'rotation',
    'shoulder_lift': 'pitch',
    'elbow_flex': 'elbow',
    'wrist_flex': 'wrist_pitch',
    'wrist_roll': 'wrist_roll',
    'gripper': 'jaw',
}

# 关节名称映射 (URDF → 代码)
URDF_TO_CODE = {
    'rotation': 'shoulder_pan',
    'pitch': 'shoulder_lift',
    'elbow': 'elbow_flex',
    'wrist_pitch': 'wrist_flex',
    'wrist_roll': 'wrist_roll',
    'jaw': 'gripper',
}

# ───────── SO100 运动学模型 (基于 DH 参数) ─────────
class SO100Kinematics:
    """
    SO100 机械臂运动学模型
    直接从官方 URDF 文件加载
    """

    # 关节名称映射 (URDF → 代码)
    JOINT_NAMES = {
        'rotation': 0,      # J1: 底座旋转
        'pitch': 1,         # J2: 大臂升降
        'elbow': 2,         # J3: 小臂弯曲
        'wrist_pitch': 3,   # J4: 手腕俯仰
        'wrist_roll': 4,    # J5: 手腕旋转
        'jaw': 5,           # J6: 夹爪
    }

    # 关节限制 (度) - 根据实际舵机调整
    JOINT_LIMITS = [
        (-170, 170),   # rotation
        (-85, 85),     # pitch
        (-150, 150),   # elbow
        (-120, 120),   # wrist_pitch
        (-180, 180),   # wrist_roll
        (0, 100),      # jaw (gripper)
    ]

    def __init__(self):
        """从 URDF 提取的 DH 参数创建机器人模型"""
        self.robot = self._create_dh_model()

        # 设置关节限制
        qlim_matrix = np.array([[np.deg2rad(lower), np.deg2rad(upper)]
                                for lower, upper in self.JOINT_LIMITS[:5]])  # 5 DOF
        self.robot.qlim = qlim_matrix.T

    def _create_dh_model(self):
        """创建 DH 参数备用模型"""
        # 从 URDF 提取的 DH 参数 (单位: 米)
        # J1: origin=(0, -0.0452, 0.0165), rpy=(π/2, 0, 0), axis=(0,1,0)
        # J2: origin=(0, 0.1025, 0.0306), axis=(1,0,0)
        # J3: origin=(0, 0.11257, 0.028), axis=(1,0,0)
        # J4: origin=(0, 0.0052, 0.1349), axis=(1,0,0)
        # J5: origin=(0, -0.0601, 0), axis=(0,1,0)

        # 使用标准 DH 参数 (基于 URDF 提取)
        # SO100 是 5 DOF + 1 个夹爪
        dh_params = [
            # a,      α,        d,       θ
            (0.0,    np.pi/2,  0.0165,  np.pi/2),     # J1: rotation
            (0.0,    -np.pi/2, 0.1478,  0.0),         # J2: pitch
            (0.0,    0.0,      0.14057, 0.0),         # J3: elbow
            (0.0,    0.0,      0.1349,  0.0),         # J4: wrist_pitch
            (0.0,    np.pi/2,  0.0,     0.0),         # J5: wrist_roll
        ]

        robot = rtb.DHRobot(
            [rtb.RevoluteDH(a=row[0], alpha=row[1], d=row[2], offset=row[3])
             for row in dh_params],
            name="SO100_DH",
            tool=SE3(0, 0, 0.0601)  # 末端到夹爪的距离
        )

        return robot

    def fk(self, joint_angles: dict) -> SE3:
        """
        正向运动学：计算末端位姿

        Args:
            joint_angles: 关节角度字典 (degrees)

        Returns:
            SE3 末端位姿
        """
        # 获取关节角度 (弧度)
        q = np.zeros(5)  # 5 DOF (不包括夹爪)
        q[0] = np.deg2rad(joint_angles.get('rotation', 0))
        q[1] = np.deg2rad(joint_angles.get('pitch', 0))
        q[2] = np.deg2rad(joint_angles.get('elbow', 0))
        q[3] = np.deg2rad(joint_angles.get('wrist_pitch', 0))
        q[4] = np.deg2rad(joint_angles.get('wrist_roll', 0))

        return self.robot.fkine(q)

    def ik(self, target_pose: SE3, current_angles: dict = None) -> dict:
        """
        逆向运动学：计算关节角度

        Args:
            target_pose: 目标位姿 (SE3)
            current_angles: 当前关节角度 (初值)

        Returns:
            关节角度字典 (degrees)
        """
        # 初值
        if current_angles is None:
            q0 = np.zeros(5)
        else:
            q0 = np.array([
                np.deg2rad(current_angles.get('rotation', 0)),
                np.deg2rad(current_angles.get('pitch', 0)),
                np.deg2rad(current_angles.get('elbow', 0)),
                np.deg2rad(current_angles.get('wrist_pitch', 0)),
                np.deg2rad(current_angles.get('wrist_roll', 0)),
            ])

        # 转换目标为 SE3
        if not isinstance(target_pose, SE3):
            if len(target_pose) == 3:
                target_pose = SE3.Tx(target_pose[0]) * SE3.Ty(target_pose[1]) * SE3.Tz(target_pose[2])
            elif len(target_pose) == 6:
                target_pose = SE3(target_pose[:3], target_pose[3:])

        # 使用 Levenberg-Marquardt IK
        sol = self.robot.ikine_LM(
            target_pose,
            q0=q0,
            ilimit=100,
            tol=1e-6
        )

        if sol.success:
            q = sol.q
            result = {
                'rotation': np.rad2deg(q[0]),
                'pitch': np.rad2deg(q[1]),
                'elbow': np.rad2deg(q[2]),
                'wrist_pitch': np.rad2deg(q[3]),
                'wrist_roll': np.rad2deg(q[4]),
            }
            # 应用安全限位
            return validate_joint_angles(result)
        else:
            print(f"[IK Warning] {sol.reason}")
            return None

    def ik_numerical(self, target_pos: np.ndarray, current_angles: dict) -> dict:
        """
        使用 scipy 优化的数值 IK（位置控制）

        Args:
            target_pos: 目标位置 [x, y, z] (m)
            current_angles: 当前关节角度

        Returns:
            关节角度字典
        """
        # 目标函数：最小化位置误差
        def objective(q):
            q_deg = {
                'rotation': np.rad2deg(q[0]),
                'pitch': np.rad2deg(q[1]),
                'elbow': np.rad2deg(q[2]),
                'wrist_pitch': np.rad2deg(q[3]),
                'wrist_roll': np.rad2deg(q[4]),
            }
            pose = self.fk(q_deg)
            current_pos = pose.t
            return np.sum((current_pos - target_pos)**2)

        # 初值
        q0 = np.array([
            np.deg2rad(current_angles.get('rotation', 0)),
            np.deg2rad(current_angles.get('pitch', 0)),
            np.deg2rad(current_angles.get('elbow', 0)),
            np.deg2rad(current_angles.get('wrist_pitch', 0)),
            np.deg2rad(current_angles.get('wrist_roll', 0)),
        ])

        # 关节限制 - 使用安全限位而非物理限位
        joint_names = ['rotation', 'pitch', 'elbow', 'wrist_pitch', 'wrist_roll']
        bounds = [(np.deg2rad(SAFE_ANGLE_LIMITS[name][0]),
                   np.deg2rad(SAFE_ANGLE_LIMITS[name][1]))
                  for name in joint_names]

        # 优化
        result = minimize(
            objective,
            q0,
            method='SLSQP',
            bounds=bounds,
            options={'ftol': 1e-6, 'maxiter': 100}
        )

        if result.success:
            q = result.x
            result_angles = {
                'rotation': np.rad2deg(q[0]),
                'pitch': np.rad2deg(q[1]),
                'elbow': np.rad2deg(q[2]),
                'wrist_pitch': np.rad2deg(q[3]),
                'wrist_roll': np.rad2deg(q[4]),
            }
            # 二次验证，确保在安全范围内
            return validate_joint_angles(result_angles)
        return None

    def get_joint_limits(self):
        """返回关节限制"""
        return self.JOINT_LIMITS.copy()


# ───────── SO100 硬件控制器 ─────────
class SO100Hardware:
    """SO100 硬件控制（使用原始 SDK）"""

    TORQUE_ENABLE = 40
    GOAL_POSITION = 42
    PRESENT_POSITION = 56

    # 电机 ID 映射
    MOTOR_MAP = {
        'rotation': 1,      # shoulder_pan
        'pitch': 2,         # shoulder_lift
        'elbow': 3,         # elbow_flex
        'wrist_pitch': 4,   # wrist_flex
        'wrist_roll': 5,    # wrist_roll
        'jaw': 6,           # gripper
    }

    def __init__(self, port: str = "COM7", baudrate: int = 1000000):
        self.port = port
        self.baudrate = baudrate
        self.port_handler = None
        self.packet_handler = None

    def connect(self):
        self.port_handler = PortHandler(self.port)
        self.packet_handler = PacketHandler(0.0)
        if not self.port_handler.openPort():
            raise ConnectionError(f"无法打开端口 {self.port}")
        if not self.port_handler.setBaudRate(self.baudrate):
            raise ConnectionError(f"无法设置波特率 {self.baudrate}")
        print(f"[OK] 已连接到 {self.port}")

    def disconnect(self):
        if self.port_handler:
            for motor_id in range(1, 7):
                self.disable_torque(motor_id)
            self.port_handler.closePort()
            print("[OK] 已断开连接")

    def enable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.TORQUE_ENABLE, 1)

    def disable_torque(self, motor_id: int):
        self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.TORQUE_ENABLE, 0)

    def enable_all(self):
        for m in range(1, 7):
            self.enable_torque(m)

    def set_position(self, motor_id: int, position: int):
        position = max(0, min(4095, position))
        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, self.GOAL_POSITION, position)

    def get_position(self, motor_id: int) -> int:
        data, result, _ = self.packet_handler.read2ByteTxRx(
            self.port_handler, motor_id, self.PRESENT_POSITION)
        return data if result == 0 else 0

    def get_all_positions(self) -> dict:
        return {m: self.get_position(m) for m in range(1, 7)}

    def set_joint_angles(self, joint_angles: dict, validate: bool = True):
        """
        按名称设置关节角度

        Args:
            joint_angles: 关节角度字典 (度)
            validate: 是否进行安全验证 (默认 True)
        """
        # 安全验证
        if validate:
            joint_angles = validate_joint_angles(joint_angles)

        # 获取当前角度用于轨迹检查
        current = self.get_joint_angles() if validate else {}

        # 检查单步变化
        if validate and not check_safe_trajectory(current, joint_angles):
            print("[SAFETY] 轨迹检查失败，取消动作")
            return

        # 执行设置
        for name, angle in joint_angles.items():
            if name in self.MOTOR_MAP:
                motor_id = self.MOTOR_MAP[name]
                # 角度已经 validate 过，这里直接转换
                pos = angle_to_position(angle)
                self.set_position(motor_id, pos)

    def get_joint_angles(self) -> dict:
        """获取当前关节角度"""
        positions = self.get_all_positions()
        return {name: position_to_angle(positions[motor_id])
                for name, motor_id in self.MOTOR_MAP.items()}

    def ping(self, motor_id: int) -> bool:
        _, result, _ = self.packet_handler.ping(self.port_handler, motor_id)
        return result == 0


# ───────── 安全限位检查 ─────────
def clamp_angle(joint_name: str, angle_deg: float) -> float:
    """
    将角度限制在安全范围内

    Args:
        joint_name: 关节名称 (rotation, pitch, elbow, wrist_pitch, wrist_roll, gripper)
        angle_deg: 输入角度 (度)

    Returns:
        裁剪后的角度
    """
    # 尝试多个名称变体
    limits_key = None
    if joint_name in SAFE_ANGLE_LIMITS:
        limits_key = joint_name
    elif joint_name in CODE_TO_URDF:
        limits_key = CODE_TO_URDF[joint_name]
    elif joint_name in URDF_TO_CODE_MAP:
        limits_key = joint_name

    if limits_key and limits_key in SAFE_ANGLE_LIMITS:
        min_angle, max_angle = SAFE_ANGLE_LIMITS[limits_key]
        clamped = max(min_angle, min(max_angle, angle_deg))
        if clamped != angle_deg:
            print(f"[SAFETY] {joint_name}: {angle_deg:.1f}° → {clamped:.1f}° (限位: {min_angle}° ~ {max_angle}°)")
        return clamped

    # 未定义限制，使用保守默认值
    return max(-135, min(135, angle_deg))


def validate_joint_angles(joint_angles: dict) -> dict:
    """
    验证并裁剪所有关节角度到安全范围

    Args:
        joint_angles: 关节角度字典

    Returns:
        裁剪后的关节角度字典
    """
    validated = {}
    for name, angle in joint_angles.items():
        validated[name] = clamp_angle(name, angle)
    return validated


def check_safe_trajectory(current_angles: dict, target_angles: dict, max_step: float = 30.0) -> bool:
    """
    检查轨迹是否安全（单步最大变化限制）

    Args:
        current_angles: 当前角度
        target_angles: 目标角度
        max_step: 单步最大变化角度 (度)

    Returns:
        True if safe, False otherwise
    """
    for name, target in target_angles.items():
        current = current_angles.get(name, 0)
        delta = abs(target - current)
        if delta > max_step:
            print(f"[SAFETY] {name} 变化过大: {delta:.1f}° > {max_step}°，拒绝执行")
            return False
    return True


# ───────── 角度转换 ─────────
def angle_to_position(angle_deg: float, center: int = 2047, range_deg: float = 270) -> int:
    """
    将角度转换为舵机位置 (0-4095)

    注意: 调用前应先使用 clamp_angle() 确保角度在安全范围内
    """
    pos = int(center + angle_deg * (4095 / range_deg))
    # 双重保护: 确保位置在硬件范围内
    return max(POSITION_MIN, min(POSITION_MAX, pos))


def position_to_angle(pos: int, center: int = 2047, range_deg: float = 270) -> float:
    """将舵机位置转换为角度"""
    return (pos - center) * (range_deg / 4095)


# ───────── 主程序 ─────────
def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="SO100 运动学控制 (基于 DH 参数)",
        epilog="""
示例:
  # 测试正向运动学
  python so100_kinematics_control.py --test-fk

  # 测试逆向运动学
  python so100_kinematics_control.py --test-ik

  # 交互式笛卡尔控制
  python so100_kinematics_control.py --interactive

  # 可视化机器人模型
  python so100_kinematics_control.py --visualize
        """
    )
    parser.add_argument('--port', type=str, default='COM7', help='串口')
    parser.add_argument('--test-fk', action='store_true', help='测试正向运动学')
    parser.add_argument('--test-ik', action='store_true', help='测试逆向运动学')
    parser.add_argument('--move', type=float, nargs=3, metavar=('X', 'Y', 'Z'),
                        help='移动到笛卡尔坐标 (米) 例如: --move 0.2 0 0.3')
    parser.add_argument('--interactive', action='store_true', help='交互式笛卡尔控制')
    parser.add_argument('--visualize', action='store_true', help='可视化机器人模型')
    parser.add_argument('--show-limits', action='store_true', help='显示安全角度限位')
    parser.add_argument('--test-limits', action='store_true', help='测试安全限位裁剪')

    args = parser.parse_args()

    if not IK_AVAILABLE:
        print("[ERROR] 运动学功能需要安装依赖库:")
        print("        pip install roboticstoolbox spatialmath scipy")
        return 1

    # 创建运动学模型
    kinematics = SO100Kinematics()

    print("="*60)
    print("    SO100 机械臂运动学控制 (官方 URDF)")
    print("="*60)

    # 显示安全限位
    if args.show_limits:
        print("\n[安全角度限位]")
        print("-" * 40)
        for name, (min_angle, max_angle) in SAFE_ANGLE_LIMITS.items():
            range_deg = max_angle - min_angle
            print(f"  {name:15s}: {min_angle:4.0f}° ~ {max_angle:4.0f}°  (范围: {range_deg}°)")
        print("-" * 40)
        print("\n对应舵机位置:")
        for name, (min_angle, max_angle) in SAFE_ANGLE_LIMITS.items():
            min_pos = angle_to_position(min_angle)
            max_pos = angle_to_position(max_angle)
            print(f"  {name:15s}: {min_pos:4d} ~ {max_pos:4d}")
        return

    # 测试安全限位
    if args.test_limits:
        print("\n[安全限位测试]")
        test_cases = [
            {'rotation': 200, 'pitch': 100, 'elbow': 0},   # 超出限位
            {'rotation': -150, 'pitch': -80, 'elbow': 50}, # 正常范围
            {'wrist_pitch': 150, 'wrist_roll': 200},       # 超出限位
        ]
        for i, angles in enumerate(test_cases, 1):
            print(f"\n测试 {i}: {angles}")
            validated = validate_joint_angles(angles)
            print(f"  结果:   {validated}")
        return

    # 可视化机器人
    if args.visualize:
        print("\n[3D 可视化]")
        print(kinematics.robot)
        kinematics.robot.plot(np.zeros(5))
        return

    # 测试正向运动学
    if args.test_fk:
        print("\n[正向运动学测试]")
        test_angles = {
            'rotation': 0,
            'pitch': 30,
            'elbow': -45,
            'wrist_pitch': 20,
            'wrist_roll': 0,
        }
        pose = kinematics.fk(test_angles)
        print(f"输入角度: {test_angles}")
        print(f"末端位置: x={pose.t[0]:.3f}, y={pose.t[1]:.3f}, z={pose.t[2]:.3f}")
        print(f"末端旋转 (RPY): {np.rad2deg(pose.rpy())}")

    # 测试逆向运动学
    if args.test_ik:
        print("\n[逆向运动学测试]")
        target = [0.2, 0.0, 0.3]  # x, y, z (m)
        result = kinematics.ik_numerical(np.array(target), {})
        if result:
            print(f"目标位置: {target} m")
            print(f"解算角度: {result}")
            # 验证
            pose = kinematics.fk(result)
            print(f"验证位置: x={pose.t[0]:.3f}, y={pose.t[1]:.3f}, z={pose.t[2]:.3f}")
        else:
            print("[FAIL] IK 无解")

    # 直接移动到指定位置
    if args.move:
        x, y, z = args.move
        print(f"\n[笛卡尔空间移动]")
        print(f"目标: x={x:.3f}, y={y:.3f}, z={z:.3f} m")

        hardware = SO100Hardware(args.port)
        try:
            hardware.connect()
            hardware.enable_all()

            # 检测电机
            found = [m for m in range(1, 7) if hardware.ping(m)]
            print(f"[OK] 检测到 {len(found)}/6 个电机")

            # 获取当前位置作为初值
            current = hardware.get_joint_angles()

            # 计算 IK
            result = kinematics.ik_numerical(np.array([x, y, z]), current)
            if result:
                print(f"[OK] 解算成功:")
                for name, angle in result.items():
                    print(f"  {name}: {angle:.1f}°")

                # 发送到硬件
                hardware.set_joint_angles(result)
                time.sleep(0.5)
                print("[OK] 移动完成")
            else:
                print("[FAIL] IK 无解 - 目标点可能超出工作空间")

        finally:
            hardware.disconnect()

    # 交互式笛卡尔控制
    if args.interactive:
        print("\n[交互式笛卡尔控制]")
        print("格式: x y z (米)")
        print("示例: 0.2 0 0.3")
        print("输入 'q' 退出")

        hardware = SO100Hardware(args.port)
        try:
            hardware.connect()
            hardware.enable_all()

            # 检测电机
            found = [m for m in range(1, 7) if hardware.ping(m)]
            print(f"[OK] 检测到 {len(found)}/6 个电机")

            if len(found) < 6:
                print(f"[WARN] 部分电机离线，继续运行...")

            while True:
                cmd = input("\ncartesian> ").strip()
                if cmd.lower() == 'q':
                    break
                if not cmd:
                    continue

                try:
                    parts = [float(x) for x in cmd.split()]
                    if len(parts) >= 3:
                        x, y, z = parts[0], parts[1], parts[2]

                        # 获取当前位置作为初值
                        current = hardware.get_joint_angles()

                        # 计算 IK
                        result = kinematics.ik_numerical(np.array([x, y, z]), current)
                        if result:
                            print(f"[OK] 解算成功:")
                            for name, angle in result.items():
                                print(f"  {name}: {angle:.1f}°")

                            # 发送到硬件
                            hardware.set_joint_angles(result)
                            time.sleep(0.5)

                            # 显示新位置
                            actual = hardware.get_joint_angles()
                            print(f"[INFO] 实际角度: {actual}")
                        else:
                            print("[FAIL] IK 无解 - 目标点可能超出工作空间")
                except ValueError as e:
                    print(f"[ERROR] 无效输入: {e}")

        finally:
            hardware.disconnect()


if __name__ == "__main__":
    main()
