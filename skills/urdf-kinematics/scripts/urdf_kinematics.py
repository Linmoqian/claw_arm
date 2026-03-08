#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂 URDF 运动学模型
基于 URDF 文件构建的完整运动学模型
包含：URDF解析、正运动学、逆运动学、雅可比矩阵、轨迹规划

用法:
    python -m skills.urdf-kinematics.scripts.urdf_kinematics --test
    python -m skills.urdf-kinematics.scripts.urdf_kinematics --parse-urdf
    python -m skills.urdf-kinematics.scripts.urdf_kinematics --fk 0 -0.785 0.785 -1.047 0 -0.785
"""

import numpy as np
from scipy.optimize import fsolve, minimize
from dataclasses import dataclass, field
from typing import Tuple, List, Optional, Dict, Any
from xml.etree import ElementTree as ET
import json
import os


# ============================================================================
# 数据类定义
# ============================================================================

@dataclass
class URDFLink:
    """URDF 链接参数"""
    name: str
    mass: float = 0.0
    center_of_mass: List[float] = field(default_factory=lambda: [0, 0, 0])
    inertia: List[float] = field(default_factory=lambda: [0, 0, 0, 0, 0, 0])


@dataclass
class URDFJoint:
    """URDF 关节参数"""
    name: str
    joint_type: str = "fixed"
    parent: str = ""
    child: str = ""
    origin_xyz: List[float] = field(default_factory=lambda: [0, 0, 0])
    origin_rpy: List[float] = field(default_factory=lambda: [0, 0, 0])
    axis: List[float] = field(default_factory=lambda: [1, 0, 0])
    limit_lower: float = 0.0
    limit_upper: float = 0.0
    limit_effort: float = 0.0
    limit_velocity: float = 0.0


@dataclass
class DHParameter:
    """标准 DH 参数"""
    d: float       # 连杆偏距
    a: float       # 连杆长度
    alpha: float   # 连杆扭角
    theta_offset: float  # θ 偏移


# ============================================================================
# URDF 解析器
# ============================================================================

class URDFParser:
    """URDF 文件解析器"""

    def __init__(self, urdf_path: str):
        """
        初始化 URDF 解析器

        Args:
            urdf_path: URDF 文件路径
        """
        self.urdf_path = urdf_path
        self.links: Dict[str, URDFLink] = {}
        self.joints: Dict[str, URDFJoint] = {}
        self.robot_name: str = ""

    def parse(self) -> bool:
        """
        解析 URDF 文件

        Returns:
            bool: 解析是否成功
        """
        if not os.path.exists(self.urdf_path):
            print(f"错误: URDF 文件不存在: {self.urdf_path}")
            return False

        try:
            tree = ET.parse(self.urdf_path)
            root = tree.getroot()

            self.robot_name = root.get('name', 'unknown')

            # 解析 links
            for link_elem in root.findall('link'):
                link = self._parse_link(link_elem)
                self.links[link.name] = link

            # 解析 joints
            for joint_elem in root.findall('joint'):
                joint = self._parse_joint(joint_elem)
                self.joints[joint.name] = joint

            return True

        except Exception as e:
            print(f"解析 URDF 失败: {e}")
            return False

    def _parse_link(self, elem) -> URDFLink:
        """解析 link 元素"""
        link = URDFLink(name=elem.get('name'))

        # 惯性参数
        inertial = elem.find('inertial')
        if inertial is not None:
            mass_elem = inertial.find('mass')
            if mass_elem is not None:
                link.mass = float(mass_elem.get('value', 0))

            origin_elem = inertial.find('origin')
            if origin_elem is not None:
                xyz = origin_elem.get('xyz', '0 0 0')
                link.center_of_mass = [float(x) for x in xyz.split()]

            inertia_elem = inertial.find('inertia')
            if inertia_elem is not None:
                ixx = float(inertia_elem.get('ixx', 0))
                ixy = float(inertia_elem.get('ixy', 0))
                ixz = float(inertia_elem.get('ixz', 0))
                iyy = float(inertia_elem.get('iyy', 0))
                iyz = float(inertia_elem.get('iyz', 0))
                izz = float(inertia_elem.get('izz', 0))
                link.inertia = [ixx, iyy, izz, ixy, ixz, iyz]

        return link

    def _parse_joint(self, elem) -> URDFJoint:
        """解析 joint 元素"""
        joint = URDFJoint(
            name=elem.get('name'),
            joint_type=elem.get('type', 'fixed')
        )

        # parent 和 child
        parent_elem = elem.find('parent')
        if parent_elem is not None:
            joint.parent = parent_elem.get('link', '')

        child_elem = elem.find('child')
        if child_elem is not None:
            joint.child = child_elem.get('link', '')

        # origin
        origin_elem = elem.find('origin')
        if origin_elem is not None:
            xyz = origin_elem.get('xyz', '0 0 0')
            joint.origin_xyz = [float(x) for x in xyz.split()]
            rpy = origin_elem.get('rpy', '0 0 0')
            joint.origin_rpy = [float(x) for x in rpy.split()]

        # axis
        axis_elem = elem.find('axis')
        if axis_elem is not None:
            xyz = axis_elem.get('xyz', '1 0 0')
            joint.axis = [float(x) for x in xyz.split()]

        # limit
        limit_elem = elem.find('limit')
        if limit_elem is not None:
            joint.limit_lower = float(limit_elem.get('lower', 0))
            joint.limit_upper = float(limit_elem.get('upper', 0))
            joint.limit_effort = float(limit_elem.get('effort', 0))
            joint.limit_velocity = float(limit_elem.get('velocity', 0))

        return joint

    def get_kinematic_chain(self) -> List[URDFJoint]:
        """
        获取运动学链（从 base 到末端）

        Returns:
            List[URDFJoint]: 按顺序排列的关节列表
        """
        if not self.joints:
            return []

        # 构建父子关系图
        child_to_joint = {j.child: j.name for j in self.joints.values()}

        # 找到根 link (没有 parent 的 link)
        child_links = {j.child for j in self.joints.values()}
        root_link = None
        for link_name in self.links:
            if link_name not in child_links:
                root_link = link_name
                break

        if not root_link:
            return []

        # 遍历运动学链
        chain = []
        current_link = root_link

        while current_link in child_to_joint:
            joint_name = child_to_joint[current_link]
            joint = self.joints[joint_name]
            chain.append(joint)
            current_link = joint.child

        return chain

    def to_dict(self) -> Dict[str, Any]:
        """转换为字典格式"""
        return {
            'robot_name': self.robot_name,
            'links': {
                name: {
                    'mass': link.mass,
                    'center_of_mass': link.center_of_mass,
                    'inertia': link.inertia
                }
                for name, link in self.links.items()
            },
            'joints': {
                name: {
                    'type': joint.joint_type,
                    'parent': joint.parent,
                    'child': joint.child,
                    'origin_xyz': joint.origin_xyz,
                    'origin_rpy': joint.origin_rpy,
                    'axis': joint.axis,
                    'limit': [joint.limit_lower, joint.limit_upper]
                }
                for name, joint in self.joints.items()
            }
        }


# ============================================================================
# 齐次变换矩阵
# ============================================================================

def dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    标准 DH 变换矩阵

        | cos(θ)  -sin(θ)  0   a  |
    T = | sin(θ)cos(α)  cos(θ)cos(α)  -sin(α)  -d·sin(α) |
        | sin(θ)sin(α)  cos(θ)sin(α)   cos(α)   d·cos(α) |
        |    0           0           0       1      |

    Args:
        theta: 关节角度
        d: 连杆偏距
        a: 连杆长度
        alpha: 连杆扭角

    Returns:
        4x4 齐次变换矩阵
    """
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1]
    ])


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    RPY 角转旋转矩阵

    Args:
        roll, pitch, yaw: 欧拉角 (弧度)

    Returns:
        3x3 旋转矩阵
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(roll), -np.sin(roll)],
                    [0, np.sin(roll), np.cos(roll)]])
    R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                    [0, 1, 0],
                    [-np.sin(pitch), 0, np.cos(pitch)]])
    R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                    [np.sin(yaw), np.cos(yaw), 0],
                    [0, 0, 1]])
    return R_z @ R_y @ R_x


def rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """
    旋转矩阵转 RPY 角

    Args:
        R: 3x3 旋转矩阵

    Returns:
        (roll, pitch, yaw) 弧度
    """
    pitch = np.arcsin(-np.clip(R[0, 2], -1, 1))

    if np.abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[1, 2], R[2, 2])
        yaw = np.arctan2(R[0, 1], R[0, 0])
    else:
        roll = 0
        yaw = np.arctan2(-R[1, 0], R[1, 1])

    return roll, pitch, yaw


def rotation_matrix_to_quaternion(R: np.ndarray) -> List[float]:
    """
    旋转矩阵转四元数

    Args:
        R: 3x3 旋转矩阵

    Returns:
        [w, x, y, z] 四元数
    """
    trace = np.trace(R)

    if trace > 0:
        s = np.sqrt(trace + 1) * 2
        qw = 0.25 * s
        qx = (R[2, 1] - R[1, 2]) / s
        qy = (R[0, 2] - R[2, 0]) / s
        qz = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return [qw, qx, qy, qz]


# ============================================================================
# SO100 运动学模型
# ============================================================================

class SO100Kinematics:
    """
    SO100 机械臂运动学模型

    基于 URDF 文件参数构建的 6-DOF 运动学模型
    """

    # DH 参数 (从 URDF 提取)
    DH_PARAMS = [
        # (d, a, alpha, theta_offset)
        (0.0452, 0, np.pi/2, 0),           # Joint 1: shoulder_pan
        (0, 0.1025, 0, -np.pi/2),          # Joint 2: shoulder_lift
        (0, 0.11257, 0, np.pi/2),          # Joint 3: elbow_flex
        (0.1401, 0, -np.pi/2, 0),          # Joint 4: wrist_flex
        (0, 0, np.pi/2, np.pi/2),          # Joint 5: wrist_roll
        (0.0845, 0, 0, 0),                 # Joint 6: gripper
    ]

    # 关节限位 (弧度)
    JOINT_LIMITS = [
        (-2.0, 2.0),       # shoulder_pan
        (0.0, 3.5),        # shoulder_lift
        (-np.pi, 0),       # elbow_flex
        (-2.5, 1.2),       # wrist_flex
        (-np.pi, np.pi),   # wrist_roll
        (-0.2, 2.0),       # gripper
    ]

    # 关节名称
    JOINT_NAMES = [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper"
    ]

    # 连杆长度
    LINK_LENGTHS = {
        "base_to_shoulder": 0.0452,
        "shoulder_to_upper": 0.1025,
        "upper_to_lower": 0.11257,
        "lower_to_wrist": 0.1349,
        "wrist_to_gripper": 0.0601,
        "gripper_to_jaw": 0.0244,
    }

    # 工作空间半径
    WORKSPACE_RADIUS = 0.47387

    def __init__(self, urdf_path: Optional[str] = None):
        """
        初始化运动学模型

        Args:
            urdf_path: 可选的 URDF 文件路径，用于验证参数
        """
        self.n_joints = 6

        # 解析 DH 参数
        self.d = [p[0] for p in self.DH_PARAMS]
        self.a = [p[1] for p in self.DH_PARAMS]
        self.alpha = [p[2] for p in self.DH_PARAMS]
        self.theta_offset = [p[3] for p in self.DH_PARAMS]

        # 如果提供了 URDF 文件，验证参数
        if urdf_path:
            self._verify_with_urdf(urdf_path)

    def _verify_with_urdf(self, urdf_path: str):
        """使用 URDF 文件验证参数"""
        parser = URDFParser(urdf_path)
        if parser.parse():
            print(f"URDF 解析成功: {parser.robot_name}")
            print(f"关节数量: {len(parser.joints)}")

    # ========================================================================
    # 正运动学
    # ========================================================================

    def forward_kinematics(self, q: np.ndarray, return_all: bool = False) -> np.ndarray:
        """
        正运动学计算

        Args:
            q: 关节角度 [q1, q2, q3, q4, q5, q6] (弧度)
            return_all: 是否返回所有连杆的变换矩阵

        Returns:
            T: 末端执行器相对于基座的 4x4 齐次变换矩阵
            或 (T_all, T): 如果 return_all=True
        """
        q = np.asarray(q).flatten()
        if len(q) != 6:
            raise ValueError(f"需要 6 个关节角度，收到 {len(q)} 个")

        T_all = []
        T = np.eye(4)

        for i in range(6):
            theta = q[i] + self.theta_offset[i]
            Ti = dh_transform(theta, self.d[i], self.a[i], self.alpha[i])
            T = T @ Ti
            T_all.append(T.copy())

        if return_all:
            return T_all, T
        return T

    def get_end_effector_pose(self, q: np.ndarray) -> dict:
        """
        获取末端执行器位姿

        Args:
            q: 关节角度 (弧度)

        Returns:
            {
                'position': [x, y, z] (米),
                'orientation': {
                    'rpy': [roll, pitch, yaw] (弧度),
                    'quaternion': [w, x, y, z],
                    'rotation_matrix': 3x3 矩阵
                }
            }
        """
        T = self.forward_kinematics(q)

        position = T[:3, 3].tolist()
        R = T[:3, :3]

        roll, pitch, yaw = rotation_matrix_to_rpy(R)
        quaternion = rotation_matrix_to_quaternion(R)

        return {
            'position': position,
            'orientation': {
                'rpy': [roll, pitch, yaw],
                'quaternion': quaternion,
                'rotation_matrix': R.tolist()
            }
        }

    # ========================================================================
    # 逆运动学
    # ========================================================================

    def inverse_kinematics(self, target_pose: np.ndarray,
                          q0: Optional[np.ndarray] = None,
                          method: str = 'optimize') -> np.ndarray:
        """
        逆运动学求解

        Args:
            target_pose: 目标位姿
                - 4x4 齐次变换矩阵，或
                - [x, y, z, roll, pitch, yaw] 6维向量
            q0: 初始关节角度猜测 (弧度)
            method: 求解方法
                - 'optimize': SLSQP 优化
                - 'fsolve': scipy fsolve
                - 'jacobian': 雅可比迭代

        Returns:
            q: 关节角度 [q1, q2, q3, q4, q5, q6] (弧度)
        """
        # 处理输入格式
        target_pose = np.asarray(target_pose)

        if target_pose.shape == (6,):
            # [x, y, z, roll, pitch, yaw]
            x, y, z, roll, pitch, yaw = target_pose
            R = rpy_to_rotation_matrix(roll, pitch, yaw)
            T_target = np.eye(4)
            T_target[:3, :3] = R
            T_target[:3, 3] = [x, y, z]
        elif target_pose.shape == (4, 4):
            T_target = target_pose
        else:
            raise ValueError(f"目标位姿格式错误，期望 (6,) 或 (4, 4)，收到 {target_pose.shape}")

        # 初始猜测
        if q0 is None:
            q0 = np.array([0, -np.pi/4, np.pi/4, -np.pi/3, 0, -np.pi/4])

        # 目标位置和旋转
        target_pos = T_target[:3, 3]
        target_rot = T_target[:3, :3]

        def objective(q):
            """优化目标函数"""
            T = self.forward_kinematics(q)
            pos_error = np.sum((T[:3, 3] - target_pos) ** 2)
            rot_error = np.sum((T[:3, :3] - target_rot) ** 2)
            return pos_error * 1000 + rot_error

        def constraint_eq(q):
            """等式约束"""
            T = self.forward_kinematics(q)
            return np.concatenate([
                T[:3, 3] - target_pos,
                (T[:3, :3] - target_rot).flatten()
            ])

        # 关节限位约束
        bounds = self.JOINT_LIMITS

        if method == 'optimize':
            result = minimize(
                objective,
                q0,
                method='SLSQP',
                bounds=bounds,
                options={'ftol': 1e-6, 'maxiter': 1000}
            )
            return result.x

        elif method == 'fsolve':
            result = fsolve(constraint_eq, q0, full_output=True)
            return result[0]

        elif method == 'jacobian':
            return self._ik_jacobian(T_target, q0)

        else:
            raise ValueError(f"未知的 IK 方法: {method}")

    def _ik_jacobian(self, T_target: np.ndarray, q0: np.ndarray,
                    max_iter: int = 100, tol: float = 1e-6) -> np.ndarray:
        """
        基于雅可比矩阵的逆运动学求解 (数值迭代法)
        """
        q = q0.copy()
        limits = np.array(self.JOINT_LIMITS)

        for _ in range(max_iter):
            T = self.forward_kinematics(q)

            # 位置误差
            pos_error = T_target[:3, 3] - T[:3, 3]

            # 旋转误差 (轴角表示)
            R_error = T_target[:3, :3] @ T[:3, :3].T
            rot_error_vec = 0.5 * np.array([
                R_error[2, 1] - R_error[1, 2],
                R_error[0, 2] - R_error[2, 0],
                R_error[1, 0] - R_error[0, 1]
            ])

            error = np.concatenate([pos_error, rot_error_vec])

            if np.linalg.norm(error) < tol:
                break

            # 阻尼最小二乘法
            J = self.jacobian(q)
            lambda_damp = 0.01
            dq = J.T @ np.linalg.solve(J @ J.T + lambda_damp * np.eye(6), error)
            q = q + dq

            # 应用限位
            q = np.clip(q, limits[:, 0], limits[:, 1])

        return q

    # ========================================================================
    # 雅可比矩阵
    # ========================================================================

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        计算几何雅可比矩阵

        Args:
            q: 关节角度 (弧度)

        Returns:
            J: 6x6 雅可比矩阵
                | Jv |  线速度雅可比
                | Jw |  角速度雅可比
        """
        q = np.asarray(q).flatten()
        T_all, T_end = self.forward_kinematics(q, return_all=True)
        p_end = T_end[:3, 3]

        J = np.zeros((6, 6))

        for i in range(6):
            z_i = T_all[i][:3, :3] @ np.array([0, 0, 1])
            p_i = T_all[i][:3, 3]

            # 旋转关节
            J[:3, i] = np.cross(z_i, p_end - p_i)
            J[3:, i] = z_i

        return J

    # ========================================================================
    # 工作空间分析
    # ========================================================================

    def get_joint_limits(self) -> List[Tuple[float, float]]:
        """获取关节限位 (弧度)"""
        return self.JOINT_LIMITS.copy()

    def get_joint_limits_deg(self) -> List[Tuple[float, float]]:
        """获取关节限位 (度)"""
        return [(np.rad2deg(l[0]), np.rad2deg(l[1])) for l in self.JOINT_LIMITS]

    def check_joint_limits(self, q: np.ndarray) -> Tuple[bool, List[str]]:
        """
        检查关节角度是否在限位内

        Returns:
            (is_valid, messages)
        """
        q = np.asarray(q).flatten()
        messages = []
        is_valid = True

        for i, (q_val, (min_val, max_val)) in enumerate(zip(q, self.JOINT_LIMITS)):
            if q_val < min_val or q_val > max_val:
                is_valid = False
                messages.append(
                    f"关节 {i+1} ({self.JOINT_NAMES[i]}): "
                    f"{np.rad2deg(q_val):.1f}° 超出限位 "
                    f"[{np.rad2deg(min_val):.1f}°, {np.rad2deg(max_val):.1f}°]"
                )

        return is_valid, messages

    def in_workspace(self, position: np.ndarray) -> bool:
        """检查点是否在工作空间内"""
        r = np.linalg.norm(position)
        return r <= self.WORKSPACE_RADIUS

    # ========================================================================
    # 轨迹规划
    # ========================================================================

    def joint_trajectory(self, start_q: np.ndarray, end_q: np.ndarray,
                        num_points: int = 100) -> np.ndarray:
        """
        关节空间线性轨迹规划

        Args:
            start_q: 起始关节角度 (弧度)
            end_q: 结束关节角度 (弧度)
            num_points: 轨迹点数

        Returns:
            trajectory: (num_points, 6) 关节角度数组
        """
        trajectory = np.zeros((num_points, 6))
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            trajectory[i] = start_q + t * (end_q - start_q)
        return trajectory

    def scurve_trajectory(self, start_q: np.ndarray, end_q: np.ndarray,
                         num_points: int = 100) -> np.ndarray:
        """
        S 曲线轨迹规划 (平滑加减速)

        Args:
            start_q: 起始关节角度 (弧度)
            end_q: 结束关节角度 (弧度)
            num_points: 轨迹点数

        Returns:
            trajectory: (num_points, 6) 关节角度数组
        """
        trajectory = np.zeros((num_points, 6))
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            smooth_t = t * t * (3 - 2 * t)  # S 曲线
            trajectory[i] = start_q + smooth_t * (end_q - start_q)
        return trajectory

    def cartesian_trajectory(self, start_pose: np.ndarray, end_pose: np.ndarray,
                           num_points: int = 100) -> np.ndarray:
        """
        笛卡尔空间直线轨迹规划

        Args:
            start_pose: 起始位姿 [x, y, z, roll, pitch, yaw]
            end_pose: 结束位姿 [x, y, z, roll, pitch, yaw]
            num_points: 轨迹点数

        Returns:
            trajectory: (num_points, 6) 位姿数组
        """
        trajectory = np.zeros((num_points, 6))
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0
            trajectory[i] = start_pose + t * (end_pose - start_pose)
        return trajectory

    # ========================================================================
    # 惯性参数
    # ========================================================================

    def get_inertial_parameters(self) -> Dict[str, Dict]:
        """获取惯性参数 (从 URDF 提取)"""
        return {
            "base": {
                "mass": 1.0,
                "center_of_mass": [0, 0, 0],
                "inertia": [0.01, 0.01, 0.01, 0, 0, 0]
            },
            "shoulder": {
                "mass": 0.119226,
                "center_of_mass": [-9.07886e-05, 0.0590972, 0.031089],
                "inertia": [5.94278e-05, 5.89975e-05, 3.13712e-05, 0, 0, 0]
            },
            "upper_arm": {
                "mass": 0.162409,
                "center_of_mass": [-1.72052e-05, 0.0701802, 0.00310545],
                "inertia": [0.000213312, 0.000167164, 7.01522e-05, 0, 0, 0]
            },
            "lower_arm": {
                "mass": 0.147968,
                "center_of_mass": [-0.00339604, 0.00137796, 0.0768007],
                "inertia": [0.000138803, 0.000107748, 4.84242e-05, 0, 0, 0]
            },
            "wrist": {
                "mass": 0.0661321,
                "center_of_mass": [-0.00852653, -0.0352279, -2.34622e-05],
                "inertia": [3.45403e-05, 2.39041e-05, 1.94704e-05, 0, 0, 0]
            },
            "gripper": {
                "mass": 0.0929859,
                "center_of_mass": [0.00552377, -0.0280167, 0.000483583],
                "inertia": [5.03136e-05, 4.64098e-05, 2.72961e-05, 0, 0, 0]
            },
            "jaw": {
                "mass": 0.0202444,
                "center_of_mass": [-0.00161745, -0.0303473, 0.000449646],
                "inertia": [1.11265e-05, 8.99651e-06, 2.99548e-06, 0, 0, 0]
            }
        }


# ============================================================================
# 舵机位置转换
# ============================================================================

class ServoConverter:
    """舵机位置与角度转换 (STS3215)"""

    SERVO_CENTER = 2047      # 中位位置
    SERVO_RANGE = 270.0      # 转动范围 (度)
    SERVO_MIN = 0            # 最小位置
    SERVO_MAX = 4095         # 最大位置

    @staticmethod
    def angle_to_position(angle_deg: float) -> int:
        """
        角度转舵机位置

        Args:
            angle_deg: 角度 (度)，相对于中位

        Returns:
            舵机位置 (0-4095)
        """
        pos = int(ServoConverter.SERVO_CENTER +
                  (angle_deg * 4095 / ServoConverter.SERVO_RANGE))
        return max(ServoConverter.SERVO_MIN,
                   min(ServoConverter.SERVO_MAX, pos))

    @staticmethod
    def position_to_angle(pos: int) -> float:
        """
        舵机位置转角度

        Args:
            pos: 舵机位置 (0-4095)

        Returns:
            角度 (度)，相对于中位
        """
        return (pos - ServoConverter.SERVO_CENTER) * ServoConverter.SERVO_RANGE / 4095

    @staticmethod
    def rad_to_position(angle_rad: float) -> int:
        """弧度转舵机位置"""
        return ServoConverter.angle_to_position(np.rad2deg(angle_rad))

    @staticmethod
    def position_to_rad(pos: int) -> float:
        """舵机位置转弧度"""
        return np.deg2rad(ServoConverter.position_to_angle(pos))

    @staticmethod
    def joints_to_positions(q: np.ndarray) -> List[int]:
        """
        关节角度数组转舵机位置数组

        Args:
            q: 关节角度 (弧度)

        Returns:
            舵机位置列表
        """
        return [ServoConverter.rad_to_position(qi) for qi in q]

    @staticmethod
    def positions_to_joints(positions: List[int]) -> np.ndarray:
        """
        舵机位置数组转关节角度数组

        Args:
            positions: 舵机位置列表

        Returns:
            关节角度 (弧度)
        """
        return np.array([ServoConverter.position_to_rad(p) for p in positions])


# ============================================================================
# 测试和演示
# ============================================================================

def test_kinematics():
    """测试运动学模型"""
    print("=" * 60)
    print("SO100 运动学模型测试")
    print("=" * 60)

    model = SO100Kinematics()

    # 测试关节角度 (弧度)
    q_test = np.array([
        0.0,           # shoulder_pan
        -np.pi/4,      # shoulder_lift
        np.pi/4,       # elbow_flex
        -np.pi/3,      # wrist_flex
        0.0,           # wrist_roll
        -np.pi/4       # gripper
    ])

    print("\n1. 正运动学测试")
    print("-" * 40)
    print(f"输入关节角度 (度): {np.rad2deg(q_test)}")

    T = model.forward_kinematics(q_test)
    print(f"\n末端变换矩阵:\n{T}")

    pose = model.get_end_effector_pose(q_test)
    print(f"\n末端位置 (m): {pose['position']}")
    print(f"末端姿态 RPY (度): {np.rad2deg(pose['orientation']['rpy'])}")

    print("\n2. 逆运动学测试")
    print("-" * 40)

    q_ik = model.inverse_kinematics(T, q0=q_test, method='optimize')
    print(f"输入关节角度 (度): {np.rad2deg(q_test)}")
    print(f"IK 求解结果 (度):   {np.rad2deg(q_ik)}")
    print(f"误差 (度):         {np.rad2deg(q_test - q_ik)}")

    # 验证 IK 结果
    T_ik = model.forward_kinematics(q_ik)
    pos_error = np.linalg.norm(T[:3, 3] - T_ik[:3, 3])
    print(f"位置误差: {pos_error * 1000:.2f} mm")

    print("\n3. 雅可比矩阵测试")
    print("-" * 40)
    J = model.jacobian(q_test)
    print(f"雅可比矩阵:\n{J}")
    print(f"\n条件数: {np.linalg.cond(J):.2f}")

    print("\n4. 关节限位检查")
    print("-" * 40)
    is_valid, messages = model.check_joint_limits(q_test)
    for msg in messages:
        print(f"  {msg}")
    if is_valid:
        print("  所有关节角度在限位内")

    print("\n5. 舵机位置转换")
    print("-" * 40)
    positions = ServoConverter.joints_to_positions(q_test)
    print(f"关节角度 (度): {np.rad2deg(q_test)}")
    print(f"舵机位置: {positions}")
    print(f"转回角度 (度): {np.rad2deg(ServoConverter.positions_to_joints(positions))}")

    print("\n6. 轨迹规划测试")
    print("-" * 40)
    q_start = np.array([0, -np.pi/6, np.pi/6, -np.pi/4, 0, -np.pi/6])
    q_end = np.array([np.pi/6, -np.pi/3, np.pi/3, -np.pi/2, np.pi/6, -np.pi/3])

    traj = model.scurve_trajectory(q_start, q_end, num_points=5)
    print("S 曲线轨迹 (5 点):")
    for i, q in enumerate(traj):
        print(f"  点 {i+1}: {np.rad2deg(q)}")

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)


def parse_urdf_demo():
    """URDF 解析演示"""
    # 项目根目录
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
    urdf_path = os.path.join(project_root, "SO100描述文件", "so100.urdf")

    print("=" * 60)
    print("SO100 URDF 解析")
    print("=" * 60)

    parser = URDFParser(urdf_path)
    if parser.parse():
        print(f"\n机器人名称: {parser.robot_name}")
        print(f"\n链接 ({len(parser.links)}):")
        for name, link in parser.links.items():
            print(f"  - {name}: 质量={link.mass:.4f} kg")

        print(f"\n关节 ({len(parser.joints)}):")
        for name, joint in parser.joints.items():
            print(f"  - {name}: {joint.parent} -> {joint.child}")
            print(f"    类型: {joint.joint_type}")
            print(f"    限位: [{np.rad2deg(joint.limit_lower):.1f}°, {np.rad2deg(joint.limit_upper):.1f}°]")

        print("\n运动学链:")
        chain = parser.get_kinematic_chain()
        for joint in chain:
            print(f"  {joint.parent} -> [{joint.name}] -> {joint.child}")


def main():
    """命令行入口"""
    import argparse

    parser = argparse.ArgumentParser(description='SO100 运动学模型')
    parser.add_argument('--test', action='store_true', help='运行测试')
    parser.add_argument('--parse-urdf', action='store_true', help='解析 URDF 文件')
    parser.add_argument('--fk', nargs=6, type=float, metavar='Q',
                       help='计算正运动学 (6 个关节角度，弧度)')
    parser.add_argument('--ik', nargs=6, type=float, metavar='POSE',
                       help='计算逆运动学 (x y z roll pitch yaw)')

    args = parser.parse_args()

    if args.test:
        test_kinematics()
    elif args.parse_urdf:
        parse_urdf_demo()
    elif args.fk:
        model = SO100Kinematics()
        q = np.array(args.fk)
        pose = model.get_end_effector_pose(q)
        print(f"输入关节角度 (rad): {q}")
        print(f"输入关节角度 (deg): {np.rad2deg(q)}")
        print(f"末端位置 (m): {pose['position']}")
        print(f"末端姿态 RPY (deg): {np.rad2deg(pose['orientation']['rpy'])}")
    elif args.ik:
        model = SO100Kinematics()
        target = np.array(args.ik)
        q = model.inverse_kinematics(target)
        is_valid, messages = model.check_joint_limits(q)
        print(f"目标位姿 [x,y,z,r,p,y]: {target}")
        print(f"关节角度 (rad): {q}")
        print(f"关节角度 (deg): {np.rad2deg(q)}")
        if messages:
            for msg in messages:
                print(f"  警告: {msg}")
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
