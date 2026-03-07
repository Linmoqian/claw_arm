#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂手眼标定工具（完整版）
使用棋盘格进行 Eye-in-Hand 标定

流程：
1. 相机内参标定（可选）
2. 手眼标定数据采集
3. 计算摄像头到末端执行器的变换

兼容 OpenCV 4.13+
"""

import cv2
import numpy as np
import json
import time
from pathlib import Path
from typing import List, Tuple, Optional, Dict
import sys

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from SDK import PortHandler, PacketHandler

# ============================================================
#  SO100 机械臂 DH 参数（简化）
# ============================================================

# SO100 连杆长度（单位：米），根据实际机械臂测量
# 可通过测量或查看机械臂规格书获取更精确的值
SO100_DH_PARAMS = {
    "d1": 0.065,    # 基座到关节2的垂直偏移
    "a2": 0.0,      # 关节2到关节3的连杆长度
    "a3": 0.140,    # 关节3到关节4的连杆长度（大臂长度）
    "d4": 0.0,      # 关节4偏移
    "d5": 0.095,    # 关节5到关节6的连杆长度（小臂长度）
    "d6": 0.060,    # 末端长度
}

# 关节角度范围（转换为弧度）
JOOUNT_MIN = np.deg2rad(-90)
JOOUNT_MAX = np.deg2rad(90)


def joint_position_to_angle(pos: int, min_pos: int = 0, max_pos: int = 4095) -> float:
    """将舵机位置值（0-4095）转换为角度（弧度）"""
    # 归一化到 [-1, 1]
    normalized = (pos - 2047) / 2047.0
    # 映射到角度范围
    angle = normalized * np.pi  # ±180度，实际使用时可能需要调整
    return angle


def forward_kinematics_simplified(joint_positions: List[int]) -> np.ndarray:
    """
    SO100 简化的正向运动学

    Args:
        joint_positions: 6个关节位置 [j1, j2, j3, j4, j5, j6]

    Returns:
        4x4 齐次变换矩阵（基座到末端）
    """
    # 将舵机位置转换为角度（弧度）
    theta1 = joint_position_to_angle(joint_positions[0])  # shoulder_pan
    theta2 = joint_position_to_angle(joint_positions[1])  # shoulder_lift
    theta3 = joint_position_to_angle(joint_positions[2])  # elbow_flex
    theta4 = joint_position_to_angle(joint_positions[3])  # wrist_flex
    theta5 = joint_position_to_angle(joint_positions[4])  # wrist_roll
    theta6 = joint_position_to_angle(joint_positions[5])  # gripper

    # 简化的几何模型（假设6R机械臂）
    # 使用 DH 参数法构建变换矩阵

    d = SO100_DH_PARAMS

    # 关节变换矩阵（简化 DH）
    def dh_transform(alpha, a, d, theta):
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        return np.array([
            [ct, -st, 0, a],
            [st * ca, ct * ca, -sa, -d * sa],
            [st * sa, ct * sa, ca, d * ca],
            [0, 0, 0, 1]
        ])

    # 基座到关节1（旋转）
    T01 = dh_transform(0, 0, d["d1"], theta1)

    # 关节1到关节2（大臂）
    T12 = dh_transform(np.pi/2, 0, 0, theta2 + np.pi/2)  # 调整零位偏移

    # 关节2到关节3（大臂延伸）
    T23 = dh_transform(0, d["a3"], 0, theta3)

    # 关节3到关节4（小臂）
    T34 = dh_transform(np.pi/2, 0, 0, theta4)

    # 关节4到关节5（小臂延伸）
    T45 = dh_transform(-np.pi/2, 0, d["d5"], theta5)

    # 关节5到末端
    T56 = dh_transform(0, 0, d["d6"], theta6)

    # 级联变换
    T06 = T01 @ T12 @ T23 @ T34 @ T45 @ T56

    return T06


# ============================================================
#  棋盘格标定器
# ============================================================

class ChessboardCalibrator:
    """棋盘格标定器"""

    def __init__(self, pattern_size: Tuple[int, int] = (9, 6), square_size: float = 0.025):
        """
        Args:
            pattern_size: 内角点数量 (cols, rows)
            square_size: 方格边长（米）
        """
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 相机内参（初始化为 None，需要标定）
        self.camera_matrix = None
        self.dist_coeffs = None

        # 准备 3D 点
        self.obj_points = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.obj_points[:, :2] = np.mgrid[0:self.pattern_size[0],
                                          0:self.pattern_size[1]].T.reshape(-1, 2)
        self.obj_points *= self.square_size

    def detect(self, image: np.ndarray,
               camera_matrix: Optional[np.ndarray] = None,
               dist_coeffs: Optional[np.ndarray] = None) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], bool]:
        """
        检测棋盘格

        Args:
            image: 输入图像
            camera_matrix: 相机内参矩阵
            dist_coeffs: 畸变系数

        Returns:
            (rvec, tvec, success): 旋转向量、平移向量、是否成功
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 查找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if not ret:
            return None, None, False

        # 精细化角点位置
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)

        # 使用内参
        if camera_matrix is None:
            camera_matrix = self.camera_matrix
        if dist_coeffs is None:
            dist_coeffs = self.dist_coeffs

        # 如果没有内参，使用默认值
        if camera_matrix is None:
            camera_matrix = np.array([[1000, 0, image.shape[1]/2],
                                      [0, 1000, image.shape[0]/2],
                                      [0, 0, 1]], dtype=float)
        if dist_coeffs is None:
            dist_coeffs = np.zeros(5)

        # 估计姿态
        ret, rvec, tvec = cv2.solvePnP(self.obj_points, corners, camera_matrix, dist_coeffs)

        if not ret:
            return None, None, False

        return rvec, tvec, True

    def draw(self, image: np.ndarray, rvec: np.ndarray, tvec: np.ndarray,
             camera_matrix: Optional[np.ndarray] = None,
             dist_coeffs: Optional[np.ndarray] = None) -> np.ndarray:
        """在图像上绘制检测结果"""
        result = image.copy()

        # 绘制棋盘格
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)
        if ret:
            cv2.drawChessboardCorners(result, self.pattern_size, corners, ret)

        # 绘制坐标轴
        if rvec is not None and tvec is not None:
            if camera_matrix is None:
                camera_matrix = self.camera_matrix if self.camera_matrix is not None else np.array([[1000, 0, image.shape[1]/2], [0, 1000, image.shape[0]/2], [0, 0, 1]], dtype=float)
            if dist_coeffs is None:
                dist_coeffs = self.dist_coeffs if self.dist_coeffs is not None else np.zeros(5)

            cv2.drawFrameAxes(result, camera_matrix, dist_coeffs, rvec, tvec, self.square_size * 2)

        return result

    def generate_board_image(self, output_path: str = "chessboard.png",
                             squares_x: int = 10, squares_y: int = 7,
                             square_size_px: int = 100) -> None:
        """
        生成棋盘格图像

        Args:
            output_path: 输出路径
            squares_x: X 方向方格数量（比内角点多1）
            squares_y: Y 方向方格数量（比内角点多1）
            square_size_px: 每个方格的像素大小
        """
        width = squares_x * square_size_px
        height = squares_y * square_size_px

        img = np.zeros((height, width), dtype=np.uint8)

        for y in range(squares_y):
            for x in range(squares_x):
                if (x + y) % 2 == 0:
                    y1 = y * square_size_px
                    y2 = (y + 1) * square_size_px
                    x1 = x * square_size_px
                    x2 = (x + 1) * square_size_px
                    img[y1:y2, x1:x2] = 255

        cv2.imwrite(output_path, img)

        # 计算实际尺寸
        width_m = squares_x * self.square_size
        height_m = squares_y * self.square_size

        print(f"棋盘格已保存到: {output_path}")
        print(f"打印尺寸: {width_m*100:.1f}cm x {height_m*100:.1f}cm")
        print(f"图像尺寸: {width} x {height} 像素")
        print(f"\n打印建议:")
        print(f"1. 确保每个方格边长为 {self.square_size*100:.1f}cm")
        print(f"2. 使用白纸打印，确保黑白对比清晰")
        print(f"3. 将打印好的棋盘格贴在平整表面上")


# ============================================================
#  相机内参标定
# ============================================================

class CameraCalibrator:
    """相机内参标定器"""

    def __init__(self, pattern_size: Tuple[int, int] = (9, 6), square_size: float = 0.025):
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 标定数据
        self.obj_points: List[np.ndarray] = []
        self.img_points: List[np.ndarray] = []

        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rms_error = None

        # 准备 3D 点
        objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
        objp *= square_size
        self.objp = objp

    def detect_chessboard(self, image: np.ndarray) -> Tuple[bool, np.ndarray]:
        """检测棋盘格角点"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if ret:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)

        return ret, corners

    def add_sample(self, image: np.ndarray) -> bool:
        """添加一个标定样本"""
        ret, corners = self.detect_chessboard(image)

        if ret:
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            return True
        return False

    def calibrate(self, image_size: Tuple[int, int]) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        执行相机标定

        Returns:
            (camera_matrix, dist_coeffs, rms_error)
        """
        if len(self.obj_points) < 10:
            raise ValueError(f"样本数量不足，需要至少 10 个，当前: {len(self.obj_points)}")

        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, image_size, None, None
        )

        if not ret:
            raise RuntimeError("相机标定失败")

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rms_error = ret

        return camera_matrix, dist_coeffs, ret

    def save(self, filepath: str):
        """保存标定结果"""
        data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "rms_error": float(self.rms_error),
            "image_size": [int(self.camera_matrix[0, 2] * 2), int(self.camera_matrix[1, 2] * 2)],
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"相机内参已保存到: {filepath}")

    @classmethod
    def load(cls, filepath: str) -> Tuple[np.ndarray, np.ndarray]:
        """加载标定结果"""
        with open(filepath, 'r') as f:
            data = json.load(f)

        camera_matrix = np.array(data["camera_matrix"])
        dist_coeffs = np.array(data["dist_coeffs"])

        print(f"相机内参已加载: {filepath}")
        print(f"RMS 误差: {data['rms_error']:.4f}")

        return camera_matrix, dist_coeffs


# ============================================================
#  标定姿态数据
# ============================================================

class CalibrationPose:
    """单个标定姿态"""

    def __init__(self, joint_positions: List[int],
                 gripper_to_base: np.ndarray,
                 board_to_camera: np.ndarray):
        """
        Args:
            joint_positions: 6个关节的位置 [j1, j2, j3, j4, j5, j6]
            gripper_to_base: 末端到基座的变换矩阵 4x4
            board_to_camera: 标定板到相机的变换矩阵 4x4
        """
        self.joint_positions = joint_positions
        self.gripper_to_base = gripper_to_base
        self.board_to_camera = board_to_camera

    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "joint_positions": self.joint_positions,
            "gripper_to_base": self.gripper_to_base.tolist(),
            "board_to_camera": self.board_to_camera.tolist()
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'CalibrationPose':
        """从字典创建"""
        return cls(
            joint_positions=data["joint_positions"],
            gripper_to_base=np.array(data["gripper_to_base"]),
            board_to_camera=np.array(data["board_to_camera"])
        )


# ============================================================
#  手眼标定器
# ============================================================

class HandEyeCalibrator:
    """手眼标定器（Eye-in-Hand）"""

    def __init__(self, port: str = "COM7", camera_id: int = 0,
                 pattern_size: Tuple[int, int] = (9, 6),
                 square_size: float = 0.025):
        """
        Args:
            port: 串口
            camera_id: 摄像头ID
            pattern_size: 棋盘格内角点数量
            square_size: 方格边长（米）
        """
        self.port = port
        self.camera_id = camera_id

        # 初始化硬件
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(0.0)
        self.cap = cv2.VideoCapture(camera_id)

        # 棋盘格标定器
        self.board = ChessboardCalibrator(pattern_size, square_size)

        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None

        # 标定数据
        self.poses: List[CalibrationPose] = []

    def connect(self) -> bool:
        """连接硬件"""
        if not self.port_handler.openPort():
            print(f"无法打开串口 {self.port}")
            return False

        self.port_handler.setBaudRate(1000000)

        if not self.cap.isOpened():
            print(f"无法打开摄像头 {self.camera_id}")
            return False

        print("硬件连接成功")
        return True

    def disconnect(self):
        """断开连接"""
        if self.port_handler:
            self.port_handler.closePort()
        if self.cap:
            self.cap.release()

    def set_camera_intrinsics(self, camera_matrix: np.ndarray, dist_coeffs: np.ndarray):
        """设置相机内参"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.board.camera_matrix = camera_matrix
        self.board.dist_coeffs = dist_coeffs
        print("相机内参已设置")

    def load_camera_calibration(self, filepath: str):
        """加载相机标定结果"""
        self.camera_matrix, self.dist_coeffs = CameraCalibrator.load(filepath)
        self.board.camera_matrix = self.camera_matrix
        self.board.dist_coeffs = self.dist_coeffs

    def read_joint_positions(self) -> List[int]:
        """读取所有关节位置"""
        positions = []
        for motor_id in range(1, 7):
            data, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, 56
            )
            if result == 0:
                positions.append(data)
            else:
                positions.append(2047)
        return positions

    def compute_gripper_pose(self, joint_positions: List[int]) -> np.ndarray:
        """计算末端执行器相对于基座的位姿"""
        return forward_kinematics_simplified(joint_positions)

    def capture_pose(self) -> Tuple[bool, str]:
        """
        捕获当前姿态

        Returns:
            (success, message): 是否成功和消息
        """
        # 读取关节位置
        joint_positions = self.read_joint_positions()

        # 检测棋盘格
        ret, frame = self.cap.read()
        if not ret:
            return False, "无法读取摄像头"

        rvec, tvec, success = self.board.detect(frame, self.camera_matrix, self.dist_coeffs)

        if not success:
            return False, "未检测到棋盘格，请确保棋盘格在视野内且清晰可见"

        # 计算标定板到相机的变换矩阵
        R, _ = cv2.Rodrigues(rvec)
        T_board_to_cam = np.eye(4)
        T_board_to_cam[:3, :3] = R
        T_board_to_cam[:3, 3] = tvec.flatten()

        # 计算末端到基座的变换矩阵
        T_gripper_to_base = self.compute_gripper_pose(joint_positions)

        # 保存姿态
        pose = CalibrationPose(joint_positions, T_gripper_to_base, T_board_to_cam)
        self.poses.append(pose)

        msg = f"成功捕获姿态 {len(self.poses)}"
        print(msg)
        return True, msg

    def interactive_calibration(self, min_poses: int = 15):
        """交互式标定流程"""
        print("\n" + "=" * 60)
        print("SO100 手眼标定工具（棋盘格版）")
        print("=" * 60)

        # 显示相机内参状态
        if self.camera_matrix is None:
            print("\n警告: 未设置相机内参，使用默认值（不推荐）")
            print("建议先运行相机标定: python hand_eye_calibration_simple.py --calibrate-camera")
        else:
            print("\n相机内参已加载")

        print("\n操作说明:")
        print("1. 将棋盘格放置在机械臂工作区域内")
        print("2. 移动机械臂到不同位置（保持棋盘格在视野内）")
        print("3. 在每个位置按 'c' 捕获姿态")
        print(f"4. 捕获至少 {min_poses} 个姿态")
        print("5. 按 'q' 完成标定")
        print("\n按键说明:")
        print("  c - 捕获姿态")
        print("  u - 撤销最后一个")
        print("  s - 保存中间数据")
        print("  q - 完成标定")
        print("\n提示: 尝试从不同角度和距离捕获姿态，确保棋盘格完全在视野内")

        window_name = "Hand-Eye Calibration"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("无法读取摄像头")
                break

            # 检测棋盘格
            rvec, tvec, detected = self.board.detect(frame, self.camera_matrix, self.dist_coeffs)

            if detected:
                frame = self.board.draw(frame, rvec, tvec, self.camera_matrix, self.dist_coeffs)

            # 显示状态
            color = (0, 255, 0) if len(self.poses) >= min_poses else (0, 165, 255)
            status = f"Poses: {len(self.poses)}/{min_poses}"
            cv2.putText(frame, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            if len(self.poses) >= min_poses:
                cv2.putText(frame, "Ready! Press 'q' to calibrate", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, f"Need {min_poses - len(self.poses)} more poses",
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 显示检测状态
            detect_status = "Detected" if detected else "Not Detected"
            detect_color = (0, 255, 0) if detected else (0, 0, 255)
            cv2.putText(frame, detect_status, (10, frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, detect_color, 2)

            cv2.imshow(window_name, frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                if len(self.poses) >= min_poses:
                    break
                else:
                    print(f"需要至少 {min_poses} 个姿态，当前: {len(self.poses)}")
            elif key == ord('c'):
                success, msg = self.capture_pose()
                if not success:
                    print(f"捕获失败: {msg}")
            elif key == ord('u'):
                if self.poses:
                    self.poses.pop()
                    print(f"撤销，剩余 {len(self.poses)} 个姿态")
            elif key == ord('s'):
                self.save_poses("calibration_poses.json")
            elif key == 27:  # ESC
                print("取消标定")
                return

        cv2.destroyWindow(window_name)

    def calibrate(self) -> Tuple[Optional[np.ndarray], float]:
        """
        执行手眼标定计算

        使用 OpenCV 的 calibrateHandEye 函数

        Returns:
            (X_matrix, error): 变换矩阵和误差
        """
        if len(self.poses) < 5:
            print("姿态数量不足，需要至少 5 个")
            return None, 0.0

        print("\n开始手眼标定计算...")
        print(f"使用 {len(self.poses)} 个姿态")

        # 准备数据
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for i, pose in enumerate(self.poses):
            # 基座到末端
            T_g2b = pose.gripper_to_base
            R_g2b = T_g2b[:3, :3]
            t_g2b = T_g2b[:3, 3]

            # 标定板到相机
            T_c2t = pose.board_to_camera
            R_c2t = T_c2t[:3, :3]
            t_c2t = T_c2t[:3, 3]

            # 转换为目标到相机
            R_t2c = R_c2t.T
            t_t2c = -R_c2t.T @ t_c2t

            R_gripper2base.append(R_g2b)
            t_gripper2base.append(t_g2b)
            R_target2cam.append(R_t2c)
            t_target2cam.append(t_t2c)

        # 转换为 numpy 数组
        R_gripper2base = np.array(R_gripper2base)
        t_gripper2base = np.array(t_gripper2base)
        R_target2cam = np.array(R_target2cam)
        t_target2cam = np.array(t_target2cam)

        # 使用 OpenCV 手眼标定
        # 尝试不同方法，选择误差最小的
        methods = [
            cv2.CALIB_HAND_EYE_TSAI,
            cv2.CALIB_HAND_EYE_PARK,
            cv2.CALIB_HAND_EYE_HORAUD,
            cv2.CALIB_HAND_EYE_ANDREFF,
            cv2.CALIB_HAND_EYE_DANIILIDIS,
        ]

        best_R = None
        best_t = None
        best_error = float('inf')

        for method in methods:
            try:
                R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                    R_gripper2base=R_gripper2base,
                    t_gripper2base=t_gripper2base,
                    R_target2cam=R_target2cam,
                    t_target2cam=t_target2cam,
                    method=method
                )

                # 计算误差
                error = self._compute_reprojection_error(
                    R_cam2gripper, t_cam2gripper,
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam
                )

                if error < best_error:
                    best_error = error
                    best_R = R_cam2gripper
                    best_t = t_cam2gripper

            except cv2.error:
                continue

        if best_R is None:
            print("手眼标定失败")
            return None, 0.0

        # 构造 4x4 变换矩阵
        X = np.eye(4)
        X[:3, :3] = best_R
        X[:3, 3] = best_t.flatten()

        print(f"\n标定完成!")
        print(f"最优方法误差: {best_error:.4f}")

        return X, best_error

    def _compute_reprojection_error(self, R_cam2gripper, t_cam2gripper,
                                    R_gripper2base, t_gripper2base,
                                    R_target2cam, t_target2cam) -> float:
        """计算重投影误差"""
        errors = []

        T_cam2gripper = np.eye(4)
        T_cam2gripper[:3, :3] = R_cam2gripper
        T_cam2gripper[:3, 3] = t_cam2gripper.flatten()

        for i in range(len(R_gripper2base)):
            # 基座到末端
            T_g2b = np.eye(4)
            T_g2b[:3, :3] = R_gripper2base[i]
            T_g2b[:3, 3] = t_gripper2base[i]

            # 目标到相机
            T_t2c = np.eye(4)
            T_t2c[:3, :3] = R_target2cam[i]
            T_t2c[:3, 3] = t_target2cam[i]

            # 计算: T_g2b @ X @ T_t2c 应该恒定
            predicted = T_g2b @ T_cam2gripper @ T_t2c

            # 比较平移部分的变化
            errors.append(np.linalg.norm(predicted[:3, 3]))

        # 误差是标准差，越小越好
        return np.std(errors) if errors else float('inf')

    def save_poses(self, filename: str):
        """保存姿态数据"""
        data = [pose.to_dict() for pose in self.poses]
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"姿态数据已保存到 {filename}")

    def load_poses(self, filename: str):
        """加载姿态数据"""
        with open(filename, 'r') as f:
            data = json.load(f)
        self.poses = [CalibrationPose.from_dict(d) for d in data]
        print(f"加载了 {len(self.poses)} 个姿态")

    def save_calibration(self, X: np.ndarray, error: float,
                         filename: str = "hand_eye_calibration.json"):
        """保存标定结果"""
        data = {
            "camera_to_gripper": X.tolist(),
            "rotation": X[:3, :3].tolist(),
            "translation": X[:3, 3].tolist(),
            "translation_mm": X[:3, 3].tolist(),
            "error": float(error),
            "pattern_size": list(self.board.pattern_size),
            "square_size": self.board.square_size,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        # 保存到项目目录
        output_path = Path(__file__).parent.parent / "skills" / "vision-control" / filename
        output_path.parent.mkdir(parents=True, exist_ok=True)

        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"标定结果已保存到: {output_path}")

        # 同时保存当前目录
        local_path = Path(__file__).parent / filename
        with open(local_path, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"标定结果已保存到: {local_path}")


# ============================================================
#  相机标定界面
# ============================================================

def run_camera_calibration(camera_id: int, pattern_size: Tuple[int, int],
                           square_size: float) -> CameraCalibrator:
    """运行相机内参标定"""
    calibrator = CameraCalibrator(pattern_size, square_size)
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"无法打开摄像头 {camera_id}")
        return None

    print("\n" + "=" * 60)
    print("相机内参标定")
    print("=" * 60)
    print("\n操作说明:")
    print("1. 将棋盘格放在摄像头前")
    print("2. 移动棋盘格到不同位置和角度")
    print("3. 按 'c' 捕获图像（需要至少 15 张）")
    print("4. 按 'q' 完成标定")
    print("\n提示: 尝试覆盖整个视野，包括边缘和角落")

    window_name = "Camera Calibration"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    min_samples = 15

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detected, corners = calibrator.detect_chessboard(frame)

        # 绘制检测结果
        display = frame.copy()
        if detected:
            cv2.drawChessboardCorners(display, pattern_size, corners, True)

        # 显示状态
        count = len(calibrator.obj_points)
        color = (0, 255, 0) if count >= min_samples else (0, 165, 255)
        status = f"Samples: {count}/{min_samples}"
        cv2.putText(display, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        if count >= min_samples:
            cv2.putText(display, "Ready! Press 'q' to calibrate", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, f"Need {min_samples - count} more samples",
                       (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        detect_status = "Detected" if detected else "Not Detected"
        detect_color = (0, 255, 0) if detected else (0, 0, 255)
        cv2.putText(display, detect_status, (10, display.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, detect_color, 2)

        cv2.imshow(window_name, display)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            if count >= min_samples:
                break
            else:
                print(f"需要至少 {min_samples} 个样本，当前: {count}")
        elif key == ord('c'):
            if detected:
                calibrator.add_sample(frame)
                print(f"已捕获样本 {len(calibrator.obj_points)}")
            else:
                print("未检测到棋盘格")
        elif key == 27:  # ESC
            cap.release()
            cv2.destroyAllWindows()
            return None

    cv2.destroyWindow(window_name)
    cap.release()

    # 执行标定
    print("\n正在计算相机内参...")
    ret = cap.read()
    if ret:
        image_size = (frame.shape[1], frame.shape[0])
    else:
        image_size = (640, 480)

    camera_matrix, dist_coeffs, rms = calibrator.calibrate(image_size)

    print(f"\n相机内参标定完成!")
    print(f"RMS 误差: {rms:.4f}")
    print(f"\n相机矩阵:")
    print(camera_matrix)
    print(f"\n畸变系数:")
    print(dist_coeffs.flatten())

    return calibrator


# ============================================================
#  主函数
# ============================================================

def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(
        description="SO100 手眼标定工具（完整版）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
使用流程:
  1. 生成棋盘格: python hand_eye_calibration_simple.py --generate-board
  2. 标定相机内参: python hand_eye_calibration_simple.py --calibrate-camera
  3. 手眼标定: python hand_eye_calibration_simple.py --camera-intrinsics camera_calibration.json
        """
    )

    parser.add_argument("--port", type=str, default="COM7", help="串口")
    parser.add_argument("--camera", type=int, default=0, help="摄像头ID")
    parser.add_argument("--generate-board", action="store_true", help="生成棋盘格图像")
    parser.add_argument("--calibrate-camera", action="store_true", help="标定相机内参")
    parser.add_argument("--camera-intrinsics", type=str, help="相机内参文件路径")
    parser.add_argument("--min-poses", type=int, default=15, help="最少姿态数量")
    parser.add_argument("--load", type=str, help="加载已保存的姿态数据")
    parser.add_argument("--pattern", type=int, nargs=2, default=[9, 6],
                        help="棋盘格内角点数量 (cols rows)")
    parser.add_argument("--square-size", type=float, default=0.025,
                        help="方格边长（米）")

    args = parser.parse_args()

    pattern_size = tuple(args.pattern)
    square_size = args.square_size

    # 生成棋盘格
    if args.generate_board:
        board = ChessboardCalibrator(pattern_size, square_size)
        board.generate_board_image()
        return

    # 标定相机内参
    if args.calibrate_camera:
        calibrator = run_camera_calibration(args.camera, pattern_size, square_size)
        if calibrator:
            # 保存结果
            output_path = Path(__file__).parent / "camera_calibration.json"
            calibrator.save(str(output_path))
            print(f"\n相机内参已保存到: {output_path}")
        return

    # 手眼标定
    print("\n" + "=" * 60)
    print("SO100 手眼标定工具")
    print("=" * 60)

    calibrator = HandEyeCalibrator(
        port=args.port,
        camera_id=args.camera,
        pattern_size=pattern_size,
        square_size=square_size
    )

    if not calibrator.connect():
        return

    try:
        # 加载相机内参
        if args.camera_intrinsics:
            calibrator.load_camera_calibration(args.camera_intrinsics)
        else:
            print("\n警告: 未指定相机内参文件，使用默认值")
            print("建议先运行: python hand_eye_calibration_simple.py --calibrate-camera")

        # 加载已有数据
        if args.load:
            calibrator.load_poses(args.load)

        # 交互式标定
        calibrator.interactive_calibration(min_poses=args.min_poses)

        if len(calibrator.poses) < 5:
            print("\n姿态数量不足，无法标定")
            return

        # 计算标定
        X, error = calibrator.calibrate()

        if X is not None:
            calibrator.save_calibration(X, error)

            print("\n" + "=" * 60)
            print("手眼标定完成!")
            print("=" * 60)
            print("\n变换矩阵 (摄像头 -> 末端):")
            print(X)
            print(f"\n平移向量 (米):")
            print(X[:3, 3])
            print(f"\n平移向量 (毫米):")
            print(X[:3, 3] * 1000)
            print(f"\n标定误差: {error:.4f}")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    finally:
        calibrator.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
