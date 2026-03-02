#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂视觉控制器
整合摄像头视觉识别和机械臂控制功能
"""

import sys
import os
import json
import time
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional

# 添加项目根目录到路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, project_root)

# 导入机械臂控制器
from skills.arm_control.main import create_controller, JOINT_NAMES


class VisionController:
    """视觉控制器 - 整合视觉和机械臂控制"""

    def __init__(
        self,
        port: str = "COM7",
        baudrate: int = 1000000,
        camera_id: int = 0,
        calibration_file: Optional[str] = None
    ):
        """
        初始化视觉控制器

        Args:
            port: 串口端口号
            baudrate: 波特率
            camera_id: 摄像头设备ID
            calibration_file: 标定数据文件路径
        """
        # 机械臂控制器
        self.arm = create_controller(port=port)
        self.port = port

        # 摄像头
        self.camera_id = camera_id
        self.cap = None

        # 标定数据
        self.calibration_file = calibration_file or "scripts/vision-control/calibration_data.json"
        self.calibration_matrix = None
        self.load_calibration()

        # 图像尺寸
        self.frame_width = 640
        self.frame_height = 480

    def connect(self):
        """连接机械臂"""
        print(f"[VisionController] 连接到机械臂 ({self.port})...")
        self.arm.connect()

        # 扫描电机
        motors = self.arm.scan_motors()
        print(f"[VisionController] 检测到 {len(motors)} 个电机")

        # 回到初始位置
        print("[VisionController] 回到初始位置...")
        self.arm.go_home()
        time.sleep(2)

        print("[VisionController] 机械臂就绪")

    def disconnect(self):
        """断开连接"""
        if self.cap:
            self.cap.release()
        self.arm.disconnect()
        print("[VisionController] 已断开连接")

    def open_camera(self):
        """打开摄像头"""
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 {self.camera_id}")

            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            print(f"[VisionController] 摄像头已打开 (ID: {self.camera_id})")

    def capture_frame(self):
        """捕获一帧图像"""
        if self.cap is None:
            self.open_camera()

        ret, frame = self.cap.read()

        if not ret:
            raise RuntimeError("无法从摄像头读取图像")

        return frame

    def detect_by_color(
        self,
        color_lower: Tuple[int, int, int],
        color_upper: Tuple[int, int, int],
        min_area: int = 500,
        show_debug: bool = False
    ) -> List[Dict]:
        """
        通过颜色检测物体

        Args:
            color_lower: HSV 颜色下限 (H, S, V)
            color_upper: HSV 颜色上限 (H, S, V)
            min_area: 最小轮廓面积
            show_debug: 是否显示调试窗口

        Returns:
            检测到的物体列表
        """
        # 捕获图像
        frame = self.capture_frame()

        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建颜色掩码
        mask = cv2.inRange(hsv, np.array(color_lower), np.array(color_upper))

        # 形态学操作去噪
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        frame_copy = frame.copy()

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < min_area:
                continue

            # 计算轮廓中心
            M = cv2.moments(contour)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # 绘制轮廓和中心
            cv2.drawContours(frame_copy, [contour], -1, (0, 255, 0), 2)
            cv2.circle(frame_copy, (cx, cy), 5, (0, 0, 255), -1)

            # 计算边界框
            x, y, w, h = cv2.boundingRect(contour)

            objects.append({
                "center_x": cx,
                "center_y": cy,
                "area": area,
                "bounding_box": (x, y, w, h),
                "contour": contour
            })

        if show_debug:
            cv2.imshow("Color Detection", frame_copy)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        return objects

    def detect_by_shape(
        self,
        shape: str,
        min_area: int = 500,
        show_debug: bool = False
    ) -> List[Dict]:
        """
        通过形状检测物体

        Args:
            shape: 形状类型 ("circle", "rectangle", "triangle")
            min_area: 最小轮廓面积
            show_debug: 是否显示调试窗口

        Returns:
            检测到的物体列表
        """
        # 捕获图像
        frame = self.capture_frame()

        # 转换为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 高斯模糊
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Canny 边缘检测
        edges = cv2.Canny(blurred, 50, 150)

        # 查找轮廓
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        objects = []
        frame_copy = frame.copy()

        for contour in contours:
            area = cv2.contourArea(contour)

            if area < min_area:
                continue

            # 多边形近似
            peri = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * peri, True)

            # 形状识别
            detected_shape = None

            if shape == "circle":
                # 圆形检测
                circularity = 4 * np.pi * area / (peri * peri)
                if circularity > 0.8:  # 圆形度阈值
                    detected_shape = "circle"

            elif shape == "rectangle":
                # 矩形检测（4个顶点）
                if len(approx) == 4:
                    detected_shape = "rectangle"

            elif shape == "triangle":
                # 三角形检测（3个顶点）
                if len(approx) == 3:
                    detected_shape = "triangle"

            if detected_shape:
                # 计算轮廓中心
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # 绘制
                    cv2.drawContours(frame_copy, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(frame_copy, (cx, cy), 5, (0, 0, 255), -1)

                    objects.append({
                        "center_x": cx,
                        "center_y": cy,
                        "area": area,
                        "shape": detected_shape,
                        "contour": contour
                    })

        if show_debug:
            cv2.imshow("Shape Detection", frame_copy)
            cv2.waitKey(1)

        return objects

    def get_largest_object(self, objects: List[Dict]) -> Optional[Dict]:
        """获取最大的物体"""
        if not objects:
            return None

        return max(objects, key=lambda o: o["area"])

    def pixel_to_arm_coord(
        self,
        pixel_x: int,
        pixel_y: int,
        grab_height: float = 50.0
    ) -> Tuple[float, float, float]:
        """
        将像素坐标转换为机械臂坐标

        Args:
            pixel_x: 像素 X 坐标
            pixel_y: 像素 Y 坐标
            grab_height: 抓取高度（mm）

        Returns:
            (x, y, z) 机械臂坐标
        """
        if self.calibration_matrix is None:
            raise RuntimeError("未加载标定数据，请先进行手眼标定")

        # 像素坐标归一化（相对于图像中心）
        norm_x = (pixel_x - self.frame_width / 2) / (self.frame_width / 2)
        norm_y = (pixel_y - self.frame_height / 2) / (self.frame_height / 2)

        # 使用标定矩阵转换
        # 这里使用简化的线性变换，实际应用中可能需要更复杂的模型
        scale_x = self.calibration_matrix.get("scale_x", 100)
        scale_y = self.calibration_matrix.get("scale_y", 100)
        offset_x = self.calibration_matrix.get("offset_x", 200)
        offset_y = self.calibration_matrix.get("offset_y", 0)

        arm_x = offset_x + norm_x * scale_x
        arm_y = offset_y + norm_y * scale_y
        arm_z = grab_height

        return arm_x, arm_y, arm_z

    def arm_coord_to_joints(
        self,
        x: float,
        y: float,
        z: float
    ) -> Dict[str, int]:
        """
        将机械臂坐标转换为关节位置（简化版）

        Args:
            x, y, z: 机械臂坐标（mm）

        Returns:
            关节位置字典
        """
        # 这是一个简化的逆运动学
        # 实际应用中应该使用完整的逆运动学求解器

        # 简化的映射关系（仅用于演示）
        # shoulder_pan: 底部旋转，对应 Y 坐标
        # shoulder_lift: 大臂升降，对应 Z 坐标
        # elbow_flex: 小臂弯曲，对应 X 坐标
        # wrist_flex: 手腕俯仰，保持水平
        # wrist_roll: 手腕旋转，保持不变
        # gripper: 夹爪，独立控制

        joints = {
            "shoulder_pan": int(2047 + y * 2),  # Y 轴映射
            "shoulder_lift": int(2047 + z * 2),  # Z 轴映射
            "elbow_flex": int(2047 - x * 2),  # X 轴映射
            "wrist_flex": 2047,  # 保持水平
            "wrist_roll": 2047,  # 保持不变
            "gripper": 2047  # 初始状态
        }

        # 限制范围
        for joint in joints:
            joints[joint] = max(0, min(4095, joints[joint]))

        return joints

    def grasp_object_at(
        self,
        pixel_x: int,
        pixel_y: int,
        grab_height: float = 50.0,
        approach_height: float = 150.0
    ):
        """
        在指定像素坐标位置抓取物体

        Args:
            pixel_x: 像素 X 坐标
            pixel_y: 像素 Y 坐标
            grab_height: 抓取高度（mm）
            approach_height: 接近高度（mm）
        """
        print(f"[VisionController] 开始抓取物体 (像素: {pixel_x}, {pixel_y})")

        # 转换坐标
        arm_x, arm_y, arm_z = self.pixel_to_arm_coord(pixel_x, pixel_y, grab_height)
        print(f"[VisionController] 机械臂坐标: ({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})")

        # 计算关节位置
        joints = self.arm_coord_to_joints(arm_x, arm_y, arm_z)

        # 1. 打开夹爪
        print("[VisionController] 打开夹爪...")
        self.arm.open_gripper(3000)
        time.sleep(1)

        # 2. 移动到接近位置（上方）
        print("[VisionController] 移动到接近位置...")
        approach_joints = self.arm_coord_to_joints(arm_x, arm_y, approach_height)
        self.arm.move_smooth(approach_joints)
        time.sleep(2)

        # 3. 下降到抓取高度
        print("[VisionController] 下降到抓取高度...")
        self.arm.move_smooth(joints)
        time.sleep(1)

        # 4. 闭合夹爪
        print("[VisionController] 闭合夹爪...")
        self.arm.close_gripper(1000)
        time.sleep(1)

        # 5. 抬起
        print("[VisionController] 抬起物体...")
        self.arm.move_smooth(approach_joints)
        time.sleep(1)

        print("[VisionController] 抓取完成")

    def place_object_at(
        self,
        pixel_x: int,
        pixel_y: int,
        place_height: float = 50.0
    ):
        """
        在指定位置放置物体

        Args:
            pixel_x: 像素 X 坐标
            pixel_y: 像素 Y 坐标
            place_height: 放置高度（mm）
        """
        print(f"[VisionController] 开始放置物体 (像素: {pixel_x}, {pixel_y})")

        # 转换坐标
        arm_x, arm_y, arm_z = self.pixel_to_arm_coord(pixel_x, pixel_y, place_height)

        # 计算关节位置
        joints = self.arm_coord_to_joints(arm_x, arm_y, arm_z)

        # 1. 移动到放置位置上方
        approach_joints = self.arm_coord_to_joints(arm_x, arm_y, place_height + 50)
        self.arm.move_smooth(approach_joints)
        time.sleep(1)

        # 2. 下降
        self.arm.move_smooth(joints)
        time.sleep(0.5)

        # 3. 打开夹爪
        print("[VisionController] 打开夹爪...")
        self.arm.open_gripper(3000)
        time.sleep(1)

        # 4. 抬起
        print("[VisionController] 抬起...")
        self.arm.move_smooth(approach_joints)
        time.sleep(1)

        print("[VisionController] 放置完成")

    def calibrate_hand_eye(
        self,
        calibration_points: List[Dict[str, Tuple]]
    ) -> np.ndarray:
        """
        手眼标定

        Args:
            calibration_points: 标定点列表
                [{"pixel": (x, y), "arm": (x, y, z)}, ...]

        Returns:
            标定矩阵
        """
        print("[VisionController] 开始手眼标定...")

        # 构建方程组：Ax = b
        A = []
        b_x = []
        b_y = []

        for point in calibration_points:
            pixel_x, pixel_y = point["pixel"]
            arm_x, arm_y, _ = point["arm"]

            # 归一化像素坐标
            norm_x = (pixel_x - self.frame_width / 2) / (self.frame_width / 2)
            norm_y = (pixel_y - self.frame_height / 2) / (self.frame_height / 2)

            # [norm_x, norm_y, 1] * [scale_x, offset_x] = arm_x
            A.append([norm_x, 1])
            b_x.append(arm_x)

            A.append([norm_y, 1])
            b_y.append(arm_y)

        A = np.array(A)
        b_x = np.array(b_x)
        b_y = np.array(b_y)

        # 最小二乘求解
        x_params, _, _, _ = np.linalg.lstsq(A, b_x, rcond=None)
        y_params, _, _, _ = np.linalg.lstsq(A, b_y, rcond=None)

        # 构建标定矩阵
        calibration_matrix = {
            "scale_x": float(x_params[0]),
            "offset_x": float(x_params[1]),
            "scale_y": float(y_params[0]),
            "offset_y": float(y_params[1]),
            "frame_width": self.frame_width,
            "frame_height": self.frame_height
        }

        self.calibration_matrix = calibration_matrix

        print(f"[VisionController] 标定完成:")
        print(f"  scale_x: {calibration_matrix['scale_x']:.2f}")
        print(f"  offset_x: {calibration_matrix['offset_x']:.2f}")
        print(f"  scale_y: {calibration_matrix['scale_y']:.2f}")
        print(f"  offset_y: {calibration_matrix['offset_y']:.2f}")

        return calibration_matrix

    def save_calibration(self, filename: str = None):
        """保存标定数据"""
        if filename is None:
            filename = self.calibration_file

        if self.calibration_matrix is None:
            print("[VisionController] 没有标定数据可保存")
            return

        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, 'w') as f:
            json.dump(self.calibration_matrix, f, indent=2)

        print(f"[VisionController] 标定数据已保存到 {filename}")

    def load_calibration(self, filename: str = None):
        """加载标定数据"""
        if filename is None:
            filename = self.calibration_file

        if os.path.exists(filename):
            with open(filename, 'r') as f:
                self.calibration_matrix = json.load(f)
            print(f"[VisionController] 已加载标定数据: {filename}")
        else:
            print(f"[VisionController] 标定文件不存在: {filename}")


# 使用示例
if __name__ == "__main__":
    # 创建控制器
    vc = VisionController(port="COM7", camera_id=0)

    try:
        # 连接
        vc.connect()
        vc.open_camera()

        # 检测红色物体
        print("\n检测红色物体...")
        red_objects = vc.detect_by_color(
            color_lower=(0, 120, 70),
            color_upper=(10, 255, 255),
            show_debug=True
        )

        if red_objects:
            print(f"找到 {len(red_objects)} 个红色物体")

            # 抓取最大的
            largest = vc.get_largest_object(red_objects)
            print(f"抓取最大物体 (面积: {largest['area']})")

            vc.grasp_object_at(largest["center_x"], largest["center_y"])

            # 放置到其他位置
            vc.place_object_at(500, 200)
        else:
            print("未找到红色物体")

    finally:
        vc.disconnect()
