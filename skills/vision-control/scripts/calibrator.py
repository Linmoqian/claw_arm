#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂手眼标定工具
支持Eye-in-Hand配置(摄像头固定在机械臂上)的标定
"""

import sys
import os
import json
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass, asdict

# 添加项目根目录到路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, project_root)

from skills.arm_control.main import create_controller


@dataclass
class CalibrationPoint:
    """标定点数据"""
    pixel_coords: List[Tuple[int, int]]  # 图像中的角点坐标
    arm_position: Dict[str, int]  # 机械臂关节位置
    pose_id: int  # 位姿ID


class HandEyeCalibrator:
    """手眼标定器 - Eye-in-Hand配置"""

    def __init__(
        self,
        camera_id: int = 0,
        pattern_type: str = "chessboard",
        pattern_size: Tuple[int, int] = (9, 6),
        square_size: float = 30.0,
        output_file: Optional[str] = None
    ):
        """
        初始化标定器

        Args:
            camera_id: 摄像头设备ID
            pattern_type: 标定板类型 ("chessboard", "circles")
            pattern_size: 标定板尺寸 (cols, rows)
            square_size: 方格/圆点间距(mm)
            output_file: 输出文件路径
        """
        self.camera_id = camera_id
        self.pattern_type = pattern_type
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.output_file = output_file or "skills/vision-control/calibration_data.json"

        # 摄像头
        self.cap = None
        self.frame_width = 640
        self.frame_height = 480

        # 标定数据
        self.calibration_points: List[CalibrationPoint] = []
        self.obj_points = []  # 3D世界坐标点
        self.img_points = []  # 2D图像坐标点

        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.reprojection_error = None

        # 机械臂控制器
        self.arm = None

    def open_camera(self):
        """打开摄像头"""
        if self.cap is None:
            self.cap = cv2.VideoCapture(self.camera_id)

            if not self.cap.isOpened():
                raise RuntimeError(f"无法打开摄像头 {self.camera_id}")

            # 设置分辨率
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)

            self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            print(f"[Calibrator] 摄像头已打开 (分辨率: {self.frame_width}x{self.frame_height})")

    def detect_pattern(self, frame: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        检测标定板

        Args:
            frame: 输入图像

        Returns:
            (corners, gray) - 角点坐标和灰度图,未检测到返回None
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.pattern_type == "chessboard":
            # 棋盘格检测
            ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

            if ret:
                # 亚像素精度优化
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                return corners, gray

        elif self.pattern_type == "circles":
            # 圆点阵列检测
            ret, corners = cv2.findCirclesGrid(gray, self.pattern_size, None)

            if ret:
                return corners, gray

        return None

    def capture_calibration_point(
        self,
        arm_position: Dict[str, int],
        show_preview: bool = True
    ) -> bool:
        """
        采集一个标定点

        Args:
            arm_position: 机械臂当前关节位置
            show_preview: 是否显示预览窗口

        Returns:
            是否成功采集
        """
        if self.cap is None:
            self.open_camera()

        ret, frame = self.cap.read()
        if not ret:
            print("[Calibrator] 无法读取图像")
            return False

        # 检测标定板
        result = self.detect_pattern(frame)

        if result is None:
            print("[Calibrator] 未检测到标定板,请调整位置")
            if show_preview:
                cv2.imshow("Calibration", frame)
                cv2.waitKey(1000)
            return False

        corners, gray = result

        # 可视化
        if show_preview:
            display_frame = frame.copy()
            cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, True)
            cv2.putText(display_frame, "Pattern Detected!", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Calibration", display_frame)
            cv2.waitKey(500)

        # 保存标定点
        calib_point = CalibrationPoint(
            pixel_coords=[(int(c[0][0]), int(c[0][1])) for c in corners],
            arm_position=arm_position,
            pose_id=len(self.calibration_points)
        )
        self.calibration_points.append(calib_point)

        # 准备相机标定数据
        # 创建3D世界坐标点
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        self.obj_points.append(objp)
        self.img_points.append(corners)

        print(f"[Calibrator] 已采集标定点 #{len(self.calibration_points)}")
        return True

    def interactive_calibration(
        self,
        arm_port: str = "COM7",
        num_poses: int = 15,
        preview_window: bool = True
    ) -> Dict:
        """
        交互式标定流程

        Args:
            arm_port: 机械臂串口端口
            num_poses: 需要采集的姿态数量
            preview_window: 是否显示预览窗口

        Returns:
            标定数据字典
        """
        print("=" * 60)
        print("SO100 手眼标定工具 - Eye-in-Hand")
        print("=" * 60)
        print(f"标定板类型: {self.pattern_type}")
        print(f"标定板尺寸: {self.pattern_size[0]}x{self.pattern_size[1]}")
        print(f"方格大小: {self.square_size}mm")
        print(f"目标姿态数量: {num_poses}")
        print("=" * 60)

        # 连接机械臂
        print("\n[步骤1] 连接机械臂...")
        self.arm = create_controller(port=arm_port)
        self.arm.connect()
        print("机械臂已连接")

        # 打开摄像头
        print("\n[步骤2] 打开摄像头...")
        self.open_camera()

        # 回到初始位置
        print("\n[步骤3] 机械臂回到初始位置...")
        self.arm.go_home()
        import time
        time.sleep(2)

        # 采集标定点
        print(f"\n[步骤4] 采集标定点 (共需要{num_poses}个)")
        print("操作说明:")
        print("  1. 将标定板放置在机械臂工作空间内的不同位置")
        print("  2. 每次放置后,按空格键采集")
        print("  3. 按'q'键提前结束")
        print("  4. 按'r'键机械臂移动到随机位置(建议)")
        print("-" * 60)

        captured_count = 0

        while captured_count < num_poses:
            # 显示预览
            ret, frame = self.cap.read()
            if not ret:
                continue

            display_frame = frame.copy()

            # 尝试检测标定板
            result = self.detect_pattern(frame)
            if result:
                corners, _ = result
                cv2.drawChessboardCorners(display_frame, self.pattern_size, corners, True)
                status = "Pattern Detected! Press SPACE to capture"
                color = (0, 255, 0)
            else:
                status = "No Pattern detected"
                color = (0, 0, 255)

            # 显示状态
            cv2.putText(display_frame, f"Captured: {captured_count}/{num_poses}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.putText(display_frame, status, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(display_frame, "SPACE: Capture, R: Random move, Q: Quit", (10, self.frame_height - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)

            cv2.imshow("Calibration", display_frame)

            # 读取按键
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                print("\n用户中断标定")
                break

            elif key == ord('r'):
                # 移动到随机位置
                print("移动机械臂到随机位置...")
                self._move_to_random_pose()
                time.sleep(1.5)

            elif key == ord(' '):
                # 采集标定点
                if result:
                    # 获取当前机械臂位置
                    state = self.arm.get_state()
                    arm_pos = {
                        "shoulder_pan": state.joints[0].position,
                        "shoulder_lift": state.joints[1].position,
                        "elbow_flex": state.joints[2].position,
                        "wrist_flex": state.joints[3].position,
                        "wrist_roll": state.joints[4].position,
                        "gripper": state.joints[5].position
                    }

                    if self.capture_calibration_point(arm_pos, show_preview=preview_window):
                        captured_count += 1

                        if captured_count < num_poses:
                            print(f"建议: 移动机械臂到不同位置 (按'r'随机移动)")
                    else:
                        print("采集失败,请重试")
                else:
                    print("未检测到标定板,无法采集")

        # 关闭预览窗口
        cv2.destroyAllWindows()

        if captured_count < 4:
            print(f"\n错误: 采集点数量不足({captured_count} < 4),无法进行标定")
            return None

        # 执行标定
        print("\n[步骤5] 执行相机内参标定...")
        self._calibrate_camera_intrinsics()

        print("\n[步骤6] 计算手眼标定矩阵...")
        self._calculate_hand_eye_matrix()

        # 保存标定结果
        print("\n[步骤7] 保存标定数据...")
        calibration_data = self._get_calibration_data()
        self._save_calibration_data(calibration_data)

        print("\n" + "=" * 60)
        print("标定完成!")
        print(f"重投影误差: {self.reprojection_error:.3f} 像素")
        print(f"标定数据已保存到: {self.output_file}")
        print("=" * 60)

        return calibration_data

    def _move_to_random_pose(self):
        """移动机械臂到随机位置"""
        import random

        # 生成随机关节位置(在安全范围内)
        random_pos = {
            "shoulder_pan": random.randint(1500, 2500),
            "shoulder_lift": random.randint(1500, 2500),
            "elbow_flex": random.randint(1500, 2500),
            "wrist_flex": random.randint(1800, 2300),
            "wrist_roll": 2047,
            "gripper": 2047
        }

        self.arm.move_smooth(random_pos)

    def _calibrate_camera_intrinsics(self):
        """标定相机内参"""
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            (self.frame_width, self.frame_height),
            None, None
        )

        if not ret:
            raise RuntimeError("相机内参标定失败")

        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.rvecs = rvecs
        self.tvecs = tvecs

        # 计算重投影误差
        total_error = 0
        for i in range(len(self.obj_points)):
            img_points_reprojected, _ = cv2.projectPoints(
                self.obj_points[i],
                self.rvecs[i],
                self.tvecs[i],
                self.camera_matrix,
                self.dist_coeffs
            )
            error = cv2.norm(self.img_points[i], img_points_reprojected, cv2.NORM_L2) / len(img_points_reprojected)
            total_error += error

        self.reprojection_error = total_error / len(self.obj_points)

        print(f"相机内参矩阵:\n{self.camera_matrix}")
        print(f"畸变系数: {self.dist_coeffs.ravel()}")

    def _calculate_hand_eye_matrix(self):
        """
        计算手眼标定矩阵
        对于Eye-in-Hand配置,求解: AX = ZB
        """
        if len(self.calibration_points) < 4:
            print("警告: 标定点数量不足,手眼矩阵可能不准确")

        # 这里使用简化的方法
        # 完整的手眼标定需要使用cv2.calibrateHandEye()

        # 收集数据
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        # 简化: 假设标定板在世界坐标系中固定
        # 实际应用中需要更复杂的处理

        # 计算平均变换(简化版本)
        # 对于完整版本,应该使用 cv2.calibrateHandEye()

        # 这里提供一个简化的平移估计
        avg_x = avg_y = avg_z = 0
        count = 0

        for calib_point in self.calibration_points:
            # 使用标定板中心点
            center_idx = len(calib_point.pixel_coords) // 2
            pixel_x, pixel_y = calib_point.pixel_coords[center_idx]

            # 简化的深度估计(假设标定板距离相机500mm)
            depth = 500.0

            # 像素坐标转相机坐标(简化)
            cam_x = (pixel_x - self.frame_width / 2) * depth / self.camera_matrix[0, 0]
            cam_y = (pixel_y - self.frame_height / 2) * depth / self.camera_matrix[1, 1]
            cam_z = depth

            avg_x += cam_x
            avg_y += cam_y
            avg_z += cam_z
            count += 1

        avg_x /= count
        avg_y /= count
        avg_z /= count

        # 构建简化的手眼矩阵(仅平移)
        self.hand_eye_matrix = np.eye(4)
        self.hand_eye_matrix[0, 3] = avg_x
        self.hand_eye_matrix[1, 3] = avg_y
        self.hand_eye_matrix[2, 3] = avg_z

        print(f"手眼标定矩阵(简化版):\n{self.hand_eye_matrix}")

    def _get_calibration_data(self) -> Dict:
        """获取标定数据"""
        return {
            "camera_id": self.camera_id,
            "pattern_type": self.pattern_type,
            "pattern_size": self.pattern_size,
            "square_size": self.square_size,
            "frame_width": self.frame_width,
            "frame_height": self.frame_height,
            "camera_matrix": self.camera_matrix.tolist(),
            "dist_coeffs": self.dist_coeffs.tolist(),
            "hand_eye_matrix": self.hand_eye_matrix.tolist(),
            "reprojection_error": float(self.reprojection_error),
            "num_poses": len(self.calibration_points),

            # 简化的转换参数(用于快速转换)
            "scale_x": 100.0,
            "scale_y": 100.0,
            "offset_x": 200.0,
            "offset_y": 0.0,
            "focal_length": float(self.camera_matrix[0, 0]) if self.camera_matrix is not None else 500.0
        }

    def _save_calibration_data(self, data: Dict):
        """保存标定数据到文件"""
        os.makedirs(os.path.dirname(self.output_file), exist_ok=True)

        with open(self.output_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2)

        print(f"标定数据已保存")

    def load_calibration(self, filename: Optional[str] = None) -> bool:
        """加载标定数据"""
        if filename is None:
            filename = self.output_file

        if not os.path.exists(filename):
            print(f"标定文件不存在: {filename}")
            return False

        with open(filename, 'r', encoding='utf-8') as f:
            data = json.load(f)

        self.camera_matrix = np.array(data["camera_matrix"])
        self.dist_coeffs = np.array(data["dist_coeffs"])
        self.hand_eye_matrix = np.array(data["hand_eye_matrix"])
        self.reprojection_error = data.get("reprojection_error", 0.0)

        print(f"已加载标定数据: {filename}")
        print(f"重投影误差: {self.reprojection_error:.3f} 像素")

        return True

    def save_calibration(self, filename: Optional[str] = None):
        """保存标定数据(别名)"""
        if filename is not None:
            self.output_file = filename

        data = self._get_calibration_data()
        self._save_calibration_data(data)

    def release(self):
        """释放资源"""
        if self.cap:
            self.cap.release()
        if self.arm:
            self.arm.disconnect()
        cv2.destroyAllWindows()


# 命令行使用
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="SO100 手眼标定工具")
    parser.add_argument("--camera", type=int, default=0, help="摄像头ID")
    parser.add_argument("--port", type=str, default="COM7", help="机械臂串口端口")
    parser.add_argument("--pattern", type=str, default="chessboard", choices=["chessboard", "circles"],
                       help="标定板类型")
    parser.add_argument("--size", type=int, nargs=2, default=[9, 6], help="标定板尺寸 (列数 行数)")
    parser.add_argument("--square", type=float, default=30.0, help="方格大小(mm)")
    parser.add_argument("--poses", type=int, default=15, help="采集姿态数量")
    parser.add_argument("--output", type=str, default="skills/vision-control/calibration_data.json",
                       help="输出文件路径")

    args = parser.parse_args()

    # 创建标定器
    calibrator = HandEyeCalibrator(
        camera_id=args.camera,
        pattern_type=args.pattern,
        pattern_size=tuple(args.size),
        square_size=args.square,
        output_file=args.output
    )

    try:
        # 执行交互式标定
        calibration_data = calibrator.interactive_calibration(
            arm_port=args.port,
            num_poses=args.poses,
            preview_window=True
        )

        if calibration_data:
            print("\n标定成功完成!")
        else:
            print("\n标定失败")

    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        calibrator.release()
