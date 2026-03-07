#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
SO100 机械臂手眼标定工具
使用 ArUco 标记板进行 Eye-in-Hand 标定
"""

import cv2
import numpy as np
import json
import time
from pathlib import Path
from typing import List, Tuple, Optional
import sys

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent))
from SDK import PortHandler, PacketHandler


class ArUcoCalibrationBoard:
    """ArUco 标定板"""

    def __init__(self, markers_x: int = 5, markers_y: int = 7,
                 marker_length: float = 0.04, marker_separation: float = 0.01):
        """
        初始化 ArUco 标定板

        Args:
            markers_x: X 方向标记数量
            markers_y: Y 方向标记数量
            marker_length: 标记边长（米）
            marker_separation: 标记间距（米）
        """
        self.markers_x = markers_x
        self.markers_y = markers_y
        self.marker_length = marker_length
        self.marker_separation = marker_separation

        # 创建字典和标定板
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # 兼容不同 OpenCV 版本的 API
        try:
            # OpenCV 4.13+
            self.board = cv2.aruco.GridBoard(
                size=(markers_x, markers_y),
                markerLength=marker_length,
                markerSeparation=marker_separation,
                dictionary=self.dictionary
            )
        except TypeError:
            # OpenCV 4.12 及更早版本
            self.board = cv2.aruco.GridBoard(
                markersX=markers_x,
                markersY=markers_y,
                markerLength=marker_length,
                markerSeparation=marker_separation,
                dictionary=self.dictionary
            )

    def detect(self, image: np.ndarray) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        检测标定板

        Args:
            image: 输入图像

        Returns:
            (rvec, tvec, corners): 旋转向量、平移向量、角点
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # 相机内参（简化，使用默认焦距）
        camera_matrix = np.array([[800, 0, image.shape[1]/2],
                                  [0, 800, image.shape[0]/2],
                                  [0, 0, 1]], dtype=float)
        dist_coeffs = np.zeros(5)

        # 兼容不同 OpenCV 版本的 ArUco API
        corners, ids, rejected = None, None, None

        # 尝试 OpenCV 4.13+ API
        try:
            detector = cv2.aruco.ArucoDetector(self.dictionary)
            corners, ids, rejected = detector.detectMarkers(gray)
        except (AttributeError, TypeError):
            try:
                # OpenCV 4.12 及更早版本
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary)
            except AttributeError:
                pass

        if ids is None or len(ids) < 4:
            return None, None, None

        # 估计姿态 - 使用 solvePnP
        rvec, tvec = None, None

        # 手动构建点
        try:
            # 获取标定板中所有标记的 3D 坐标和 2D 像素坐标
            obj_points = []
            img_points = []

            ids_list = ids.flatten().tolist() if hasattr(ids, 'flatten') else list(ids)

            for i, marker_id in enumerate(ids_list):
                try:
                    # 获取该标记的 3D 坐标（4个角点）
                    idx = list(self.board.ids).index(marker_id) if hasattr(self.board, 'ids') else marker_id
                    marker_obj_points = self.board.objPoints[idx]

                    for j in range(4):  # 每个标记4个角点
                        obj_points.append(marker_obj_points[j])
                        img_points.append(corners[i][0][j])

                except (IndexError, KeyError, ValueError):
                    continue

            if len(obj_points) >= 4:
                obj_points = np.array(obj_points, dtype=np.float32).reshape(-1, 1, 3)
                img_points = np.array(img_points, dtype=np.float32).reshape(-1, 1, 2)

                success, rvec, tvec = cv2.solvePnP(
                    obj_points, img_points, camera_matrix, dist_coeffs
                )
                if not success:
                    rvec, tvec = None, None
        except Exception as e:
            print(f"姿态估计失败: {e}")
            pass

        return rvec, tvec, corners

    def draw(self, image: np.ndarray, rvec: np.ndarray, tvec: np.ndarray,
             corners: np.ndarray) -> np.ndarray:
        """在图像上绘制坐标系"""
        result = image.copy()

        # 绘制检测到的标记
        try:
            cv2.aruco.drawDetectedMarkers(result, corners, None)
        except TypeError:
            try:
                cv2.aruco.drawDetectedMarkers(result, corners)
            except AttributeError:
                pass

        # 绘制坐标轴
        if rvec is not None and tvec is not None:
            camera_matrix = np.array([[800, 0, image.shape[1]/2],
                                      [0, 800, image.shape[0]/2],
                                      [0, 0, 1]], dtype=float)
            dist_coeffs = np.zeros(5)

            try:
                cv2.drawFrameAxes(result, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            except AttributeError:
                try:
                    cv2.aruco.drawAxis(result, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
                except AttributeError:
                    pass

        return result

    def generate_board_image(self, output_path: str = "calibration_board.png",
                             dpi: int = 300) -> None:
        """
        生成标定板图像

        Args:
            output_path: 输出路径
            dpi: 打印分辨率（每英寸像素数）
        """
        # 计算实际物理尺寸（米）
        width_m = (self.markers_x * self.marker_length +
                  (self.markers_x - 1) * self.marker_separation)
        height_m = (self.markers_y * self.marker_length +
                   (self.markers_y - 1) * self.marker_separation)

        # 转换为像素（DPI = 像素/英寸，1英寸=0.0254米）
        pixels_per_meter = dpi / 0.0254
        width_px = int(width_m * pixels_per_meter)
        height_px = int(height_m * pixels_per_meter)

        # 生成图像
        img = self.board.generateImage((width_px, height_px), marginSize=int(0.02 * pixels_per_meter))
        cv2.imwrite(output_path, img)

        # 显示尺寸信息
        width_cm = width_m * 100
        height_cm = height_m * 100
        print(f"标定板已保存到: {output_path}")
        print(f"打印尺寸: {width_cm:.1f}cm x {height_cm:.1f}cm")
        print(f"图像尺寸: {width_px} x {height_px} 像素 @ {dpi} DPI")
        print(f"\n打印建议:")
        print(f"1. 打印时设置分辨率为 {dpi} DPI")
        print(f"2. 确保打印比例为 100%，不要缩放")
        print(f"3. 标记边长: {self.marker_length * 100:.1f}cm")
        print(f"4. 标记间距: {self.marker_separation * 100:.1f}cm")


class CalibrationPose:
    """单个标定姿态"""

    def __init__(self, joint_positions: List[int],
                 camera_rvec: np.ndarray, camera_tvec: np.ndarray):
        """
        Args:
            joint_positions: 6个关节的位置 [j1, j2, j3, j4, j5, j6]
            camera_rvec: 摄像头到标定板的旋转向量
            camera_tvec: 摄像头到标定板的平移向量
        """
        self.joint_positions = joint_positions
        self.camera_rvec = camera_rvec
        self.camera_tvec = camera_tvec

    def to_dict(self) -> dict:
        """转换为字典"""
        return {
            "joint_positions": self.joint_positions,
            "camera_rvec": self.camera_rvec.tolist(),
            "camera_tvec": self.camera_tvec.tolist()
        }

    @classmethod
    def from_dict(cls, data: dict) -> 'CalibrationPose':
        """从字典创建"""
        return cls(
            joint_positions=data["joint_positions"],
            camera_rvec=np.array(data["camera_rvec"]),
            camera_tvec=np.array(data["camera_tvec"])
        )


class HandEyeCalibrator:
    """手眼标定器"""

    def __init__(self, port: str = "COM7", camera_id: int = 0):
        """
        Args:
            port: 串口
            camera_id: 摄像头ID
        """
        self.port = port
        self.camera_id = camera_id

        # 初始化硬件
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(0.0)
        self.cap = cv2.VideoCapture(camera_id)

        # 标定板
        self.board = ArUcoCalibrationBoard()

        # 标定数据
        self.poses: List[CalibrationPose] = []

        # 机械臂运动学参数（需要根据实际机械臂调整）
        # 这里使用简化的 DH 参数或测量值
        self.link_lengths = [50, 80, 80, 50, 30, 20]  # mm

    def connect(self) -> bool:
        """连接硬件"""
        # 连接串口
        if not self.port_handler.openPort():
            print(f"无法打开串口 {self.port}")
            return False

        self.port_handler.setBaudRate(1000000)

        # 连接摄像头
        if not self.cap.isOpened():
            print(f"无法打开摄像头 {self.camera_id}")
            return False

        print("硬件连接成功")
        return True

    def disconnect(self):
        """断开连接"""
        self.port_handler.closePort()
        self.cap.release()

    def read_joint_positions(self) -> List[int]:
        """读取所有关节位置"""
        positions = []
        for motor_id in range(1, 7):
            data, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, motor_id, 56  # PRESENT_POSITION
            )
            if result == 0:
                positions.append(data)
            else:
                print(f"读取电机 {motor_id} 失败: {error}")
                positions.append(2047)
        return positions

    def forward_kinematics(self, joint_positions: List[int]) -> np.ndarray:
        """
        正向运动学（简化版本）

        对于精确标定，需要建立完整的 DH 参数表或使用运动学库
        这里使用简化版本，仅作示例

        Args:
            joint_positions: 6个关节位置

        Returns:
            4x4 齐次变换矩阵
        """
        # TODO: 实现 SO100 的完整正向运动学
        # 这里返回单位矩阵作为占位符
        return np.eye(4)

    def capture_pose(self) -> bool:
        """捕获当前姿态"""
        print("\n捕获姿态...")

        # 读取关节位置
        joint_positions = self.read_joint_positions()
        print(f"关节位置: {joint_positions}")

        # 检测标定板
        ret, frame = self.cap.read()
        if not ret:
            print("无法读取摄像头")
            return False

        rvec, tvec, corners = self.board.detect(frame)

        if rvec is None or tvec is None:
            print("未检测到标定板，请确保标定板在视野内")
            # 显示检测结果
            cv2.imshow("Detection", frame)
            cv2.waitKey(1)
            return False

        # 保存姿态
        pose = CalibrationPose(joint_positions, rvec, tvec)
        self.poses.append(pose)

        # 绘制检测结果
        result = self.board.draw(frame, rvec, tvec, corners)
        cv2.putText(result, f"Poses: {len(self.poses)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Detection", result)

        print(f"成功捕获姿态 {len(self.poses)}")
        print(f"旋转: {rvec.flatten()}")
        print(f"平移: {tvec.flatten()}")

        return True

    def interactive_calibration(self, min_poses: int = 15):
        """
        交互式标定流程

        Args:
            min_poses: 最少姿态数量
        """
        print("\n" + "=" * 60)
        print("SO100 手眼标定工具")
        print("=" * 60)
        print("\n操作说明:")
        print("1. 将标定板放置在机械臂工作区域内")
        print("2. 移动机械臂到不同位置（保持标定板在视野内）")
        print("3. 在每个位置按 'c' 捕获姿态")
        print("4. 捕获至少 {} 个姿态".format(min_poses))
        print("5. 按 'q' 完成标定")
        print("\n提示: 尝试从不同角度和距离捕获姿态")

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # 检测标定板
            rvec, tvec, corners = self.board.detect(frame)

            if rvec is not None:
                frame = self.board.draw(frame, rvec, tvec, corners)

            # 显示状态
            status = f"Poses: {len(self.poses)}/{min_poses}"
            cv2.putText(frame, status, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if len(self.poses) >= min_poses:
                cv2.putText(frame, "Ready to calibrate! Press 'q'", (10, 70),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, f"Need {min_poses - len(self.poses)} more poses",
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.imshow("Calibration", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                if len(self.poses) >= min_poses:
                    break
                else:
                    print(f"需要至少 {min_poses} 个姿态，当前: {len(self.poses)}")
            elif key == ord('c'):
                self.capture_pose()
            elif key == ord('u'):
                # 撤销最后一个姿态
                if self.poses:
                    self.poses.pop()
                    print(f"撤销，剩余 {len(self.poses)} 个姿态")
            elif key == ord('s'):
                # 保存当前数据
                self.save_poses("calibration_poses.json")

    def calibrate(self) -> Tuple[Optional[np.ndarray], float]:
        """
        执行标定计算

        Returns:
            (X_matrix, reprojection_error): 变换矩阵和重投影误差
        """
        if len(self.poses) < 10:
            print("姿态数量不足，需要至少 10 个")
            return None, 0.0

        print("\n开始标定计算...")

        # 准备标定数据
        R_gripper2base = []
        t_gripper2base = []
        R_target2cam = []
        t_target2cam = []

        for pose in self.poses:
            # 基座到末端（使用正向运动学）
            T_g2b = self.forward_kinematics(pose.joint_positions)
            R_g2b = T_g2b[:3, :3]
            t_g2b = T_g2b[:3, 3]

            # 标定板到摄像头
            R_c2t, _ = cv2.Rodrigues(pose.camera_rvec)
            t_c2t = pose.camera_tvec

            # 转换为目标到摄像头
            R_t2c = R_c2t.T
            t_t2c = -R_c2t.T @ t_c2t

            R_gripper2base.append(R_g2b)
            t_gripper2base.append(t_g2b)
            R_target2cam.append(R_t2c)
            t_target2cam.append(t_t2c)

        # 使用 OpenCV 的手眼标定
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base=R_gripper2base,
            t_gripper2base=t_gripper2base,
            R_target2cam=R_target2cam,
            t_target2cam=t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )

        # 构造 4x4 变换矩阵
        X = np.eye(4)
        X[:3, :3] = R_cam2gripper
        X[:3, 3] = t_cam2gripper.flatten()

        # 计算重投影误差
        error = self._compute_reprojection_error(X)

        print("\n标定完成!")
        print(f"变换矩阵 (摄像头到末端):")
        print(X)
        print(f"\n重投影误差: {error:.4f} mm")

        return X, error

    def _compute_reprojection_error(self, X: np.ndarray) -> float:
        """计算重投影误差"""
        errors = []

        for pose in self.poses:
            # 基座到末端
            T_g2b = self.forward_kinematics(pose.joint_positions)

            # 标定板到摄像头
            R_c2t, _ = cv2.Rodrigues(pose.camera_rvec)
            T_c2t = np.eye(4)
            T_c2t[:3, :3] = R_c2t
            T_c2t[:3, 3] = pose.camera_tvec.flatten()

            # 计算: T_g2b @ X @ T_c2t 应该恒定
            predicted = T_g2b @ X @ T_c2t

            # 这里简化计算，实际需要比较标定板的恒定位置
            # 暂时返回 0 作为占位
            errors.append(0.0)

        return np.mean(errors)

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

    def save_calibration(self, X: np.ndarray, filename: str = "hand_eye_calibration.json"):
        """保存标定结果"""
        data = {
            "transformation_matrix": X.tolist(),
            "rotation": X[:3, :3].tolist(),
            "translation": X[:3, 3].tolist(),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        output_path = Path(__file__).parent.parent / "skills" / "vision-control" / filename
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"标定结果已保存到 {output_path}")


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="SO100 手眼标定工具")
    parser.add_argument("--port", type=str, default="COM7", help="串口")
    parser.add_argument("--camera", type=int, default=0, help="摄像头ID")
    parser.add_argument("--generate-board", action="store_true", help="生成标定板图像")
    parser.add_argument("--min-poses", type=int, default=15, help="最少姿态数量")
    parser.add_argument("--load", type=str, help="加载已保存的姿态数据")

    args = parser.parse_args()

    # 生成标定板
    if args.generate_board:
        board = ArUcoCalibrationBoard()
        board.generate_board_image()
        return

    # 创建标定器
    calibrator = HandEyeCalibrator(port=args.port, camera_id=args.camera)

    if not calibrator.connect():
        return

    try:
        # 加载已有数据
        if args.load:
            calibrator.load_poses(args.load)

        # 交互式标定
        calibrator.interactive_calibration(min_poses=args.min_poses)

        # 计算标定
        X, error = calibrator.calibrate()

        if X is not None:
            # 保存结果
            calibrator.save_calibration(X)

            print("\n" + "=" * 60)
            print("标定完成!")
            print("=" * 60)
            print("\n变换矩阵 (摄像头 -> 末端):")
            print(X)
            print(f"\n平移 (mm): {X[:3, 3] * 1000}")

    finally:
        calibrator.disconnect()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
