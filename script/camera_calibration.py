#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
相机内参标定工具
使用棋盘格标定摄像头内参矩阵和畸变系数
"""

import cv2
import numpy as np
import json
import time
from pathlib import Path
from typing import List, Tuple


class CameraCalibrator:
    """相机标定器"""

    def __init__(self, pattern_size: Tuple[int, int] = (9, 6), square_size: float = 0.025):
        """
        Args:
            pattern_size: 内角点数量 (cols, rows)
            square_size: 方格边长（米）
        """
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # 标定数据
        self.obj_points = []  # 3D 点
        self.img_points = []  # 2D 点
        self.images = []      # 采集的图像

        # 标定结果
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rvecs = None
        self.tvecs = None
        self.calibrated = False

    def detect_chessboard(self, image: np.ndarray) -> Tuple[bool, np.ndarray]:
        """
        检测棋盘格角点

        Args:
            image: 输入图像

        Returns:
            (ret, corners): 是否检测到、角点坐标
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, None)

        if ret:
            # 精细化角点
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), self.criteria)

        return ret, corners

    def add_image(self, image: np.ndarray) -> bool:
        """
        添加一张标定图像

        Args:
            image: 输入图像

        Returns:
            是否成功添加
        """
        ret, corners = self.detect_chessboard(image)

        if not ret:
            return False

        # 生成 3D 点
        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0],
                               0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size

        self.obj_points.append(objp)
        self.img_points.append(corners)
        self.images.append(image.copy())

        return True

    def calibrate(self) -> float:
        """
        执行相机标定

        Returns:
            重投影误差
        """
        if len(self.obj_points) < 5:
            print(f"图像数量不足，至少需要 5 张，当前: {len(self.obj_points)}")
            return -1.0

        print(f"\n开始标定，使用 {len(self.obj_points)} 张图像...")

        # 执行标定
        ret, self.camera_matrix, self.dist_coeffs, self.rvecs, self.tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points,
            self.images[0].shape[:2][::-1],
            None, None
        )

        if not ret:
            print("标定失败")
            return -1.0

        self.calibrated = True

        # 计算重投影误差
        total_error = 0
        for i in range(len(self.obj_points)):
            img_points_reproj, _ = cv2.projectPoints(
                self.obj_points[i], self.rvecs[i], self.tvecs[i],
                self.camera_matrix, self.dist_coeffs
            )
            error = cv2.norm(self.img_points[i], img_points_reproj, cv2.NORM_L2) / len(img_points_reproj)
            total_error += error

        mean_error = total_error / len(self.obj_points)

        print("\n标定完成!")
        print(f"相机内参矩阵:")
        print(self.camera_matrix)
        print(f"\n畸变系数:")
        print(self.dist_coeffs.flatten())
        print(f"\n重投影误差: {mean_error:.4f} 像素")

        return mean_error

    def undistort(self, image: np.ndarray) -> np.ndarray:
        """去畸变"""
        if not self.calibrated:
            return image
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

    def draw_corners(self, image: np.ndarray, corners: np.ndarray) -> np.ndarray:
        """绘制角点"""
        result = image.copy()
        cv2.drawChessboardCorners(result, self.pattern_size, corners, True)
        return result

    def save(self, filename: str = "camera_calibration.json"):
        """保存标定结果"""
        if not self.calibrated:
            print("尚未完成标定")
            return

        data = {
            "camera_matrix": self.camera_matrix.tolist(),
            "distortion_coefficients": self.dist_coeffs.flatten().tolist(),
            "image_size": self.images[0].shape[:2][::-1],
            "pattern_size": self.pattern_size,
            "square_size": self.square_size,
            "num_images": len(self.images),
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S")
        }

        output_path = Path(__file__).parent.parent / "skills" / "vision-control" / filename
        with open(output_path, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"\n标定结果已保存到: {output_path}")

    @classmethod
    def load(cls, filename: str):
        """加载标定结果"""
        with open(filename, 'r') as f:
            data = json.load(f)

        calibrator = cls(
            pattern_size=tuple(data["pattern_size"]),
            square_size=data["square_size"]
        )
        calibrator.camera_matrix = np.array(data["camera_matrix"])
        calibrator.dist_coeffs = np.array(data["distortion_coefficients"])
        calibrator.calibrated = True

        print(f"已加载相机标定结果")
        print(f"相机内参矩阵:")
        print(calibrator.camera_matrix)
        print(f"\n畸变系数:")
        print(calibrator.dist_coeffs.flatten())

        return calibrator


def interactive_calibration(camera_id: int = 1, min_images: int = 15):
    """
    交互式相机标定

    Args:
        camera_id: 摄像头ID
        min_images: 最少图像数量
    """
    print("\n" + "=" * 60)
    print("相机内参标定工具")
    print("=" * 60)
    print(f"\n操作说明:")
    print(f"1. 打印 chessboard.png 并放置在摄像头前")
    print(f"2. 移动标定板到不同位置和角度")
    print(f"3. 按 'c' 捕获图像（至少 {min_images} 张）")
    print(f"4. 按 'q' 完成标定")
    print(f"\n提示: 确保标定板在图像的不同位置、角度和距离")

    cap = cv2.VideoCapture(camera_id)
    calibrator = CameraCalibrator()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 检测棋盘格
        detected, corners = calibrator.detect_chessboard(frame)

        # 显示状态
        status = f"Images: {len(calibrator.obj_points)}/{min_images}"
        color = (0, 255, 0) if len(calibrator.obj_points) >= min_images else (0, 0, 255)

        cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        if detected:
            frame = calibrator.draw_corners(frame, corners)
            cv2.putText(frame, "Press 'c' to capture", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "No chessboard detected", (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if len(calibrator.obj_points) >= min_images:
            cv2.putText(frame, "Ready to calibrate! Press 'q'", (10, 110),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow("Camera Calibration", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            if len(calibrator.obj_points) >= min_images:
                break
            else:
                print(f"需要至少 {min_images} 张图像，当前: {len(calibrator.obj_points)}")
        elif key == ord('c'):
            if detected:
                success = calibrator.add_image(frame)
                if success:
                    print(f"已添加图像 {len(calibrator.obj_points)}")
            else:
                print("未检测到棋盘格，无法捕获")
        elif key == ord('u'):
            # 撤销最后一张
            if calibrator.obj_points:
                calibrator.obj_points.pop()
                calibrator.img_points.pop()
                calibrator.images.pop()
                print(f"撤销，剩余 {len(calibrator.obj_points)} 张图像")

    cap.release()
    cv2.destroyAllWindows()

    # 执行标定
    error = calibrator.calibrate()

    if error > 0:
        # 保存结果
        calibrator.save()

        # 显示去畸变效果
        print("\n按任意键退出...")
        ret, frame = cap.read()
        if ret:
            undistorted = calibrator.undistort(frame)
            cv2.imshow("Original", frame)
            cv2.imshow("Undistorted", undistorted)
            cv2.waitKey(0)

    return calibrator


def main():
    import argparse

    parser = argparse.ArgumentParser(description="相机内参标定工具")
    parser.add_argument("--camera", type=int, default=1, help="摄像头ID")
    parser.add_argument("--min-images", type=int, default=15, help="最少图像数量")

    args = parser.parse_args()

    interactive_calibration(camera_id=args.camera, min_images=args.min_images)


if __name__ == "__main__":
    main()
