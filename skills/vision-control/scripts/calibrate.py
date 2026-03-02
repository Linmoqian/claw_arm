#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
手眼标定工具
用于标定相机坐标系和机械臂坐标系之间的转换关系
"""

import sys
import os
import json
import cv2
import numpy as np
from typing import List, Dict, Tuple

# 添加项目根目录到路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, project_root)

from scripts.vision_controller import VisionController


class HandEyeCalibrator:
    """手眼标定器"""

    def __init__(self, port: str = "COM7", camera_id: int = 0):
        self.vc = VisionController(port=port, camera_id=camera_id)
        self.calibration_points = []
        self.current_frame = None

    def start(self):
        """开始标定流程"""
        print("=" * 60)
        print("SO100 机械臂手眼标定工具")
        print("=" * 60)

        # 连接设备
        self.vc.connect()
        self.vc.open_camera()

        print("\n标定说明:")
        print("1. 在工作空间放置一个明显的标记点（如红色圆形物体）")
        print("2. 使用摄像头对准标记点，点击图像记录像素坐标")
        print("3. 手动移动机械臂末端到标记点位置，输入当前机械臂坐标")
        print("4. 重复上述步骤，至少采集 6-10 个标定点")
        print("5. 标定点分布要覆盖整个工作空间\n")

        input("准备好后按 Enter 继续...")

        # 开始标定循环
        self.calibration_loop()

        # 计算标定矩阵
        if len(self.calibration_points) >= 4:
            self.calculate_calibration()
        else:
            print(f"错误: 标定点数量不足 ({len(self.calibration_points)}/4)")

    def calibration_loop(self):
        """标定点采集循环"""
        point_num = 1

        while True:
            print(f"\n--- 标定点 {point_num} ---")
            print("操作说明:")
            print("1. 移动标记点到新位置")
            print("2. 在图像窗口中点击标记点中心")
            print("3. 按 'a' 接受该点")
            print("4. 按 'q' 完成标定")
            print("5. 按 'c' 清除所有点重新开始\n")

            # 显示摄像头画面
            self.select_point_on_screen(point_num)

            # 输入机械臂坐标
            print("\n请手动移动机械臂末端到标记点位置...")
            input("移动完成后按 Enter...")

            # 读取当前机械臂位置（简化版，使用预设值）
            print("\n输入机械臂坐标（mm）:")
            arm_x = float(input("  X: "))
            arm_y = float(input("  Y: "))
            arm_z = float(input("  Z: "))

            # 保存标定点
            self.calibration_points.append({
                "pixel": self.last_pixel_point,
                "arm": (arm_x, arm_y, arm_z)
            })

            print(f"\n✓ 已添加标定点 {point_num}")
            print(f"  像素坐标: {self.last_pixel_point}")
            print(f"  机械臂坐标: ({arm_x}, {arm_y}, {arm_z})")
            print(f"  总点数: {len(self.calibration_points)}")

            point_num += 1

            # 询问是否继续
            if len(self.calibration_points) >= 4:
                cont = input("\n继续采集？(y/n, 默认 y): ").strip().lower()
                if cont == 'n':
                    break

    def select_point_on_screen(self, point_num: int):
        """在屏幕上选择点"""
        cv2.namedWindow(f"Calibration - Point {point_num}")

        # 鼠标回调
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                self.last_pixel_point = (x, y)
                # 在图像上绘制标记
                frame = self.current_frame.copy()
                cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
                cv2.putText(frame, f"({x}, {y})", (x + 15, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                cv2.imshow(f"Calibration - Point {point_num}", frame)

        cv2.setMouseCallback(f"Calibration - Point {point_num}", mouse_callback)

        # 实时显示
        while True:
            frame = self.vc.capture_frame()
            self.current_frame = frame

            # 显示当前点数
            cv2.putText(frame, f"Points: {len(self.calibration_points)}",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                       1, (0, 255, 0), 2)

            # 显示提示
            cv2.putText(frame, "Click to select point, 'a' to accept, 'q' to quit",
                       (10, frame.shape[0] - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            cv2.imshow(f"Calibration - Point {point_num}", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                # 退出标定
                self.vc.disconnect()
                cv2.destroyAllWindows()
                exit(0)

            elif key == ord('a'):
                # 确认选择
                if hasattr(self, 'last_pixel_point'):
                    cv2.destroyAllWindows()
                    break
                else:
                    print("请先点击选择一个点")

    def calculate_calibration(self):
        """计算标定矩阵"""
        print("\n" + "=" * 60)
        print("计算标定矩阵...")
        print("=" * 60)

        # 显示所有标定点
        print(f"\n已采集 {len(self.calibration_points)} 个标定点:")
        for i, point in enumerate(self.calibration_points, 1):
            pixel = point["pixel"]
            arm = point["arm"]
            print(f"  {i}. 像素: {pixel}, 机械臂: ({arm[0]:.1f}, {arm[1]:.1f}, {arm[2]:.1f})")

        # 使用视觉控制器的标定方法
        matrix = self.vc.calibrate_hand_eye(self.calibration_points)

        # 保存标定结果
        filename = input("\n保存标定文件 (直接回车使用默认路径): ").strip()
        if not filename:
            filename = None

        self.vc.save_calibration(filename)

        # 测试标定结果
        print("\n是否测试标定结果？(y/n): ", end="")
        if input().strip().lower() == 'y':
            self.test_calibration()

    def test_calibration(self):
        """测试标定结果"""
        print("\n测试标定:")
        print("移动标记点到任意位置，点击图像中心，观察机械臂是否准确移动到该位置")

        cv2.namedWindow("Calibration Test")

        test_point = None

        def mouse_callback(event, x, y, flags, param):
            nonlocal test_point
            if event == cv2.EVENT_LBUTTONDOWN:
                test_point = (x, y)
                # 绘制标记
                frame = self.vc.capture_frame()
                cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)
                cv2.imshow("Calibration Test", frame)

                # 转换坐标
                arm_x, arm_y, arm_z = self.vc.pixel_to_arm_coord(x, y)
                print(f"\n像素坐标: ({x}, {y})")
                print(f"机械臂坐标: ({arm_x:.1f}, {arm_y:.1f}, {arm_z:.1f})")

                # 移动机械臂
                confirm = input("移动机械臂到此位置？(y/n): ").strip().lower()
                if confirm == 'y':
                    joints = self.vc.arm_coord_to_joints(arm_x, arm_y, arm_z)
                    self.vc.arm.move_smooth(joints)

        cv2.setMouseCallback("Calibration Test", mouse_callback)

        print("\n按 'q' 退出测试")

        while True:
            frame = self.vc.capture_frame()
            cv2.imshow("Calibration Test", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

        cv2.destroyAllWindows()


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="SO100 手眼标定工具")
    parser.add_argument("--port", type=str, default="COM7", help="串口端口号")
    parser.add_argument("--camera", type=int, default=0, help="摄像头设备ID")

    args = parser.parse_args()

    calibrator = HandEyeCalibrator(port=args.port, camera_id=args.camera)
    calibrator.start()


if __name__ == "__main__":
    main()
