#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
世界坐标转换
将像素坐标转换为世界坐标（基于固定 Z 高度假设）
"""

import json
import cv2
import numpy as np
from pathlib import Path
from cv_utils import Colors, print_colored


def load_camera_config(config_path: str = None) -> dict:
    """
    加载相机标定配置

    Args:
        config_path: 配置文件路径，默认为同目录下的 camera_config.json

    Returns:
        包含 camera_matrix, dist_coeffs, fixed_height 的字典
    """
    if config_path is None:
        config_path = Path(__file__).parent / "camera_config.json"

    with open(config_path, 'r', encoding='utf-8') as f:
        config = json.load(f)

    return {
        'camera_matrix': np.array(config['camera_matrix'], dtype=np.float64),
        'dist_coeffs': np.array(config['dist_coeffs'], dtype=np.float64),
        'fixed_height': config['fixed_height']
    }


def pixel_to_world(pixel_x: float, pixel_y: float, camera_matrix: np.ndarray,
                   dist_coeffs: np.ndarray, fixed_z: float) -> tuple:
    """
    将像素坐标转换为世界坐标（单目相机，固定 Z 高度假设）

    原理：
    1. 去除畸变
    2. 使用针孔相机模型反投影
    3. 根据 Z 高度计算 X, Y

    Args:
        pixel_x: 像素 x 坐标
        pixel_y: 像素 y 坐标
        camera_matrix: 3x3 相机内参矩阵
        dist_coeffs: 畸变系数
        fixed_z: 固定的世界坐标 Z 值（高度）

    Returns:
        (X, Y, Z) 世界坐标，单位：米
    """
    # 像素点数组
    pixel_points = np.array([[pixel_x, pixel_y]], dtype=np.float64)
    pixel_points = pixel_points.reshape(-1, 1, 2)

    # 去除畸变，得到归一化坐标
    undistorted = cv2.undistortPoints(pixel_points, camera_matrix, dist_coeffs)

    # 归一化坐标 (x', y') 对应相机坐标系中的方向向量
    # 其中 x' = X/Z, y' = Y/Z
    x_norm = undistorted[0, 0, 0]
    y_norm = undistorted[0, 0, 1]

    # 假设 Z = fixed_z，计算 X, Y
    # X = x' * Z
    # Y = y' * Z
    world_x = x_norm * fixed_z
    world_y = y_norm * fixed_z
    world_z = fixed_z

    return (world_x, world_y, world_z)


def pixel_to_world_simple(pixel_x: float, pixel_y: float, camera_matrix: np.ndarray,
                          fixed_z: float) -> tuple:
    """
    简化版像素到世界坐标转换（忽略畸变）

    直接使用相机内参矩阵进行反投影计算

    Args:
        pixel_x: 像素 x 坐标
        pixel_y: 像素 y 坐标
        camera_matrix: 3x3 相机内参矩阵
        fixed_z: 固定的世界坐标 Z 值（高度）

    Returns:
        (X, Y, Z) 世界坐标，单位：米
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # 相机模型反投影
    # x = (u - cx) * Z / fx
    # y = (v - cy) * Z / fy
    world_x = (pixel_x - cx) * fixed_z / fx
    world_y = (pixel_y - cy) * fixed_z / fy
    world_z = fixed_z

    return (world_x, world_y, world_z)


def world_to_pixel(world_x: float, world_y: float, world_z: float,
                   camera_matrix: np.ndarray, dist_coeffs: np.ndarray) -> tuple:
    """
    将世界坐标转换为像素坐标（用于验证）

    Args:
        world_x: 世界坐标 X
        world_y: 世界坐标 Y
        world_z: 世界坐标 Z
        camera_matrix: 3x3 相机内参矩阵
        dist_coeffs: 畸变系数

    Returns:
        (pixel_x, pixel_y) 像素坐标
    """
    # 世界坐标点（假设相机坐标系 = 世界坐标系）
    points_3d = np.array([[world_x, world_y, world_z]], dtype=np.float64)

    # 3D 到 2D 投影
    points_2d, _ = cv2.projectPoints(
        points_3d,
        np.zeros(3, dtype=np.float64),  # 旋转向量（无旋转）
        np.zeros(3, dtype=np.float64),  # 平移向量（无平移）
        camera_matrix,
        dist_coeffs
    )

    pixel_x = points_2d[0, 0, 0]
    pixel_y = points_2d[0, 0, 1]

    return (pixel_x, pixel_y)


class WorldCoordinateConverter:
    """世界坐标转换器类"""

    def __init__(self, config_path: str = None):
        """
        初始化转换器

        Args:
            config_path: 配置文件路径
        """
        config = load_camera_config(config_path)
        self.camera_matrix = config['camera_matrix']
        self.dist_coeffs = config['dist_coeffs']
        self.fixed_height = config['fixed_height']

        print_colored("世界坐标转换器已初始化", Colors.GREEN)
        print_colored(f"  固定高度 Z = {self.fixed_height} m", Colors.GRAY)

    def convert(self, pixel_x: float, pixel_y: float, z_height: float = None) -> tuple:
        """
        转换像素坐标到世界坐标

        Args:
            pixel_x: 像素 x 坐标
            pixel_y: 像素 y 坐标
            z_height: Z 高度，默认使用配置中的固定高度

        Returns:
            (X, Y, Z) 世界坐标
        """
        if z_height is None:
            z_height = self.fixed_height

        return pixel_to_world(pixel_x, pixel_y, self.camera_matrix,
                             self.dist_coeffs, z_height)

    def convert_simple(self, pixel_x: float, pixel_y: float,
                       z_height: float = None) -> tuple:
        """
        简化版转换（忽略畸变）

        Args:
            pixel_x: 像素 x 坐标
            pixel_y: 像素 y 坐标
            z_height: Z 高度，默认使用配置中的固定高度

        Returns:
            (X, Y, Z) 世界坐标
        """
        if z_height is None:
            z_height = self.fixed_height

        return pixel_to_world_simple(pixel_x, pixel_y, self.camera_matrix, z_height)

    def reverse(self, world_x: float, world_y: float, world_z: float) -> tuple:
        """
        反向转换：世界坐标到像素坐标

        Args:
            world_x: 世界坐标 X
            world_y: 世界坐标 Y
            world_z: 世界坐标 Z

        Returns:
            (pixel_x, pixel_y) 像素坐标
        """
        return world_to_pixel(world_x, world_y, world_z,
                             self.camera_matrix, self.dist_coeffs)

    def print_config(self):
        """打印当前配置信息"""
        print_colored("\n相机配置信息:", Colors.CYAN)
        print_colored("内参矩阵:", Colors.BLUE)
        print(self.camera_matrix)
        print_colored("畸变系数:", Colors.BLUE)
        print(self.dist_coeffs)
        print_colored(f"固定高度: {self.fixed_height} m", Colors.BLUE)


def main():
    """主函数：演示坐标转换"""
    print_colored("\n世界坐标转换演示", Colors.CYAN)
    print_colored("将像素坐标转换为世界坐标（固定 Z 高度假设）", Colors.GRAY)
    print()

    # 创建转换器
    converter = WorldCoordinateConverter()

    # 打印配置
    converter.print_config()

    # 测试转换
    print_colored("\n转换测试:", Colors.CYAN)

    test_points = [
        (320, 240),   # 图像中心
        (0, 0),       # 左上角
        (640, 480),   # 右下角
        (160, 120),   # 左上区域
        (480, 360),   # 右下区域
    ]

    print_colored("\n像素坐标 -> 世界坐标:", Colors.BLUE)
    print(f"{'像素坐标':<20} {'世界坐标 (完整)':<30} {'世界坐标 (简化)':<30}")
    print("-" * 80)

    for px, py in test_points:
        # 完整转换（含去畸变）
        world_full = converter.convert(px, py)
        # 简化转换
        world_simple = converter.convert_simple(px, py)

        pixel_str = f"({px}, {py})"
        world_full_str = f"({world_full[0]:.4f}, {world_full[1]:.4f}, {world_full[2]:.4f})"
        world_simple_str = f"({world_simple[0]:.4f}, {world_simple[1]:.4f}, {world_simple[2]:.4f})"

        print(f"{pixel_str:<20} {world_full_str:<30} {world_simple_str:<30}")

    # 反向验证
    print_colored("\n反向验证 (世界坐标 -> 像素坐标):", Colors.BLUE)
    print(f"{'世界坐标':<30} {'像素坐标':<20}")
    print("-" * 50)

    for px, py in test_points:
        world = converter.convert(px, py)
        back_pixel = converter.reverse(*world)
        world_str = f"({world[0]:.4f}, {world[1]:.4f}, {world[2]:.4f})"
        pixel_str = f"({back_pixel[0]:.1f}, {back_pixel[1]:.1f})"
        print(f"{world_str:<30} {pixel_str:<20}")

    # 交互式输入
    print_colored("\n交互式转换 (输入 'q' 退出):", Colors.CYAN)

    while True:
        try:
            user_input = input("\n输入像素坐标 (格式: x,y 或 x y): ").strip()

            if user_input.lower() == 'q':
                break

            # 解析输入
            if ',' in user_input:
                parts = user_input.split(',')
            else:
                parts = user_input.split()

            if len(parts) >= 2:
                px = float(parts[0].strip())
                py = float(parts[1].strip())
                z = float(parts[2].strip()) if len(parts) >= 3 else None

                world = converter.convert(px, py, z)
                print_colored(
                    f"像素 ({px}, {py}) -> 世界 ({world[0]:.4f}, {world[1]:.4f}, {world[2]:.4f}) m",
                    Colors.GREEN
                )
            else:
                print_colored("请输入两个坐标值", Colors.YELLOW)

        except ValueError as e:
            print_colored(f"输入格式错误: {e}", Colors.RED)
        except KeyboardInterrupt:
            break

    print_colored("\n程序已退出", Colors.CYAN)


if __name__ == "__main__":
    main()
