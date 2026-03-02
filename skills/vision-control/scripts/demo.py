#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
vision-detection 技能演示脚本
展示如何使用视觉检测器获取物体位置
"""

import sys
import os

# 添加项目根目录到路径
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, project_root)

from scripts.vision_detector import VisionDetector


def demo_basic_detection():
    """基础检测演示"""
    print("=" * 60)
    print("演示 1: 基础物体检测")
    print("=" * 60)

    # 创建检测器
    detector = VisionDetector(
        camera_id=0,
        detector="yolo",
        output_format="all"
    )

    try:
        # 加载模型
        detector.load_model()

        # 检测杯子
        print("\n正在检测杯子...")
        results = detector.detect_objects(target="cup", confidence=0.5)

        if results:
            print(f"\n找到 {len(results)} 个杯子:\n")
            for i, obj in enumerate(results, 1):
                print(f"[杯子 {i}]")
                print(f"  置信度: {obj.confidence:.2f}")
                print(f"  像素坐标: {obj.pixel}")
                print(f"  边界框: {obj.bounding_box}")

                if obj.arm_3d:
                    print(f"  3D坐标: X={obj.arm_3d[0]:.1f}mm, Y={obj.arm_3d[1]:.1f}mm, Z={obj.arm_3d[2]:.1f}mm")

                if obj.polar:
                    print(f"  极坐标: 角度={obj.polar[0]:.1f}°, 距离={obj.polar[1]:.1f}px")
                print()

            # 显示结果
            detector.show_results(results, annotate=True, wait_key=True)
        else:
            print("未检测到杯子")

    finally:
        detector.release()


def demo_multi_format():
    """多格式输出演示"""
    print("\n" + "=" * 60)
    print("演示 2: 多格式坐标输出")
    print("=" * 60)

    # 创建检测器 - 只输出像素坐标
    detector_pixel = VisionDetector(camera_id=0, output_format="pixel")

    # 创建检测器 - 只输出3D坐标
    detector_3d = VisionDetector(camera_id=0, output_format="3d")

    # 创建检测器 - 只输出极坐标
    detector_polar = VisionDetector(camera_id=0, output_format="polar")

    try:
        detector_pixel.load_model()

        print("\n正在检测手机...")
        results = detector_pixel.detect_objects(target="cell phone", confidence=0.5)

        if results:
            obj = results[0]
            print(f"\n检测到手机 (置信度: {obj.confidence:.2f})")
            print(f"\n像素坐标: {obj.pixel}")
            print(f"边界框: {obj.bounding_box}")

            # 手动计算其他格式
            arm_3d = detector_pixel._pixel_to_arm_3d(obj.pixel[0], obj.pixel[1],
                                                    (obj.bounding_box[2]-obj.bounding_box[0],
                                                     obj.bounding_box[3]-obj.bounding_box[1]))
            if arm_3d:
                print(f"\n机械臂3D坐标: ({arm_3d[0]:.1f}, {arm_3d[1]:.1f}, {arm_3d[2]:.1f}) mm")

            polar = detector_pixel._pixel_to_polar(obj.pixel[0], obj.pixel[1])
            print(f"\n极坐标 (相对于图像中心):")
            print(f"  角度: {polar[0]:.1f}°")
            print(f"  距离: {polar[1]:.1f} 像素")

    finally:
        detector_pixel.release()


def demo_polar_tracking():
    """极坐标追踪演示"""
    print("\n" + "=" * 60)
    print("演示 3: 基于极坐标的目标追踪")
    print("=" * 60)

    detector = VisionDetector(
        camera_id=0,
        detector="yolo",
        output_format="polar"
    )

    try:
        detector.load_model()

        print("\n追踪瓶子 (按 'q' 退出)...")
        detector.track_object(
            target="bottle",
            duration=30,
            confidence=0.5
        )

    finally:
        detector.release()


def demo_color_detection():
    """颜色检测演示"""
    print("\n" + "=" * 60)
    print("演示 4: 颜色检测")
    print("=" * 60)

    detector = VisionDetector(
        camera_id=0,
        detector="color",
        output_format="all"
    )

    try:
        detector.load_model()

        # 检测红色物体
        print("\n正在检测红色物体...")
        red_lower = (0, 120, 70)  # HSV
        red_upper = (10, 255, 255)

        results = detector.detect_objects(
            target="red",
            color_lower=red_lower,
            color_upper=red_upper,
            max_results=5
        )

        if results:
            print(f"\n找到 {len(results)} 个红色物体:")
            for i, obj in enumerate(results, 1):
                print(f"\n[红色物体 {i}]")
                print(f"  像素坐标: {obj.pixel}")
                print(f"  面积: {obj.bounding_box[2] * obj.bounding_box[3]} px²")

                if obj.polar:
                    print(f"  极坐标: {obj.polar[0]:.1f}° @ {obj.polar[1]:.1f}px")

            # 显示结果
            detector.show_results(results, annotate=True, wait_key=True)
        else:
            print("未检测到红色物体")

    finally:
        detector.release()


def demo_coordinate_conversion():
    """坐标转换演示"""
    print("\n" + "=" * 60)
    print("演示 5: 坐标系转换")
    print("=" * 60)

    detector = VisionDetector(camera_id=0, output_format="all")

    try:
        detector.load_model()

        # 示例: 假设检测到物体在像素坐标 (320, 240)
        pixel_x, pixel_y = 320, 240

        print(f"\n像素坐标: ({pixel_x}, {pixel_y})")

        # 转换为极坐标
        angle, distance = detector.get_polar_coord(pixel_x, pixel_y)
        print(f"极坐标: {angle:.1f}°, {distance:.1f}px")

        # 转换为3D坐标(需要标定数据)
        arm_3d = detector._pixel_to_arm_3d(pixel_x, pixel_y, (100, 100))
        if arm_3d:
            print(f"3D坐标: ({arm_3d[0]:.1f}, {arm_3d[1]:.1f}, {arm_3d[2]:.1f}) mm")

        print("\n坐标系说明:")
        print("  - 像素坐标: 原点在图像左上角,单位像素")
        print("  - 极坐标: 原点在图像中心,角度和距离")
        print("  - 3D坐标: 原点在机械臂底座,单位毫米")

    finally:
        detector.release()


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="vision-detection 技能演示")
    parser.add_argument("--demo", type=str, default="basic",
                       choices=["basic", "multi", "tracking", "color", "conversion", "all"],
                       help="运行的演示类型")

    args = parser.parse_args()

    print("\n")
    print("╔" + "═" * 58 + "╗")
    print("║" + " " * 15 + "vision-detection 演示" + " " * 23 + "║")
    print("╚" + "═" * 58 + "╝")

    if args.demo == "basic" or args.demo == "all":
        demo_basic_detection()

    if args.demo == "multi" or args.demo == "all":
        demo_multi_format()

    if args.demo == "tracking" or args.demo == "all":
        demo_polar_tracking()

    if args.demo == "color" or args.demo == "all":
        demo_color_detection()

    if args.demo == "conversion" or args.demo == "all":
        demo_coordinate_conversion()

    print("\n" + "=" * 60)
    print("演示完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
