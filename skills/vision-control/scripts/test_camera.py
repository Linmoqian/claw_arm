#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
摄像头测试工具
用于测试摄像头是否正常工作
"""

import cv2
import sys


def test_camera(camera_id: int = 0):
    """
    测试摄像头

    Args:
        camera_id: 摄像头设备ID
    """
    print("=" * 60)
    print("SO100 摄像头测试工具")
    print("=" * 60)
    print(f"\n正在打开摄像头 {camera_id}...")

    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"错误: 无法打开摄像头 {camera_id}")
        return False

    # 获取摄像头属性
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))

    print(f"✓ 摄像头已打开")
    print(f"  分辨率: {width}x{height}")
    print(f"  帧率: {fps} FPS")
    print("\n操作说明:")
    print("  按 'q' 退出")
    print("  按 's' 保存截图")

    # 创建窗口
    cv2.namedWindow("Camera Test")

    frame_count = 0

    while True:
        ret, frame = cap.read()

        if not ret:
            print("错误: 无法读取图像")
            break

        frame_count += 1

        # 在图像上显示信息
        info_text = f"Frame: {frame_count} | {width}x{height} | {fps} FPS"
        cv2.putText(frame, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 显示图像
        cv2.imshow("Camera Test", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            print(f"\n已捕获 {frame_count} 帧")
            break
        elif key == ord('s'):
            # 保存截图
            filename = f"camera_test_{frame_count}.jpg"
            cv2.imwrite(filename, frame)
            print(f"✓ 截图已保存: {filename}")

    cap.release()
    cv2.destroyAllWindows()

    print("\n摄像头测试完成")

    return True


def list_cameras():
    """列出可用的摄像头"""
    print("\n扫描可用摄像头...")

    available = []

    for i in range(5):  # 测试前 5 个设备
        cap = cv2.VideoCapture(i)

        if cap.isOpened():
            ret, frame = cap.read()

            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                available.append((i, width, height))
                print(f"  摄像头 {i}: {width}x{height}")

            cap.release()

    if not available:
        print("  未找到可用摄像头")

    return available


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="SO100 摄像头测试工具")
    parser.add_argument("--camera", type=int, default=0, help="摄像头设备ID")
    parser.add_argument("--list", action="store_true", help="列出所有可用摄像头")

    args = parser.parse_args()

    if args.list:
        list_cameras()
    else:
        test_camera(args.camera)


if __name__ == "__main__":
    main()
