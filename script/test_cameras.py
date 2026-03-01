#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
测试所有可用的 OpenCV 相机
"""

import cv2
import time

def test_camera(camera_id, width=640, height=480):
    """测试单个相机"""
    print(f"\n测试相机 #{camera_id}...")
    print(f"  尝试打开: {camera_id}")

    # 尝试打开相机
    cap = cv2.VideoCapture(camera_id)

    if not cap.isOpened():
        print(f"  [失败] 无法打开相机 #{camera_id}")
        return False

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    # 读取一帧
    ret, frame = cap.read()

    if not ret or frame is None:
        print(f"  [失败] 无法读取画面")
        cap.release()
        return False

    # 获取实际分辨率
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = int(cap.get(cv2.CAP_PROP_FPS))

    print(f"  [成功] 相机工作正常！")
    print(f"    实际分辨率: {actual_width} x {actual_height}")
    print(f"    FPS: {actual_fps}")
    print(f"    画面形状: {frame.shape}")

    # 保存测试图像
    filename = f"test_camera_{camera_id}.jpg"
    cv2.imwrite(filename, frame)
    print(f"    测试图像已保存: {filename}")

    cap.release()
    return True

def main():
    print("=" * 60)
    print("OpenCV 相机测试程序")
    print("=" * 60)

    # 测试相机 0-5
    working_cameras = []
    for i in range(6):
        if test_camera(i):
            working_cameras.append(i)

    print("\n" + "=" * 60)
    print("测试总结")
    print("=" * 60)
    print(f"\n可用的相机 ID: {working_cameras}")
    print(f"共 {len(working_cameras)} 个相机可用")

    if working_cameras:
        print("\n推荐配置:")
        print("  - 主相机:", working_cameras[0] if working_cameras else "无")
        print("  - 副相机:", working_cameras[1] if len(working_cameras) > 1 else "无")
        print("\n使用示例:")
        for cam_id in working_cameras:
            print(f"  camera{cam_id} = CameraConfig(")
            print(f"      type='opencv',")
            print(f"      camera_id={cam_id},")
            print(f"      width=640,")
            print(f"      height=480,")
            print(f"      fps=30")
            print(f"  )")

if __name__ == "__main__":
    main()
