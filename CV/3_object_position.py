#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
物体位置检测
基于颜色检测输出物体中心的像素坐标
"""

import cv2
import numpy as np
from cv_utils import Colors, print_colored


# 绿色 HSV 范围
GREEN_LOWER = (35, 50, 50)
GREEN_UPPER = (85, 255, 255)

# 最小检测面积阈值
MIN_AREA = 500


def detect_objects(frame, hsv_lower: tuple, hsv_upper: tuple) -> list:
    """
    检测指定颜色范围内的物体

    Args:
        frame: BGR 图像帧
        hsv_lower: HSV 下限 (h, s, v)
        hsv_upper: HSV 上限 (h, s, v)

    Returns:
        检测结果列表，每个元素包含 center, box, area
    """
    # 转换到 HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 创建掩码
    mask = cv2.inRange(hsv, np.array(hsv_lower), np.array(hsv_upper))

    # 形态学去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    results = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > MIN_AREA:
            x, y, w, h = cv2.boundingRect(contour)
            M = cv2.moments(contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                results.append({
                    'center': (cx, cy),
                    'box': (x, y, w, h),
                    'area': area
                })

    return results, mask


def draw_detections(frame, results: list) -> np.ndarray:
    """
    在画面上绘制检测结果

    Args:
        frame: 原始图像帧
        results: 检测结果列表

    Returns:
        标注后的图像
    """
    display = frame.copy()
    height, width = frame.shape[:2]
    frame_center = (width // 2, height // 2)

    for i, r in enumerate(results, 1):
        x, y, w, h = r['box']
        cx, cy = r['center']

        # 绘制边界框
        cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 绘制中心点
        cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
        cv2.circle(display, (cx, cy), 10, (255, 255, 0), 2)

        # 标注坐标值
        coord_text = f"({cx}, {cy})"
        cv2.putText(display, coord_text, (cx + 15, cy - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # 显示编号和面积
        label = f"#{i} Area:{r['area']}"
        cv2.putText(display, label, (x, y - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 绘制画面中心十字线
    cv2.line(display, (frame_center[0] - 30, frame_center[1]),
             (frame_center[0] + 30, frame_center[1]), (255, 255, 0), 1)
    cv2.line(display, (frame_center[0], frame_center[1] - 30),
             (frame_center[0], frame_center[1] + 30), (255, 255, 0), 1)

    # 显示检测数量
    cv2.putText(display, f"Objects: {len(results)}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 显示操作提示
    cv2.putText(display, "Press 'q' to quit", (10, height - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return display


def main():
    """主函数"""
    print_colored("\n物体位置检测", Colors.CYAN)
    print_colored("基于颜色检测输出物体中心像素坐标", Colors.GRAY)
    print()
    print_colored("操作说明:", Colors.BLUE)
    print_colored("  q - 退出程序", Colors.GRAY)
    print()

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print_colored("错误: 无法打开摄像头!", Colors.RED)
        return

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    print_colored(f"摄像头已连接: {width}x{height}", Colors.GREEN)
    print_colored("开始检测绿色物体...\n", Colors.GREEN)

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print_colored("警告: 读取帧失败", Colors.YELLOW)
                continue

            # 检测物体
            results, mask = detect_objects(frame, GREEN_LOWER, GREEN_UPPER)

            # 绘制检测结果
            display = draw_detections(frame, results)

            # 显示掩码小窗口
            mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_display = cv2.resize(mask_display, (160, 120))
            display[0:120, width - 160:width] = mask_display

            cv2.imshow("Object Position Detection", display)

            # 终端打印坐标信息
            if results:
                for i, r in enumerate(results, 1):
                    cx, cy = r['center']
                    print_colored(
                        f"物体 #{i}: 坐标 ({cx:4d}, {cy:4d})  面积: {int(r['area']):6d}",
                        Colors.GREEN
                    )
            else:
                print_colored("未检测到物体", Colors.YELLOW)

            # 检查按键
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
                break

    except KeyboardInterrupt:
        print_colored("\n用户中断", Colors.YELLOW)

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print_colored("\n程序已退出", Colors.CYAN)


if __name__ == "__main__":
    main()
