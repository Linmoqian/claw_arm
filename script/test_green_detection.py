#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
绿色物体检测测试
简单打开摄像头，检测并框选绿色物体
"""

import cv2
import numpy as np

# 绿色 HSV 范围
GREEN_LOWER = (35, 50, 50)
GREEN_UPPER = (85, 255, 255)

def detect_green(frame):
    """检测绿色物体"""
    # 转换到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 创建掩码
    mask = cv2.inRange(hsv, np.array(GREEN_LOWER), np.array(GREEN_UPPER))

    # 去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    results = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # 最小面积阈值
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

def main():
    print("绿色物体检测测试")
    print("按 'q' 退出, 's' 调整HSV范围")
    print()

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("无法打开摄像头!")
        return

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center = (width // 2, height // 2)

    print(f"摄像头已打开: {width}x{height}")
    print("开始检测绿色物体...\n")

    # 可调整的HSV范围
    h_min, h_max = 35, 85
    s_min, s_max = 50, 255
    v_min, v_max = 50, 255

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 检测
        results, mask = detect_green(frame)

        # 显示结果
        display = frame.copy()

        for r in results:
            x, y, w, h = r['box']
            cx, cy = r['center']

            # 绘制边界框
            cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 绘制中心点
            cv2.circle(display, (cx, cy), 5, (0, 0, 255), -1)
            cv2.circle(display, (cx, cy), 10, (255, 255, 0), 2)

            # 显示面积
            cv2.putText(display, f"Area: {r['area']}", (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 绘制中心十字线
        cv2.line(display, (center[0] - 50, center[1]), (center[0] + 50, center[1]), (255, 255, 0), 1)
        cv2.line(display, (center[0], center[1] - 50), (center[0], center[1] + 50), (255, 255, 0), 1)

        # 显示检测数量
        cv2.putText(display, f"Green Objects: {len(results)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 显示HSV范围
        info = f"HSV: H({h_min}-{h_max}) S({s_min}-{s_max}) V({v_min}-{v_max})"
        cv2.putText(display, info, (10, height - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # 显示掩码（小窗口）
        mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_display = cv2.resize(mask_display, (160, 120))
        display[0:120, width-160:width] = mask_display

        cv2.imshow("Green Detection", display)

        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('s'):
            # 调整HSV范围
            print("\n调整HSV范围:")
            h_min = int(input(f"  H_min (当前{h_min}): ") or h_min)
            h_max = int(input(f"  H_max (当前{h_max}): ") or h_max)
            s_min = int(input(f"  S_min (当前{s_min}): ") or s_min)
            s_max = int(input(f"  S_max (当前{s_max}): ") or s_max)

            global GREEN_LOWER, GREEN_UPPER
            GREEN_LOWER = (h_min, s_min, v_min)
            GREEN_UPPER = (h_max, s_max, v_max)
            print(f"新范围: {GREEN_LOWER} - {GREEN_UPPER}")

    cap.release()
    cv2.destroyAllWindows()
    print("\n程序结束")

if __name__ == "__main__":
    main()
