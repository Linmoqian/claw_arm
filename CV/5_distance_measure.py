#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
基于颜色检测的物体距离测量

使用已知物体实际尺寸 + 相机焦距计算距离
公式：distance = (real_width * focal_length) / pixel_width
"""

import cv2
import numpy as np
import json
from pathlib import Path

# ============== 配置参数 ==============
# 物体实际宽度（厘米）
KNOWN_WIDTH = 10.0

# 相机焦距（像素单位）- 需要通过标定获得
FOCAL_LENGTH = 600.0

# 颜色检测范围 (HSV) - 默认红色
COLOR_LOWER = np.array([0, 100, 100])
COLOR_UPPER = np.array([10, 255, 255])
COLOR_LOWER2 = np.array([160, 100, 100])  # 红色需要两个范围
COLOR_UPPER2 = np.array([180, 255, 255])

# 最小检测面积（像素）
MIN_CONTOUR_AREA = 500

# 相机索引
CAMERA_INDEX = 0

# 配置文件路径
CONFIG_FILE = Path(__file__).parent / "camera_config.json"


def load_config():
    """加载配置文件"""
    global KNOWN_WIDTH, FOCAL_LENGTH, COLOR_LOWER, COLOR_UPPER, COLOR_LOWER2, COLOR_UPPER2

    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                config = json.load(f)

            if "distance_measure" in config:
                dm_config = config["distance_measure"]
                KNOWN_WIDTH = dm_config.get("known_width", KNOWN_WIDTH)
                FOCAL_LENGTH = dm_config.get("focal_length", FOCAL_LENGTH)

                if "color_range" in dm_config:
                    color = dm_config["color_range"]
                    COLOR_LOWER = np.array(color.get("lower", COLOR_LOWER.tolist()))
                    COLOR_UPPER = np.array(color.get("upper", COLOR_UPPER.tolist()))
                    COLOR_LOWER2 = np.array(color.get("lower2", COLOR_LOWER2.tolist()))
                    COLOR_UPPER2 = np.array(color.get("upper2", COLOR_UPPER2.tolist()))

                print(f"[配置] 已加载 camera_config.json")
                print(f"  - 已知物体宽度: {KNOWN_WIDTH} cm")
                print(f"  - 焦距: {FOCAL_LENGTH} px")
        except Exception as e:
            print(f"[警告] 配置文件加载失败: {e}")


def calculate_distance(pixel_width: float, real_width: float = None, focal_length: float = None) -> float:
    """
    计算物体到摄像头的距离

    Args:
        pixel_width: 检测到的物体像素宽度
        real_width: 物体实际宽度（厘米），None 则使用全局配置
        focal_length: 相机焦距（像素），None 则使用全局配置

    Returns:
        距离（厘米）
    """
    real_width = real_width or KNOWN_WIDTH
    focal_length = focal_length or FOCAL_LENGTH

    if pixel_width <= 0:
        return 0.0

    # distance = (real_width * focal_length) / pixel_width
    distance = (real_width * focal_length) / pixel_width
    return distance


def detect_colored_object(frame: np.ndarray) -> tuple:
    """
    检测指定颜色的物体

    Args:
        frame: BGR 图像帧

    Returns:
        (contour, pixel_width, pixel_height, center) 或 (None, 0, 0, None)
    """
    # 转换到 HSV 颜色空间
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 创建颜色掩码（红色需要两个范围）
    mask1 = cv2.inRange(hsv, COLOR_LOWER, COLOR_UPPER)
    mask2 = cv2.inRange(hsv, COLOR_LOWER2, COLOR_UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    # 形态学操作：去除噪声
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=2)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, 0, 0, None

    # 找到最大的轮廓
    max_contour = max(contours, key=cv2.contourArea)

    # 检查面积是否足够大
    if cv2.contourArea(max_contour) < MIN_CONTOUR_AREA:
        return None, 0, 0, None

    # 获取边界矩形
    x, y, w, h = cv2.boundingRect(max_contour)

    # 计算中心点
    center = (x + w // 2, y + h // 2)

    return max_contour, w, h, center


def draw_info(frame: np.ndarray, contour, pixel_width: int, pixel_height: int,
              center: tuple, distance: float):
    """
    在画面上绘制检测信息
    """
    if contour is None:
        return frame

    # 绘制边界矩形
    x, y, w, h = cv2.boundingRect(contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # 绘制中心点
    cv2.circle(frame, center, 5, (0, 0, 255), -1)

    # 绘制十字线
    cv2.line(frame, (center[0] - 20, center[1]), (center[0] + 20, center[1]), (0, 0, 255), 1)
    cv2.line(frame, (center[0], center[1] - 20), (center[0], center[1] + 20), (0, 0, 255), 1)

    # 显示距离信息
    distance_text = f"Distance: {distance:.1f} cm"
    cv2.putText(frame, distance_text, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 0), 2)

    # 显示像素尺寸
    size_text = f"Pixel: {pixel_width}x{pixel_height}"
    cv2.putText(frame, size_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 0), 2)

    # 显示中心坐标
    coord_text = f"Center: ({center[0]}, {center[1]})"
    cv2.putText(frame, coord_text, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 0), 2)

    return frame


def draw_status_panel(frame: np.ndarray):
    """
    绘制状态面板
    """
    # 半透明背景
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (280, 120), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    # 显示配置信息
    y_offset = 35
    texts = [
        f"Known Width: {KNOWN_WIDTH} cm",
        f"Focal Length: {FOCAL_LENGTH} px",
        f"Press 'q' to quit",
        f"Press 'c' to calibrate"
    ]

    for text in texts:
        cv2.putText(frame, text, (20, y_offset), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)
        y_offset += 25


def calibrate_focal_length(known_distance: float, pixel_width: float, real_width: float) -> float:
    """
    通过已知距离标定焦距

    公式：focal_length = (pixel_width * known_distance) / real_width

    Args:
        known_distance: 已知距离（厘米）
        pixel_width: 检测到的像素宽度
        real_width: 物体实际宽度（厘米）

    Returns:
        计算得到的焦距
    """
    if pixel_width <= 0:
        return FOCAL_LENGTH

    focal_length = (pixel_width * known_distance) / real_width
    return focal_length


def save_focal_length(focal_length: float):
    """
    保存焦距到配置文件

    Args:
        focal_length: 新的焦距值
    """
    config = {}
    if CONFIG_FILE.exists():
        try:
            with open(CONFIG_FILE, "r", encoding="utf-8") as f:
                config = json.load(f)
        except Exception:
            config = {}

    if "distance_measure" not in config:
        config["distance_measure"] = {}

    config["distance_measure"]["focal_length"] = focal_length

    try:
        with open(CONFIG_FILE, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        print(f"[成功] 焦距已保存到 camera_config.json")
    except Exception as e:
        print(f"[错误] 保存配置文件失败: {e}")


def interactive_calibration(frame: np.ndarray):
    """
    交互式焦距标定
    """
    global FOCAL_LENGTH
    print("\n" + "=" * 50)
    print("焦距标定模式")
    print("=" * 50)
    print(f"1. 将宽度为 {KNOWN_WIDTH} cm 的物体放置在摄像头前")
    print("2. 输入物体到摄像头的实际距离（厘米）")

    try:
        known_distance = float(input("请输入距离（cm）: "))
    except ValueError:
        print("[错误] 无效输入")
        return FOCAL_LENGTH

    # 检测物体
    contour, pixel_width, _, _ = detect_colored_object(frame)

    if contour is None or pixel_width == 0:
        print("[错误] 未检测到物体")
        return FOCAL_LENGTH

    # 计算焦距
    new_focal = calibrate_focal_length(known_distance, pixel_width, KNOWN_WIDTH)
    print(f"[结果] 计算得到的焦距: {new_focal:.1f} px")

    # 询问是否保存
    save = input("是否保存到配置文件? (y/n): ").strip().lower()
    if save == 'y':
        FOCAL_LENGTH = new_focal
        save_focal_length(new_focal)
        print(f"[成功] 焦距已更新为: {FOCAL_LENGTH:.1f} px")

    return new_focal


def main():
    """主函数"""
    # 加载配置
    load_config()

    # 打开摄像头
    cap = cv2.VideoCapture(CAMERA_INDEX)

    try:
        if not cap.isOpened():
            print(f"[错误] 无法打开摄像头 {CAMERA_INDEX}")
            return

        # 设置分辨率
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        print("\n" + "=" * 50)
        print("物体距离测量 - 按颜色检测")
        print("=" * 50)
        print(f"已知物体宽度: {KNOWN_WIDTH} cm")
        print(f"相机焦距: {FOCAL_LENGTH} px")
        print("按 'q' 退出")
        print("按 'c' 进入焦距标定模式")
        print("=" * 50 + "\n")

        while True:
            ret, frame = cap.read()
            if not ret:
                print("[错误] 无法读取摄像头画面")
                break

            # 检测物体
            contour, pixel_width, pixel_height, center = detect_colored_object(frame)

            # 计算距离
            distance = 0.0
            if contour is not None:
                distance = calculate_distance(pixel_width)

            # 绘制信息
            if contour is not None:
                frame = draw_info(frame, contour, pixel_width, pixel_height, center, distance)

            # 绘制状态面板
            draw_status_panel(frame)

            # 显示画面
            cv2.imshow("Distance Measurement", frame)

            # 键盘控制
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('c'):
                # 进入标定模式
                ret, frame = cap.read()
                if ret:
                    interactive_calibration(frame)
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()
        print("\n[结束] 程序已退出")


if __name__ == "__main__":
    main()
