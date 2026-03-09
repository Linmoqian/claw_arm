#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
颜色物体检测脚本
支持 HSV 颜色范围配置，检测指定颜色物体并框选
"""

import argparse
import cv2
import numpy as np
import sys

# 语义化彩色输出
class ColorOutput:
    """语义化彩色输出工具"""
    GREEN = '\033[92m'    # 成功
    YELLOW = '\033[93m'   # 警告
    RED = '\033[91m'      # 错误
    CYAN = '\033[96m'     # 交互提示
    BLUE = '\033[94m'     # 高亮和链接
    GRAY = '\033[90m'     # 次要信息
    RESET = '\033[0m'

    @classmethod
    def success(cls, msg):
        print(f"{cls.GREEN}{msg}{cls.RESET}")

    @classmethod
    def warning(cls, msg):
        print(f"{cls.YELLOW}{msg}{cls.RESET}")

    @classmethod
    def error(cls, msg):
        print(f"{cls.RED}{msg}{cls.RESET}")

    @classmethod
    def info(cls, msg):
        print(f"{cls.CYAN}{msg}{cls.RESET}")

    @classmethod
    def highlight(cls, msg):
        print(f"{cls.BLUE}{msg}{cls.RESET}")

    @classmethod
    def secondary(cls, msg):
        print(f"{cls.GRAY}{msg}{cls.RESET}")


# 预设颜色 HSV 范围
COLOR_PRESETS = {
    'green': {
        'name': '绿色',
        'lower': (35, 50, 50),
        'upper': (85, 255, 255)
    },
    'red': {
        'name': '红色',
        'lower': (0, 50, 50),
        'upper': (10, 255, 255),
        'lower2': (160, 50, 50),
        'upper2': (180, 255, 255)
    },
    'blue': {
        'name': '蓝色',
        'lower': (100, 50, 50),
        'upper': (130, 255, 255)
    },
    'yellow': {
        'name': '黄色',
        'lower': (20, 50, 50),
        'upper': (35, 255, 255)
    },
    'orange': {
        'name': '橙色',
        'lower': (10, 50, 50),
        'upper': (20, 255, 255)
    }
}


class ColorDetector:
    """颜色检测器"""

    def __init__(self, h_min, h_max, s_min, s_max, v_min, v_max,
                 h_min2=None, h_max2=None, s_min2=None, s_max2=None, v_min2=None, v_max2=None):
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        # 第二范围（用于红色等跨越色环两端的颜色）
        self.h_min2 = h_min2
        self.h_max2 = h_max2
        self.s_min2 = s_min2
        self.s_max2 = s_max2
        self.v_min2 = v_min2
        self.v_max2 = v_max2

    @property
    def lower(self):
        return np.array([self.h_min, self.s_min, self.v_min])

    @property
    def upper(self):
        return np.array([self.h_max, self.s_max, self.v_max])

    @property
    def lower2(self):
        if self.h_min2 is None:
            return None
        return np.array([self.h_min2, self.s_min2, self.v_min2])

    @property
    def upper2(self):
        if self.h_max2 is None:
            return None
        return np.array([self.h_max2, self.s_max2, self.v_max2])

    @property
    def has_dual_range(self):
        """是否使用双范围检测"""
        return self.h_min2 is not None

    def detect(self, frame, min_area=500):
        """
        检测指定颜色的物体

        Args:
            frame: 输入图像帧
            min_area: 最小面积阈值

        Returns:
            tuple: (检测结果列表, 掩码图像)
        """
        # 转换到 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建掩码
        mask = cv2.inRange(hsv, self.lower, self.upper)

        # 如果有第二范围，合并两个掩码
        if self.has_dual_range:
            mask2 = cv2.inRange(hsv, self.lower2, self.upper2)
            mask = cv2.bitwise_or(mask, mask2)

        # 去噪处理
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area:
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

    def update_hsv(self, h_min, h_max, s_min, s_max, v_min, v_max,
                   h_min2=None, h_max2=None, s_min2=None, s_max2=None, v_min2=None, v_max2=None):
        """更新 HSV 范围"""
        self.h_min = h_min
        self.h_max = h_max
        self.s_min = s_min
        self.s_max = s_max
        self.v_min = v_min
        self.v_max = v_max
        self.h_min2 = h_min2
        self.h_max2 = h_max2
        self.s_min2 = s_min2
        self.s_max2 = s_max2
        self.v_min2 = v_min2
        self.v_max2 = v_max2

    def get_hsv_info(self):
        """获取 HSV 范围信息字符串"""
        info = f"H({self.h_min}-{self.h_max}) S({self.s_min}-{self.s_max}) V({self.v_min}-{self.v_max})"
        if self.has_dual_range:
            info += f" + H({self.h_min2}-{self.h_max2}) S({self.s_min2}-{self.s_max2}) V({self.v_min2}-{self.v_max2})"
        return info


def draw_detections(frame, results, center):
    """
    在帧上绘制检测结果

    Args:
        frame: 输入帧
        results: 检测结果列表
        center: 画面中心点

    Returns:
        绘制后的帧
    """
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

        # 显示坐标
        cv2.putText(display, f"({cx},{cy})", (x, y + h + 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # 绘制中心十字线
    cv2.line(display, (center[0] - 50, center[1]), (center[0] + 50, center[1]), (255, 255, 0), 1)
    cv2.line(display, (center[0], center[1] - 50), (center[0], center[1] + 50), (255, 255, 0), 1)

    return display


def draw_overlay(display, mask, detector, object_count, height, width):
    """
    绘制叠加信息（掩码窗口、HSV信息等）

    Args:
        display: 显示帧
        mask: 掩码图像
        detector: 检测器实例
        object_count: 检测到的物体数量
        height: 帧高度
        width: 帧宽度

    Returns:
        更新后的显示帧
    """
    # 显示检测数量
    cv2.putText(display, f"Objects: {object_count}", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # 显示 HSV 范围
    info = f"HSV: {detector.get_hsv_info()}"
    cv2.putText(display, info, (10, height - 10),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # 显示掩码（小窗口叠加在右上角）
    mask_display = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    mask_display = cv2.resize(mask_display, (160, 120))
    display[0:120, width - 160:width] = mask_display

    # 掩码窗口标签
    cv2.putText(display, "Mask", (width - 150, 20),
               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    return display


def interactive_adjust(detector):
    """交互式调整 HSV 范围"""
    ColorOutput.info("\n调整 HSV 范围 (直接回车保持当前值):")

    try:
        h_min = input(f"  H_min [{detector.h_min}]: ")
        h_max = input(f"  H_max [{detector.h_max}]: ")
        s_min = input(f"  S_min [{detector.s_min}]: ")
        s_max = input(f"  S_max [{detector.s_max}]: ")
        v_min = input(f"  V_min [{detector.v_min}]: ")
        v_max = input(f"  V_max [{detector.v_max}]: ")

        # 更新检测器参数
        new_h_min = int(h_min) if h_min.strip() else detector.h_min
        new_h_max = int(h_max) if h_max.strip() else detector.h_max
        new_s_min = int(s_min) if s_min.strip() else detector.s_min
        new_s_max = int(s_max) if s_max.strip() else detector.s_max
        new_v_min = int(v_min) if v_min.strip() else detector.v_min
        new_v_max = int(v_max) if v_max.strip() else detector.v_max

        detector.update_hsv(new_h_min, new_h_max, new_s_min, new_s_max, new_v_min, new_v_max)
        ColorOutput.success(f"HSV 范围已更新: {detector.get_hsv_info()}")

    except ValueError as e:
        ColorOutput.error(f"输入无效: {e}")


def parse_args():
    """解析命令行参数"""
    parser = argparse.ArgumentParser(description='颜色物体检测脚本')

    parser.add_argument('-c', '--color', type=str, default='green',
                       choices=list(COLOR_PRESETS.keys()),
                       help=f"预设颜色: {list(COLOR_PRESETS.keys())}")

    parser.add_argument('--h-min', type=int, help='H 通道最小值 (0-179)')
    parser.add_argument('--h-max', type=int, help='H 通道最大值 (0-179)')
    parser.add_argument('--s-min', type=int, help='S 通道最小值 (0-255)')
    parser.add_argument('--s-max', type=int, help='S 通道最大值 (0-255)')
    parser.add_argument('--v-min', type=int, help='V 通道最小值 (0-255)')
    parser.add_argument('--v-max', type=int, help='V 通道最大值 (0-255)')

    parser.add_argument('--camera', type=int, default=0, help='摄像头索引 (默认: 0)')
    parser.add_argument('--width', type=int, default=640, help='画面宽度 (默认: 640)')
    parser.add_argument('--height', type=int, default=480, help='画面高度 (默认: 480)')
    parser.add_argument('--min-area', type=int, default=500, help='最小检测面积 (默认: 500)')

    return parser.parse_args()


def main():
    args = parse_args()

    # 获取颜色预设
    preset = COLOR_PRESETS[args.color]
    ColorOutput.highlight(f"颜色检测: {preset['name']}")

    # 确定 HSV 范围（命令行参数优先）
    h_min = args.h_min if args.h_min is not None else preset['lower'][0]
    h_max = args.h_max if args.h_max is not None else preset['upper'][0]
    s_min = args.s_min if args.s_min is not None else preset['lower'][1]
    s_max = args.s_max if args.s_max is not None else preset['upper'][1]
    v_min = args.v_min if args.v_min is not None else preset['lower'][2]
    v_max = args.v_max if args.v_max is not None else preset['upper'][2]

    # 检查是否有第二范围（如红色）
    h_min2 = preset.get('lower2', (None,))[0]
    h_max2 = preset.get('upper2', (None,))[0]
    s_min2 = preset.get('lower2', (None, None))[1] if 'lower2' in preset else None
    s_max2 = preset.get('upper2', (None, None))[1] if 'upper2' in preset else None
    v_min2 = preset.get('lower2', (None, None, None))[2] if 'lower2' in preset else None
    v_max2 = preset.get('upper2', (None, None, None))[2] if 'upper2' in preset else None

    # 创建检测器
    detector = ColorDetector(h_min, h_max, s_min, s_max, v_min, v_max,
                             h_min2, h_max2, s_min2, s_max2, v_min2, v_max2)
    ColorOutput.info(f"HSV 范围: {detector.get_hsv_info()}")

    # 打开摄像头
    ColorOutput.secondary(f"正在打开摄像头 {args.camera}...")
    cap = cv2.VideoCapture(args.camera)

    if not cap.isOpened():
        ColorOutput.error("无法打开摄像头!")
        sys.exit(1)

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center = (width // 2, height // 2)

    ColorOutput.success(f"摄像头已打开: {width}x{height}")
    ColorOutput.secondary("按键: [q] 退出  [s] 调整 HSV 范围")
    ColorOutput.info("开始检测...\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                ColorOutput.warning("读取帧失败")
                break

            # 检测颜色物体
            results, mask = detector.detect(frame, min_area=args.min_area)

            # 绘制检测结果
            display = draw_detections(frame, results, center)

            # 绘制叠加信息
            display = draw_overlay(display, mask, detector, len(results), height, width)

            # 显示窗口
            cv2.imshow("Color Detection", display)

            # 按键处理
            key = cv2.waitKey(30) & 0xFF

            if key == ord('q'):
                ColorOutput.info("退出检测")
                break
            elif key == ord('s'):
                interactive_adjust(detector)

    except KeyboardInterrupt:
        ColorOutput.warning("\n检测被中断")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        ColorOutput.success("程序结束")


if __name__ == "__main__":
    main()
