#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
HSV 颜色拾取工具
用于从摄像头画面中拾取颜色，获取 HSV 范围
"""

import cv2
import numpy as np


class ColorPicker:
    """HSV 颜色拾取器"""

    def __init__(self, camera_id: int = 0):
        self.cap = cv2.VideoCapture(camera_id)
        self.color_lower = None
        self.color_upper = None
        self.selected_color = None

    def start(self):
        """启动颜色拾取"""
        cv2.namedWindow("Color Picker")
        cv2.setMouseCallback("Color Picker", self.mouse_callback)

        print("=" * 60)
        print("HSV 颜色拾取工具")
        print("=" * 60)
        print("\n操作说明:")
        print("1. 在摄像头画面中点击想要识别的颜色")
        print("2. 拖动滑块调整 HSV 范围")
        print("3. 按 's' 保存颜色范围")
        print("4. 按 'q' 退出")
        print("\n滑块说明:")
        print("  H: 色调 (0-180)")
        print("  S: 饱和度 (0-255)")
        print("  V: 明度 (0-255)")

        # 创建滑块窗口
        cv2.namedWindow("HSV Controls")

        # 初始 HSV 范围
        init_h = 0
        init_s = 120
        init_v = 70

        # 创建滑块
        cv2.createTrackbar("H Low", "HSV Controls", init_h, 180, self.nothing)
        cv2.createTrackbar("H High", "HSV Controls", init_h + 10, 180, self.nothing)
        cv2.createTrackbar("S Low", "HSV Controls", init_s, 255, self.nothing)
        cv2.createTrackbar("S High", "HSV Controls", 255, 255, self.nothing)
        cv2.createTrackbar("V Low", "HSV Controls", init_v, 255, self.nothing)
        cv2.createTrackbar("V High", "HSV Controls", 255, 255, self.nothing)

        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("无法读取摄像头")
                break

            # 转换到 HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 获取当前滑块值
            h_low = cv2.getTrackbarPos("H Low", "HSV Controls")
            h_high = cv2.getTrackbarPos("H High", "HSV Controls")
            s_low = cv2.getTrackbarPos("S Low", "HSV Controls")
            s_high = cv2.getTrackbarPos("S High", "HSV Controls")
            v_low = cv2.getTrackbarPos("V Low", "HSV Controls")
            v_high = cv2.getTrackbarPos("V High", "HSV Controls")

            # 更新颜色范围
            self.color_lower = (h_low, s_low, v_low)
            self.color_upper = (h_high, s_high, v_high)

            # 创建掩码
            mask = cv2.inRange(hsv, np.array(self.color_lower), np.array(self.color_upper))

            # 形态学操作
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # 查找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 绘制检测结果
            result = frame.copy()

            for contour in contours:
                area = cv2.contourArea(contour)

                if area > 500:
                    # 绘制轮廓
                    cv2.drawContours(result, [contour], -1, (0, 255, 0), 2)

                    # 计算中心
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)

            # 显示当前颜色范围
            cv2.putText(result, f"H: {h_low}-{h_high}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(result, f"S: {s_low}-{s_high}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(result, f"V: {v_low}-{v_high}", (10, 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 显示图像
            cv2.imshow("Color Picker", result)
            cv2.imshow("Mask", mask)

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break
            elif key == ord('s'):
                self.save_color()

        self.cap.release()
        cv2.destroyAllWindows()

    def mouse_callback(self, event, x, y, flags, param):
        """鼠标回调 - 从点击位置获取颜色"""
        if event == cv2.EVENT_LBUTTONDOWN:
            ret, frame = self.cap.read()

            if ret:
                # 获取点击位置的 BGR 颜色
                bgr = frame[y, x]
                b, g, r = bgr

                # 转换到 HSV
                pixel = np.uint8([[[b, g, r]]])
                hsv = cv2.cvtColor(pixel, cv2.COLOR_BGR2HSV)
                h, s, v = hsv[0][0]

                print(f"\n点击位置: ({x}, {y})")
                print(f"BGR: ({b}, {g}, {r})")
                print(f"HSV: ({h}, {s}, {v})")

                # 更新滑块
                cv2.setTrackbarPos("H Low", "HSV Controls", max(0, h - 10))
                cv2.setTrackbarPos("H High", "HSV Controls", min(180, h + 10))
                cv2.setTrackbarPos("S Low", "HSV Controls", max(0, s - 50))
                cv2.setTrackbarPos("S High", "HSV Controls", 255)
                cv2.setTrackbarPos("V Low", "HSV Controls", max(0, v - 50))
                cv2.setTrackbarPos("V High", "HSV Controls", 255)

                self.selected_color = {
                    "bgr": (int(b), int(g), int(r)),
                    "hsv": (int(h), int(s), int(v))
                }

    def save_color(self):
        """保存颜色范围"""
        if self.color_lower and self.color_upper:
            print("\n" + "=" * 60)
            print("当前颜色范围:")
            print(f"  下限: {self.color_lower}")
            print(f"  上限: {self.color_upper}")
            print("\nPython 代码:")
            print(f"  color_lower = {self.color_lower}")
            print(f"  color_upper = {self.color_upper}")
            print("=" * 60)

            if self.selected_color:
                print("\n参考颜色:")
                print(f"  BGR: {self.selected_color['bgr']}")
                print(f"  HSV: {self.selected_color['hsv']}")

    def nothing(self, x):
        """滑块回调（空函数）"""
        pass


def main():
    """主函数"""
    import argparse

    parser = argparse.ArgumentParser(description="HSV 颜色拾取工具")
    parser.add_argument("--camera", type=int, default=0, help="摄像头设备ID")

    args = parser.parse_args()

    picker = ColorPicker(camera_id=args.camera)
    picker.start()


if __name__ == "__main__":
    main()
