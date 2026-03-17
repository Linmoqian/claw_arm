"""
detect_color.py - 颜色检测入门示例

功能：使用摄像头检测特定颜色的物体

学习目标：
  - 理解 OpenCV 基础操作
  - 掌握 HSV 颜色空间
  - 学习颜色阈值调整

硬件要求：USB 摄像头

使用方法：
  python examples/basic/detect_color.py
  python examples/basic/detect_color.py --color red
  python examples/basic/detect_color.py --color green
"""
import argparse
import cv2
import numpy as np

# ============ 颜色阈值配置 ============
COLOR_RANGES = {
    "red": [
        ([0, 100, 100], [10, 255, 255]),      # 红色下界
        ([170, 100, 100], [180, 255, 255]),   # 红色上界（跨越 0 度）
    ],
    "green": [
        ([35, 100, 100], [85, 255, 255]),
    ],
    "blue": [
        ([100, 100, 100], [130, 255, 255]),
    ],
    "yellow": [
        ([20, 100, 100], [35, 255, 255]),
    ],
}

# ============ 主程序 ============
def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="颜色检测示例")
    parser.add_argument("--color", default="red", choices=COLOR_RANGES.keys(),
                        help="要检测的颜色 (default: red)")
    parser.add_argument("--camera", type=int, default=0, help="摄像头索引")
    args = parser.parse_args()

    print("=" * 50)
    print(f"  颜色检测 - 检测 {args.color.upper()} 物体")
    print("=" * 50)
    print("\n操作说明:")
    print("  q - 退出程序")
    print("  c - 切换颜色")
    print("=" * 50)

    # 打开摄像头
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("❌ 无法打开摄像头")
        return

    # 设置分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    current_color = args.color

    while True:
        # 读取画面
        ret, frame = cap.read()
        if not ret:
            print("⚠️ 无法读取画面")
            break

        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 创建颜色掩码
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        for lower, upper in COLOR_RANGES[current_color]:
            lower = np.array(lower)
            upper = np.array(upper)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))

        # 形态学操作（去噪）
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # 查找轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 绘制检测结果
        result = frame.copy()
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # 过滤小面积
                x, y, w, h = cv2.boundingRect(contour)
                cx, cy = x + w // 2, y + h // 2

                # 绘制边框和中心点
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)

                # 显示坐标
                cv2.putText(result, f"({cx}, {cy})", (x, y - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 显示画面
        cv2.putText(result, f"Color: {current_color}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("Color Detection", result)

        # 键盘控制
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('c'):
            colors = list(COLOR_RANGES.keys())
            idx = (colors.index(current_color) + 1) % len(colors)
            current_color = colors[idx]
            print(f"切换到: {current_color}")

    # 清理
    cap.release()
    cv2.destroyAllWindows()
    print("\n✅ 程序结束")


if __name__ == "__main__":
    main()
